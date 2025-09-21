/**
 * @file main.c
 * @brief Educational Example for Simple UART Protocol (SUP) on AVR
 *
 * @details
 * This application demonstrates the usage of the Simple UART Protocol (SUP) library
 * on an AVR microcontroller (e.g., ATmega328P). It listens for incoming SUP
 * frames over UART, processes them, and provides feedback.
 *
 * Features:
 * - Initializes UART for communication.
 * - Uses the SUP library to parse incoming byte streams into frames.
 * - Employs UART RX Complete interrupt for efficient, non-blocking reception.
 * - Implements a safe method for handling data received via ISR in the main loop.
 * - Provides a visual feedback mechanism (LED blink) upon receiving a specific frame.
 * - Responds with an acknowledgment (ACK) and echoes the data for verification.
 *
 * Demo Logic:
 * 1. Initialize SUP, UART, and an LED.
 * 2. Enable global interrupts to allow UART reception.
 * 3. Enter an infinite loop.
 * 4. In the loop, check if a new frame has been received (flag set by ISR).
 * 5. If a new frame is available, safely copy it from the ISR buffer.
 * 6. Send a generic ACK frame back to the sender.
 * 7. If the received frame matches a predefined ID and payload, send the same
 * payload back and blink the LED to confirm successful reception and validation.
 *
 * @note This implementation is for educational purposes and not production-ready.
 * @warning Ensure your F_CPU and BAUD rate settings are correctly configured in your
 * build environment for proper UART communication.
 *
 * @author m3y54m
 * @version 1.0.0
 * @date 2025
 * @license MIT License
 */

// ============================================================================
// Includes
// ============================================================================

#include <avr/interrupt.h> // Interrupt handling
#include <avr/io.h>        // AVR I/O definitions
#include <avr/pgmspace.h>  // Program space utilities
#include <stdbool.h>       // Boolean type support
#include <string.h>        // String manipulation
#include <util/atomic.h>   // Atomic operations
#include <util/delay.h>    // Delay functions
#include <util/setbaud.h>  // UART baud rate macros

#include "sup.h" // Simple UART Protocol

// ============================================================================
// Configuration and Constants
// ============================================================================

// Define the specific SUP frame this demo will react to
#define DEMO_ID SUP_ID_DATA
#define DEMO_PAYLOAD_SIZE 4

/**
 * @brief Demo payload stored in program memory to save SRAM.
 */
static const uint8_t DEMO_PAYLOAD[DEMO_PAYLOAD_SIZE] PROGMEM = {0x44, 0x55, 0x66, 0x77};

// ============================================================================
// Global Variables
// ============================================================================

/**
 * @brief Flag indicating a new SUP frame has been received
 * @details Set by UART RX ISR, processed in main loop
 */
volatile bool is_new_frame_received = false;

// ============================================================================
// Utility Macros
// ============================================================================

#define SET_BIT(reg, bit) ((reg) |= _BV(bit))
#define CLEAR_BIT(reg, bit) ((reg) &= ~_BV(bit))

// ============================================================================
// LED Control Functions
// ============================================================================

/**
 * @brief Initialize LED for status indication
 * @details Configures LED pin as output
 */
static inline void led_init(void)
{
    SET_BIT(DDRB, PB5);    // Set LED pin as output
    CLEAR_BIT(PORTB, PB5); // Start with LED off
}

/**
 * @brief Blink LED for status indication
 * @param count Number of blinks
 * @param delay_ms Delay between blinks in milliseconds
 */
static void led_blink(uint8_t count, uint16_t delay_ms)
{
    for (uint8_t i = 0; i < count; i++)
    {
        SET_BIT(PORTB, PB5); // LED on
        for (uint16_t j = 0; j < delay_ms; j++)
            _delay_ms(1);

        CLEAR_BIT(PORTB, PB5); // LED off
        for (uint16_t j = 0; j < delay_ms; j++)
            _delay_ms(1);
    }
}

// ============================================================================
// SUP Protocol Implementation
// ============================================================================

/**
 * @brief Initialize UART for SUP communication
 * @details Configures UART with build-time baud rate, enables RX interrupt
 */
static void uart_init(void)
{
    // Set baud rate using macros from util/setbaud.h
    UBRR0H = UBRRH_VALUE;
    UBRR0L = UBRRL_VALUE;

// Configure double-speed mode if recommended by setbaud.h
#if USE_2X
    SET_BIT(UCSR0A, U2X0);
#else
    CLEAR_BIT(UCSR0A, U2X0);
#endif

    // Enable receiver, transmitter, and RX Complete Interrupt
    UCSR0B = _BV(RXEN0) | _BV(TXEN0) | _BV(RXCIE0);

    // Frame format: 8 data bits, 1 stop bit, no parity (8N1)
    UCSR0C = _BV(UCSZ01) | _BV(UCSZ00);
}

/**
 * @brief UART Receive Complete Interrupt Service Routine
 * @details Processes each received byte through SUP protocol parser
 * @note Keeps ISR minimal - only processes one byte and sets flag
 */
ISR(USART_RX_vect)
{
    // Read received byte (clears interrupt flag)
    const uint8_t rx_byte = UDR0;

    // Process byte through SUP protocol state machine
    sup_handle_rx_byte(rx_byte);

    // Check if complete frame was received
    sup_rx_frame_state_t* rx_state = sup_get_rx_state();
    if ((rx_state != NULL) && (rx_state->parsing_result == SUP_RESULT_SUCCESS))
    {
        is_new_frame_received = true;
    }
}

void sup_send_byte(const uint8_t tx_byte)
{
    // Wait for transmit buffer to be empty
    while (!(UCSR0A & _BV(UDRE0)))
    {
        // Prevent infinite loop in case of UART issues
        // In production code, add timeout handling here
    }
    UDR0 = tx_byte;
}

/**
 * @brief Safely copy SUP frame data
 * @param dest Destination frame buffer
 * @param src Source frame buffer
 * @details Uses atomic operations to prevent race conditions with ISR
 */
static void copy_sup_frame(sup_frame_t* dest, const sup_frame_t* src)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        dest->id           = src->id;
        dest->payload_size = src->payload_size;

        // Copy only valid payload bytes
        if (dest->payload_size > 0 && dest->payload_size <= SUP_MAX_PAYLOAD_SIZE)
        {
            memcpy(dest->payload, src->payload, dest->payload_size);
        }
    }
}


/**
 * @brief Processes the content of a received SUP frame.
 * @param frame A pointer to the received frame to be processed.
 * @details Checks if the frame matches the predefined demo ID and payload.
 * If it matches, it echoes the payload back and blinks the LED.
 */
static void process_sup_frame(const sup_frame_t* frame)
{
    // Compare the received payload with the expected demo payload stored in Program Memory
    const bool is_demo_payload_match = (frame->payload_size == DEMO_PAYLOAD_SIZE) &&
                                       (memcmp_P(frame->payload, DEMO_PAYLOAD, frame->payload_size) == 0);

    // Check if both the frame ID and payload match our demo criteria
    if (frame->id == DEMO_ID && is_demo_payload_match)
    {
        // Echo the same data frame back to the sender for confirmation
        sup_send_frame(SUP_ID_DATA, frame->payload, frame->payload_size);

        // Blink the LED rapidly as a visual indicator of successful reception
        led_blink(10, 100); // Blink 10 times with 100ms delay
    }
}

// ============================================================================
// Main Application
// ============================================================================

/**
 * @brief Main application entry point.
 * @return Does not return.
 */
int main(void)
{
    // Initialize the SUP protocol library state
    sup_rx_frame_state_t sup_rx_state;
    sup_init(&sup_rx_state);

    // Initialize hardware peripherals
    uart_init();
    led_init();

    // Enable global interrupts to allow the UART RX ISR to function
    sei();

    // --- Main Loop ---

    while (1)
    {
        // Check if the ISR has flagged a new frame arrival
        if (is_new_frame_received)
        {
            is_new_frame_received = false; // Reset the flag immediately

            // Send a generic acknowledgment for any successfully parsed frame.
            // The payload contains the ID of the frame being acknowledged.
            sup_send_ack(SUP_ID_DATA, NULL);

            // Create a local copy of the frame for safe processing. This prevents a
            // race condition where the ISR could start writing a new frame while
            // the main loop is still reading the old one.
            sup_frame_t received_frame;
            copy_sup_frame(&received_frame, &sup_rx_state.frame);

            // Now, process the safely copied frame
            process_sup_frame(&received_frame);
        }
    }

    return 0; // Should never be reached
}