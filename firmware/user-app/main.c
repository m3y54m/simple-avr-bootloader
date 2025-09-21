/**
 * @file main.c
 * @brief User Application with UART-based Simple UART Protocol (SUP)
 *
 * @details
 * This user application demonstrates communication with a bootloader using
 * the Simple UART Protocol (SUP). It provides:
 *
 * Features:
 * - SUP protocol frame reception and processing
 * - Firmware update command handling with bootloader handoff
 * - Boot synchronization via shared memory flag
 * - LED status indication for user application activity
 *
 * Operation:
 * 1. Initialize SUP protocol and UART communication
 * 2. Monitor for incoming SUP frames
 * 3. Process firmware update commands
 * 4. Trigger bootloader mode when update requested
 * 5. Blink LED to indicate normal operation
 *
 * Memory Layout:
 * - Boot sync flag: 0x08F8 (SRAM, shared with bootloader)
 *
 * @note Educational implementation for bootloader interaction
 * @warning This code is provided for educational purposes only and is not
 * intended for production use. Use at your own risk. No warranty is provided.
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
#include <avr/wdt.h>       // Watchdog timer
#include <stdbool.h>       // Boolean type support
#include <string.h>        // String manipulation
#include <util/atomic.h>   // Atomic operations
#include <util/delay.h>    // Delay functions
#include <util/setbaud.h>  // UART baud rate macros

#include "boot_sync.h" // Boot synchronization between bootloader and app
#include "sup.h"       // Simple UART Protocol

// ============================================================================
// Utility Macros
// ============================================================================

#define SET_BIT(reg, bit) ((reg) |= _BV(bit))
#define CLEAR_BIT(reg, bit) ((reg) &= ~_BV(bit))

// ============================================================================
// Global Variables
// ============================================================================

/**
 * @brief Flag indicating a new SUP frame has been received
 * @details Set by UART RX ISR, processed in main loop
 */
volatile bool is_new_frame_received = false;

/**
 * @brief SUP protocol receiver state
 * @details Maintains frame parsing state across multiple byte receptions
 */
static sup_rx_frame_state_t sup_rx_frame_state;

// ============================================================================
// Switching to Bootloader for Firmware Update
// ============================================================================

/**
 * @brief Switch from user application to bootloader mode
 * @details Sets magic value in shared memory and triggers software reset
 * @note System will reset and enter bootloader update mode
 */
static void switch_to_bootloader(void)
{
    // Set magic value in shared memory flag
    set_firmware_update_request_flag();

    // Configure watchdog for software reset
    wdt_enable(WDTO_15MS);

    // Wait for watchdog reset
    while (1)
    {
        // Infinite loop until reset occurs
    }
}

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
 * @brief Process received SUP frame for firmware update
 * @param frame Pointer to received SUP frame
 * @details Implements firmware update state machine
 */
static void process_sup_frame(const sup_frame_t* frame)
{
    switch (frame->id)
    {
        case SUP_ID_CMD_FW_UPDATE:
            // Firmware update command received
            // Handoff to bootloader for update process
            switch_to_bootloader();
            break;

            // Add other frame handlers here as needed

        default:
            // Unknown or unhandled frame type
            break;
    }
}

// ============================================================================
// Main Application
// ============================================================================

/**
 * @brief Main user application entry point
 * @return Never returns (infinite loop)
 */
int main(void)
{
    // ========================================================================
    // System Initialization
    // ========================================================================

    // Initialize SUP protocol state machine
    sup_init(&sup_rx_frame_state);

    // Initialize hardware peripherals
    uart_init();
    led_init();

    // Enable global interrupts for UART reception
    sei();

    // ========================================================================
    // Main Application Loop
    // ========================================================================

    while (1)
    {
        // Process received SUP frames
        if (is_new_frame_received)
        {
            is_new_frame_received = false;

            // Create local copy of frame for safe processing
            // Prevents race conditions with ISR updating original
            sup_frame_t frame;
            copy_sup_frame(&frame, &sup_rx_frame_state.frame);

            // Process the received SUP frame
            process_sup_frame(&frame);
        }

        // Blink LED to indicate application is running
        led_blink(1, 1000);

        // Add other application tasks here
    }

    // Should never reach here
    return 0;
}