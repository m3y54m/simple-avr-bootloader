
/**
 * @file main.c
 * @brief Example usage of the Simple UART Protocol (SUP) on AVR
 *
 * @warning This code is provided for educational purposes only and is not
 * intended for production use. Use at your own risk. No warranty is provided.
 */

#include "sup.h"
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <stdbool.h>
#include <string.h>
#include <util/atomic.h>
#include <util/delay.h>
#include <util/setbaud.h>

/// Flag to indicate a new SUP frame has been received
volatile bool is_new_frame_received = false;

// Demo constants
#define DEMO_LED_PIN PB5
#define DEMO_ID SUP_ID_DATA
#define DEMO_PAYLOAD_SIZE 4
static const uint8_t DEMO_PAYLOAD[DEMO_PAYLOAD_SIZE] PROGMEM = {0x44, 0x55, 0x66, 0x77};

/**
 * @brief UART Receive Complete Interrupt Service Routine.
 * @note Keep the operation inside the ISR as short as possible.
 */
ISR(USART_RX_vect)
{
    // Read the received byte
    const uint8_t rx_byte = UDR0;

    // Process the received byte by SUP protocol
    sup_handle_rx_byte(rx_byte);

    // Check if a complete SUP frame has been received
    sup_rx_frame_state_t* sup_rx_frame_state = sup_get_rx_state();
    if (sup_rx_frame_state != NULL && sup_rx_frame_state->parsing_result == SUP_RESULT_SUCCESS)
    {
        is_new_frame_received = true;
    }
}

/**
 * @brief Send a byte over UART
 * @param tx_byte Byte to send
 * @note This function is declared in sup.h and must be implemented
 * by the user according to the platform's specifics.
 */
void sup_send_byte(const uint8_t tx_byte)
{
    // Wait for the transmit buffer to be empty
    while (!(UCSR0A & (1 << UDRE0)))
        ;
    // Transmit the byte
    UDR0 = tx_byte;
}

static inline void uart_init(void)
{
    // Set baud rate using macros from setbaud.h
    UBRR0H = UBRRH_VALUE;
    UBRR0L = UBRRL_VALUE;

    // Enable double speed mode (U2X) if USE_2X is defined by setbaud.h
#if USE_2X
    // It sets the U2X0 bit in the UART Control and Status Register A.
    UCSR0A |= (1 << U2X0);
#else
    // It clears the U2X0 bit.
    UCSR0A &= ~(1 << U2X0);
#endif

    // Enable receiver, transmitter, and RX Complete Interrupt
    UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);

    // Set frame format: 8 data bits, 1 stop bit, no parity
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

static inline void demo_led_init(void)
{
    DDRB |= (1 << DEMO_LED_PIN);   // Set PB5 as output
    PORTB &= ~(1 << DEMO_LED_PIN); // Ensure LED is off
}

static inline void demo_led_on(void)
{
    PORTB |= (1 << DEMO_LED_PIN);
}

static inline void demo_led_off(void)
{
    PORTB &= ~(1 << DEMO_LED_PIN);
}

static void process_received_frame(const sup_frame_t* frame)
{
    const bool is_demo_payload = (frame->payload_size == DEMO_PAYLOAD_SIZE) &&
                                 (memcmp_P(frame->payload, DEMO_PAYLOAD, frame->payload_size) == 0);

    if (frame->id == DEMO_ID && is_demo_payload)
    {
        // Respond with another DATA frame containing received frame payload
        sup_send_frame(SUP_ID_DATA, frame->payload, frame->payload_size);

        // Blinks the LED rapidly to indicate correct reception
        for (uint8_t i = 0; i < 10; i++)
        {
            demo_led_on();
            _delay_ms(100);
            demo_led_off();
            _delay_ms(100);
        }
    }
}

/**
 * @brief Main application entry point.
 */
int main(void)
{
    // Initialize SUP protocol
    sup_rx_frame_state_t sup_rx_frame_state;
    sup_init(&sup_rx_frame_state);

    // Initialize UART
    uart_init();

    // Initialize demo LED
    demo_led_init();

    // Enable global interrupts
    // So that UART RX interrupt can be serviced
    sei();

    while (1)
    {
        if (is_new_frame_received)
        {
            is_new_frame_received = false;

            // Acknowledge the received message
            // you can receive this message in the Python demo script
            uint8_t ack_payload[] = {sup_rx_frame_state.frame.id};
            sup_send_frame(SUP_ID_ACK, ack_payload, sizeof(ack_payload));

            // Copy the frame data to a local variable inside an atomic
            // section to avoid races with the RX ISR updating the state.
            sup_frame_t frame;
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
            {
                frame.id           = sup_rx_frame_state.frame.id;
                frame.payload_size = sup_rx_frame_state.frame.payload_size;
                // Copy only the valid payload bytes
                if (frame.payload_size > 0)
                {
                    memcpy(frame.payload, sup_rx_frame_state.frame.payload, frame.payload_size);
                }
            }

            // Process the contents of the received frame
            process_received_frame(&frame);
        }
    }
    return 0;
}
