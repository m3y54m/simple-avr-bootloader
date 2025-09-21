
/**
 * @file main.c
 * @brief Bootloader using UART-based Simple UART Protocol (SUP)
 *
 * @warning This code is provided for educational purposes only and is not
 * intended for production use. Use at your own risk. No warranty is provided.
 */

#include "../../simple_uart_protocol/sup.h"
#include <avr/boot.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <stdbool.h>
#include <string.h>
#include <util/atomic.h>
#include <util/delay.h>
#include <util/setbaud.h>

#define SET_BIT(reg, bit) ((reg) |= _BV(bit))
#define CLEAR_BIT(reg, bit) ((reg) &= ~_BV(bit))

#include "../boot_sync.h"
volatile uint32_t boot_sync_flag __attribute__((section(".bootflag"), used));

/**
 * @brief Switch from user application to bootloader mode
 *
 * This function sets a magic value in a special RAM variable and triggers
 * a software reset using the watchdog timer. On reset, the bootloader checks
 * this variable to determine whether to enter bootloader mode or jump to
 * the user application.
 */
void switch_to_bootloader()
{
    // Step 1: Set the magic value in our special uninitialized variable
    boot_sync_flag = FW_UPDATE_INDICATOR_MAGIC;

    // Step 2: Trigger a software reset. Using the watchdog is a reliable way.
    // Use a slightly longer timeout to give bootloader time to disable
    wdt_enable(WDTO_120MS);

    while (1)
    {
    } // Wait for the watchdog to time out and reset the MCU
}

/// Flag to indicate a new SUP frame has been received
volatile bool is_new_frame_received = false;

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
    if ((sup_rx_frame_state != NULL) && (sup_rx_frame_state->parsing_result == SUP_RESULT_SUCCESS))
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
    SET_BIT(UCSR0A, U2X0);
#else
    // It clears the U2X0 bit.
    CLEAR_BIT(UCSR0A, U2X0);
#endif

    // Enable receiver, transmitter, and RX Complete Interrupt
    UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);

    // Set frame format: 8 data bits, 1 stop bit, no parity
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

// On-board LED pin (Arduino Uno: PB5)
#define LED_PIN PB5

static inline void led_init(void)
{
    SET_BIT(DDRB, LED_PIN);    // Set PB5 as output
    CLEAR_BIT(PORTB, LED_PIN); // Ensure LED is off
}

static inline void led_on(void)
{
    SET_BIT(PORTB, LED_PIN);
}

static inline void led_off(void)
{
    CLEAR_BIT(PORTB, LED_PIN);
}

/**
 * @brief Blink LED specified number of times
 * @param count Number of blinks
 * @param period_ms Total period of one blink cycle in milliseconds
 */
static void led_blink(uint8_t count, uint16_t period_ms)
{
    for (uint8_t i = 0; i < count; i++)
    {
        led_on();
        _delay_ms(period_ms / 2); // Short on pulse for visibility
        led_off();
        _delay_ms(period_ms / 2);
    }
}

/**
 * @brief Copy a SUP frame safely
 * @param dest Pointer to destination frame
 * @param src Pointer to source frame
 */
static void copy_sup_frame(sup_frame_t* dest, const sup_frame_t* src)
{
    // Copy the frame data inside an atomic section to
    // avoid race conditions with the RX ISR updating the source.
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        dest->id           = src->id;
        dest->payload_size = src->payload_size;
        // Copy only the valid payload bytes
        if (dest->payload_size > 0)
        {
            memcpy(dest->payload, src->payload, dest->payload_size);
        }
    }
}

/**
 * @brief Process a received SUP frame
 * @param frame Pointer to the received SUP frame
 */
static void process_sup_frame(const sup_frame_t* frame)
{
    switch (frame->id)
    {
        case SUP_ID_CMD_FW_UPDATE:
        {
            // Firmware update command received
            // The rest of the process will be handled by the bootloader
            switch_to_bootloader();
            break;
        }

        default:
            break;
    }
}

/**
 * @brief Main user application function
 */
int main(void)
{
    // Initialize SUP protocol
    sup_rx_frame_state_t sup_rx_frame_state;
    sup_init(&sup_rx_frame_state);

    // Initialize UART
    uart_init();

    // Initialize demo LED
    led_init();

    // Enable global interrupts
    // So that UART RX interrupt can be serviced
    sei();

    while (1)
    {
        // Handle received SUP frames
        if (is_new_frame_received)
        {
            is_new_frame_received = false;

            // Safely copy the received frame for processing.
            // This avoids race conditions, as the original frame may be updated by the ISR.
            sup_frame_t frame;
            copy_sup_frame(&frame, &sup_rx_frame_state.frame);

            // Process the contents of the received frame
            process_sup_frame(&frame);
        }

        // Normal user app operations can be done here
        // Blink LED to indicate user app is running
        led_blink(1, 1000);
    }

    return 0;
}
