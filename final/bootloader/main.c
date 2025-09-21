
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

// Define MAX_USER_APP_SIZE if not defined by the build system
#ifndef MAX_USER_APP_SIZE
// Default to smallest possible size for ATmega328P with 4KB bootloader
#define MAX_USER_APP_SIZE (0x3800 * 2)
#endif

#define SET_BIT(reg, bit) ((reg) |= _BV(bit))
#define CLEAR_BIT(reg, bit) ((reg) &= ~_BV(bit))

#include "../boot_sync.h"
volatile uint32_t boot_sync_flag __attribute__((section(".bootflag"), used));

/**
 * @brief Check if a firmware update is requested by the user application
 * @return true if FW update requested, false to jump to user app
 */
static bool is_fw_update_requested(void)
{
    // Check if the magic value is present in the flag variable
    if (boot_sync_flag == FW_UPDATE_INDICATOR_MAGIC)
    {
        // Magic value found! Clear the flag immediately to prevent a loop
        // if the update is interrupted or fails.
        boot_sync_flag = 0;
        return true;
    }
    else
    {
        // No magic value, so no FW update requested
        return false;
    }
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

// The flash memory address where the user application starts
#define USER_APP_START_ADDR 0x0000

// Flash memory constants
#define FLASH_EMPTY_WORD 0xFFFFU
#define WORD_SIZE_BYTES 2U

/**
 * @brief Write a page to flash memory
 * @details This function writes a full page (SPM_PAGESIZE bytes) to flash memory.
 * If the provided buffer is NULL, it writes an erased page (0xFF).
 * @param write_addr
 * @param page_buffer
 * @return true on success, false on error
 */
static bool write_page_to_flash(const uint16_t write_addr, const uint8_t* page_buffer)
{
    if ((write_addr % SPM_PAGESIZE) != 0)
    {
        return false;
    }

    // Save interrupt state and disable interrupts for critical section.
    uint8_t sreg_backup = SREG;
    cli();

    // Wait for any ongoing EEPROM operations to complete.
    eeprom_busy_wait();

    // Fill page buffer word by word
    for (uint16_t offset = 0; offset < SPM_PAGESIZE; offset += 2)
    {
        uint16_t       word = FLASH_EMPTY_WORD;
        const uint16_t idx  = offset;
        if (page_buffer != NULL)
        {
            // Read word from buffer (little-endian)
            uint8_t low  = page_buffer[idx];
            uint8_t high = page_buffer[idx + 1];
            word         = (uint16_t)low | ((uint16_t)high << 8);
        }
        // Append current word into page buffer
        boot_page_fill(write_addr + offset, word);
    }
    // Erase flash page before writing
    boot_page_erase(write_addr);
    // Wait for the erase operation to complete
    boot_spm_busy_wait();
    // Write page to flash
    boot_page_write(write_addr);
    // Wait for the write operation to complete
    boot_spm_busy_wait();
    // Re-enable RWW section for application execution
    boot_rww_enable();

    // Restore interrupt state
    SREG = sreg_backup;
    return true;
}

/**
 * @brief Jump to user application
 * @note This function does not return
 */
static void __attribute__((noreturn)) jump_to_user_app(void)
{
    // Disable watchdog timer before jumping
    wdt_disable();

    // Move interrupt vectors back to the application section (address 0x0000)
    // before jumping to the user application.
    MCUCR = (1 << IVCE);
    MCUCR = 0;

    // Clear any pending interrupts
    cli();

    // Jump to application start address
    // The user application expects r1 to be zero when it starts
    __asm__ __volatile__("clr r1\n\t"                 // Clear register r1 (zero register)
                         "jmp %0"                     // Jump to application
                         :                            // Output operands (empty)
                         : "i"(USER_APP_START_ADDR)); // Input operands

    // Should never reach here
    // This is a clean way to indicate non-returning function
    // to the compiler, instead of an infinite loop
    __builtin_unreachable();

    // Bootloader never regains control (unless next reset)
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
 * @brief Firmware update states
 * This enum defines the various states that the firmware update process can be in.
 */
typedef enum
{
    FW_STATE_IDLE = 0,
    FW_STATE_READY,
    FW_STATE_RECEIVING,
    FW_STATE_FINISHED
} fw_update_state_t;

static fw_update_state_t fw_state       = FW_STATE_IDLE;
static uint16_t          fw_update_size = 0; // total bytes expected for FW update
static uint16_t          received_bytes = 0;
static uint16_t          write_address  = 0; // destination address in flash
static uint8_t           page_buffer_local[SPM_PAGESIZE];
static uint16_t          page_buffer_index = 0;

/**
 * @brief Process a received SUP frame
 * @param frame Pointer to the received SUP frame
 */
static void process_sup_frame(const sup_frame_t* frame)
{
    // All received frames for FW updates will be DATA frames except the initial CMD_FW_UPDATE
    // which was already processed in user application before entering the bootloader.

    if (frame->id != SUP_ID_DATA)
    {
        // Unexpected frame ID, ignore it
        return;
    }

    switch (fw_state)
    {
        case FW_STATE_FINISHED:
        case FW_STATE_IDLE:
            // Firmware update already finished/aborted, ignore further DATA frames
            sup_send_nack(frame->id, (const uint8_t*)&fw_state);
            break;

        case FW_STATE_READY:
            // First DATA frame must contain the firmware size (2 bytes)
            if (frame->payload_size == 2)
            {
                fw_update_size = (uint16_t)frame->payload[0] | ((uint16_t)frame->payload[1] << 8);
            }
            else
            {
                sup_send_nack(frame->id, (const uint8_t*)&fw_state);
                fw_state = FW_STATE_IDLE;
                return;
            }

            // Size out of bounds, abort update
            if ((fw_update_size == 0) || (fw_update_size > MAX_USER_APP_SIZE))
            {
                sup_send_nack(frame->id, (const uint8_t*)&fw_state);
                fw_state = FW_STATE_IDLE;
                return;
            }

            // Prepare for receiving FW update data
            received_bytes    = 0;
            write_address     = 0x0000;
            page_buffer_index = 0;
            memset(page_buffer_local, 0xFF, sizeof(page_buffer_local));

            sup_send_ack(frame->id, (const uint8_t*)&fw_state);
            fw_state = FW_STATE_RECEIVING;
            break;

        case FW_STATE_RECEIVING:
            // Process FW update data chunks
            for (uint8_t i = 0; i < frame->payload_size; i++)
            {
                if (received_bytes >= fw_update_size)
                {
                    sup_send_nack(frame->id, (const uint8_t*)&fw_state);
                    fw_state = FW_STATE_IDLE;
                    break;
                }

                page_buffer_local[page_buffer_index++] = frame->payload[i];
                received_bytes++;

                if ((page_buffer_index >= SPM_PAGESIZE) || (received_bytes == fw_update_size))
                {
                    if (!write_page_to_flash(write_address, page_buffer_local))
                    {
                        sup_send_nack(frame->id, (const uint8_t*)&fw_state);
                        fw_state = FW_STATE_IDLE;
                        break;
                    }

                    write_address += SPM_PAGESIZE;
                    page_buffer_index = 0;
                    memset(page_buffer_local, 0xFF, sizeof(page_buffer_local));
                }
            }

            fw_state = FW_STATE_RECEIVING;
            sup_send_ack(frame->id, (const uint8_t*)&fw_state);

            if (received_bytes == fw_update_size)
            {
                fw_state = FW_STATE_FINISHED;
                // Final ack (no payload)
                sup_send_frame(SUP_ID_ACK, NULL, 0);
                // Jump to user application
                jump_to_user_app();
            }
            break;
    }
}

/**
 * @brief Main bootloader function
 * @return Should never return
 */
int main(void)
{
    // Move interrupt vectors to the bootloader section.
    // This must be done before sei() is called.
    MCUCR = (1 << IVCE);
    MCUCR = (1 << IVSEL);

    // After a reset, the watchdog timer is enabled with a 16ms timeout
    // if it was enabled before the reset. We should disable it here.

    // Disable watchdog IMMEDIATELY
    cli(); // Disable interrupts

    // Clear WDRF in MCUSR
    MCUSR &= ~(1 << WDRF);

    // Write logical one to WDCE and WDE
    // Keep old prescaler setting to prevent unintended timeout
    WDTCSR |= (1 << WDCE) | (1 << WDE);

    // Turn off WDT
    WDTCSR = 0x00;

    // Re-enable interrupts
    // So that UART RX interrupt can be serviced
    sei();

    // After the reset always bootloader runs first
    // Here we check if user application requests a firmware update
    // If yes, we enter the firmware update mode
    // If not, we jump to user application
    if (is_fw_update_requested())
    {
        // Initialize SUP protocol
        sup_rx_frame_state_t sup_rx_frame_state;
        sup_init(&sup_rx_frame_state);

        // Initialize UART
        uart_init();

        // Continue FW update process
        fw_state = FW_STATE_READY;

        // Acknowledge the FW update request
        sup_send_ack(SUP_ID_CMD_FW_UPDATE, (const uint8_t*)&fw_state);

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
        }
    }

    // Jump to user application
    jump_to_user_app();

    return 0;
}
