/**
 * @file main.c
 * @brief Educational AVR Bootloader with UART-based Firmware Updates
 *
 * @details
 * This bootloader implements a Simple UART Protocol (SUP) for receiving
 * firmware updates over UART. It demonstrates key bootloader concepts:
 *
 * Features:
 * - Boot synchronization with user application via shared memory flag
 * - UART-based firmware update protocol with error handling
 * - Page-wise flash memory programming using AVR self-programming
 * - Interrupt vector relocation for proper operation
 * - Comprehensive error handling and status reporting
 *
 * Boot Process:
 * 1. Check shared memory flag for firmware update request
 * 2. If update requested: Initialize UART, enter update mode
 * 3. If no update: Jump directly to user application
 * 4. Handle SUP protocol frames for firmware data transfer
 * 5. Program received data to flash memory page by page
 * 6. Jump to newly programmed user application
 *
 * Memory Layout (ATmega328P):
 * - User Application: 0x0000 - 0x77FF (30KB)
 * - Bootloader:       0x7800 - 0x7FFF (2KB)
 * - Boot sync flag:   0x08F8 (SRAM, shared with user app)
 *
 * @note Educational implementation - not production ready
 * @warning Improper use may corrupt flash memory or brick your microcontroller
 * @warning Always verify fuse bit settings before flashing bootloader
 *
 * @author m3y54m
 * @version 1.0.0
 * @date 2025
 * @license MIT License
 */

// ============================================================================
// Includes
// ============================================================================

#include <avr/boot.h>      // Boot loader support macros
#include <avr/interrupt.h> // Interrupt handling
#include <avr/io.h>        // AVR I/O definitions
#include <avr/pgmspace.h>  // Program space utilities
#include <avr/wdt.h>       // Watchdog timer
#include <stdbool.h>       // Boolean type support
#include <stddef.h>        // Standard definitions
#include <stdint.h>        // Standard integer types
#include <string.h>        // String manipulation
#include <util/atomic.h>   // Atomic operations
#include <util/delay.h>    // Delay functions
#include <util/setbaud.h>  // UART baud rate macros

#include "boot_sync.h" // Boot synchronization between bootloader and app
#include "sup.h"       // Simple UART Protocol

// ============================================================================
// Configuration and Constants
// ============================================================================

// Ensure MAX_USER_APP_SIZE is defined (should come from build system)
#ifndef MAX_USER_APP_SIZE
#warning "MAX_USER_APP_SIZE not defined, using default value"
#define MAX_USER_APP_SIZE (0x7000) // Default to 28KB (Smallest possible value for ATmega328P)
#endif

// Memory layout constants
#define USER_APP_START_ADDR (0x0000) // User application starts at beginning of flash
#define FLASH_EMPTY_WORD (0xFFFFU)   // Value of erased flash memory
#define WORD_SIZE_BYTES (2U)         // AVR words are 2 bytes

// ============================================================================
// Utility Macros
// ============================================================================

#define SET_BIT(reg, bit) ((reg) |= _BV(bit))
#define CLEAR_BIT(reg, bit) ((reg) &= ~_BV(bit))
#define TOGGLE_BIT(reg, bit) ((reg) ^= _BV(bit))

// ============================================================================
// Global Variables
// ============================================================================

/**
 * @brief Flag indicating a new SUP frame has been received
 * @details Set by UART RX ISR, processed in main loop
 */
volatile bool is_new_frame_received = false;

// Firmware update state machine
typedef enum
{
    FW_STATE_IDLE = 0,  // Not in firmware update mode
    FW_STATE_READY,     // Ready to receive firmware size
    FW_STATE_RECEIVING, // Receiving firmware data
    FW_STATE_FINISHED,  // Firmware update completed
    FW_STATE_ERROR      // Error occurred during update
} fw_update_state_t;

static fw_update_state_t fw_state          = FW_STATE_IDLE;
static uint16_t          fw_expected_size  = 0;        // Total bytes expected
static uint16_t          fw_received_bytes = 0;        // Bytes received so far
static uint16_t          fw_write_address  = 0;        // Current flash write address
static uint8_t           fw_page_buffer[SPM_PAGESIZE]; // Page buffer for flash programming
static uint16_t          fw_page_buffer_index = 0;     // Current position in page buffer

// ============================================================================
// Flash Memory Programming
// ============================================================================

/**
 * @brief Write a single page to flash memory
 * @param page_addr Flash address of page start (must be page-aligned)
 * @param page_data Buffer containing page data, or NULL for erased page
 * @return true on success, false on error
 * @details Programs one SPM_PAGESIZE page to flash using AVR self-programming
 */
static bool write_flash_page(const uint16_t page_addr, const uint8_t* page_data)
{
    // Validate page alignment
    if ((page_addr % SPM_PAGESIZE) != 0)
    {
        return false;
    }

    // Save interrupt state and disable interrupts during critical section
    const uint8_t sreg_backup = SREG;
    cli();

    // Wait for any pending EEPROM operations
    eeprom_busy_wait();

    // Fill temporary page buffer word by word
    for (uint16_t offset = 0; offset < SPM_PAGESIZE; offset += WORD_SIZE_BYTES)
    {
        uint16_t word_data = FLASH_EMPTY_WORD; // Default to erased state

        if (page_data != NULL)
        {
            // Read word from page buffer (little-endian format)
            const uint8_t low_byte  = page_data[offset];
            const uint8_t high_byte = (offset + 1 < SPM_PAGESIZE) ? page_data[offset + 1] : 0xFF;
            word_data               = (uint16_t)low_byte | ((uint16_t)high_byte << 8);
        }

        // Load word into temporary page buffer
        boot_page_fill(page_addr + offset, word_data);
    }

    // Erase flash page
    boot_page_erase(page_addr);
    boot_spm_busy_wait(); // Wait for erase to complete

    // Write page buffer to flash
    boot_page_write(page_addr);
    boot_spm_busy_wait(); // Wait for write to complete

    // Re-enable RWW (Read-While-Write) section access
    boot_rww_enable();

    // Restore interrupt state
    SREG = sreg_backup;

    return true;
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
    // Only DATA frames are expected during firmware update
    // (CMD_FW_UPDATE was already processed in user application)
    if (frame->id != SUP_ID_DATA)
    {
        sup_send_nack(frame->id, (const uint8_t*)&fw_state);
        return;
    }

    switch (fw_state)
    {
        case FW_STATE_IDLE:
        case FW_STATE_FINISHED:
        case FW_STATE_ERROR:
            // Not expecting data in these states
            sup_send_nack(frame->id, (const uint8_t*)&fw_state);
            break;

        case FW_STATE_READY:
            // First DATA frame should contain firmware size (2 bytes, little-endian)
            if (frame->payload_size != 2)
            {
                fw_state = FW_STATE_ERROR;
                sup_send_nack(frame->id, (const uint8_t*)&fw_state);
                break;
            }

            // Extract firmware size
            fw_expected_size = (uint16_t)frame->payload[0] | ((uint16_t)frame->payload[1] << 8);

            // Validate firmware size
            if ((fw_expected_size == 0) || (fw_expected_size > MAX_USER_APP_SIZE))
            {
                fw_state = FW_STATE_ERROR;
                sup_send_nack(frame->id, (const uint8_t*)&fw_state);
                break;
            }

            // Initialize firmware reception
            fw_received_bytes    = 0;
            fw_write_address     = USER_APP_START_ADDR;
            fw_page_buffer_index = 0;
            memset(fw_page_buffer, 0xFF, sizeof(fw_page_buffer)); // Erased state

            fw_state = FW_STATE_RECEIVING;
            sup_send_ack(frame->id, (const uint8_t*)&fw_state);
            break;

        case FW_STATE_RECEIVING:
            // Process firmware data payload
            for (uint8_t i = 0; i < frame->payload_size; i++)
            {
                // Check for overrun
                if (fw_received_bytes >= fw_expected_size)
                {
                    fw_state = FW_STATE_ERROR;
                    sup_send_nack(frame->id, (const uint8_t*)&fw_state);
                    return;
                }

                // Add byte to page buffer
                fw_page_buffer[fw_page_buffer_index++] = frame->payload[i];
                fw_received_bytes++;

                // Check if page buffer is full or firmware complete
                if ((fw_page_buffer_index >= SPM_PAGESIZE) || (fw_received_bytes == fw_expected_size))
                {

                    // Write page to flash
                    if (!write_flash_page(fw_write_address, fw_page_buffer))
                    {
                        fw_state = FW_STATE_ERROR;
                        sup_send_nack(frame->id, (const uint8_t*)&fw_state);
                        return;
                    }

                    // Prepare for next page
                    fw_write_address += SPM_PAGESIZE;
                    fw_page_buffer_index = 0;
                    memset(fw_page_buffer, 0xFF, sizeof(fw_page_buffer));
                }
            }

            // Send acknowledgment
            sup_send_ack(frame->id, (const uint8_t*)&fw_state);

            // Check if firmware update is complete
            if (fw_received_bytes == fw_expected_size)
            {
                fw_state = FW_STATE_FINISHED;

                // Send final acknowledgment
                sup_send_frame(SUP_ID_ACK, NULL, 0);

                // Control flow continues to jump_to_user_app()
            }
            break;
    }
}

// ============================================================================
// Application Control Flow
// ============================================================================

/**
 * @brief Jump to user application
 * @details Performs cleanup and transfers control to user application
 * @note This function does not return
 */
static void __attribute__((noreturn)) jump_to_user_app(void)
{
    // Disable watchdog timer
    wdt_disable();

    // Move interrupt vectors back to application section
    MCUCR = _BV(IVCE); // Enable interrupt vector change
    MCUCR = 0;         // Move vectors to application section (0x0000)

    // Disable all interrupts
    cli();

    // Clean up UART (optional - application will reinitialize)
    UCSR0B = 0; // Disable UART

    // Jump to application start
    // Clear r1 register (expected to be zero by C runtime)
    __asm__ __volatile__("clr r1\n\t"               // Clear register r1 (zero register)
                         "jmp %0"                   // Jump to application
                         :                          // No output operands
                         : "i"(USER_APP_START_ADDR) // Input: application start address
    );

    // Should never reach here
    __builtin_unreachable();
}

// ============================================================================
// Main Application
// ============================================================================

/**
 * @brief Main bootloader entry point
 * @return Never returns (jumps to user application or infinite loop)
 */
int main(void)
{
    // ========================================================================
    // Critical System Initialization
    // ========================================================================

    // Move interrupt vectors to bootloader section before enabling any interrupts
    MCUCR = _BV(IVCE);  // Enable interrupt vector change
    MCUCR = _BV(IVSEL); // Move vectors to bootloader section

    // Disable watchdog immediately after reset to prevent unwanted resets
    // This is critical if watchdog was enabled before reset
    cli();                          // Disable interrupts
    MCUSR &= ~_BV(WDRF);            // Clear watchdog reset flag
    WDTCSR |= _BV(WDCE) | _BV(WDE); // Enable watchdog change
    WDTCSR = 0;                     // Disable watchdog
    sei();                          // Re-enable interrupts for UART

    // ========================================================================
    // Boot Decision Logic
    // ========================================================================

    // Check if user application requested firmware update
    if (is_firmware_update_requested())
    {
        // Initialize SUP protocol
        sup_rx_frame_state_t sup_rx_state;
        sup_init(&sup_rx_state);

        // Initialize UART communication
        uart_init();

        // Set initial firmware update state
        fw_state = FW_STATE_READY;

        // Send acknowledgment that bootloader is ready for firmware update
        sup_send_ack(SUP_ID_CMD_FW_UPDATE, (const uint8_t*)&fw_state);

        // ====================================================================
        // Firmware Update Main Loop
        // ====================================================================

        while (1)
        {
            // Process received SUP frames
            if (is_new_frame_received)
            {
                is_new_frame_received = false;

                // Create local copy of frame for safe processing
                // Prevents race conditions with ISR updating original
                sup_frame_t frame;
                copy_sup_frame(&frame, &sup_rx_state.frame);

                // Process the received SUP frame
                process_sup_frame(&frame);

                // Exit loop if firmware update completed successfully
                if (fw_state == FW_STATE_FINISHED)
                {
                    break;
                }

                // Handle error states
                if (fw_state == FW_STATE_ERROR)
                {
                    // Could implement retry logic or timeout here
                }
            }

            // Small delay to prevent excessive CPU usage
            _delay_ms(1);
        }
    }

    // ========================================================================
    // Jump to User Application
    // ========================================================================

    // Whether we completed firmware update or skipped it entirely,
    // we now jump to the user application
    jump_to_user_app();

    // Should never reach here, but compiler requires return
    return 0;
}