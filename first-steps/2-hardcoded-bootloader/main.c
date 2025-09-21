/**
 * @file main.c
 * @brief Educational AVR Bootloader with Hardcoded Application
 *
 * @details
 * This bootloader demonstrates fundamental bootloader concepts by implementing
 * a simple boot process that can load a hardcoded application into flash memory.
 * It serves as an introduction to AVR self-programming and bootloader development.
 * 
 * Features:
 * - Detection of existing user application in flash
 * - Hardcoded "blinky" application embedded in bootloader
 * - Flash memory programming using AVR boot loader support
 * - LED status indication for bootloader operations
 * - Jump to user application after verification/loading
 * 
 * Boot Process:
 * 1. Disable watchdog timer to prevent unwanted resets
 * 2. Check if user application exists at 0x0000
 * 3. If no application found: Load hardcoded blinky application
 * 4. If loading fails: Enter error indication loop
 * 5. Jump to user application at 0x0000
 * 
 * Memory Layout (ATmega328P):
 * - User Application: 0x0000 - 0x77FF (30KB)
 * - Bootloader:       0x7800 - 0x7FFF (2KB)
 * 
 * LED Indication Patterns:
 * - Loading: 3 blinks, 1 second interval
 * - Failure: 5 rapid blinks, 200ms interval, repeating
 * 
 * Hardcoded Application:
 * - Type: LED blinky at 5Hz on PB5
 * - Size: 162 bytes
 * - Storage: PROGMEM (bootloader flash space)
 * 
 * @note This is an educational implementation demonstrating bootloader basics
 * @note The hardcoded application is stored in bootloader's flash memory
 * @note No communication protocol - application is always the same
 * 
 * @warning This code is provided for educational purposes only and is not
 * intended for production use. Use at your own risk. No warranty is provided.
 * @warning Improper use may corrupt flash memory or brick your microcontroller
 * 
 * @author m3y54m
 * @version 1.0.0
 * @date 2025
 * @license MIT License
 */

// ============================================================================
// Includes
// ============================================================================

#include <avr/boot.h>      // Boot loader support functions
#include <avr/interrupt.h> // Interrupt control
#include <avr/io.h>        // I/O port definitions
#include <avr/pgmspace.h>  // Program memory access
#include <avr/wdt.h>       // Watchdog timer control
#include <stdbool.h>       // Boolean type support
#include <stddef.h>        // NULL and size_t definitions
#include <stdint.h>        // Fixed-width integer types
#include <util/delay.h>    // Delay functions

// ============================================================================
// Configuration Constants
// ============================================================================

#define LOADING_BLINK_DELAY_MS 1000U // Slow blink during loading
#define LOADING_BLINKS_COUNT 3U      // Number of blinks when loading
#define FAILURE_BLINK_DELAY_MS 200U  // Fast blink for error indication
#define FAILURE_BLINKS_COUNT 5U      // Number of blinks for error

#define USER_APP_START_ADDR (0x0000) // User application starts at beginning of flash
#define FLASH_EMPTY_WORD (0xFFFFU)   // Value of erased flash memory
#define WORD_SIZE_BYTES (2U)         // AVR words are 2 bytes

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
// Hardcoded Application Binary
// ============================================================================

/**
 * @brief Hardcoded LED blinky application binary
 *
 * @details
 * This array contains the complete compiled binary of a simple LED blinky
 * application. The application blinks an LED on PB5 at 5Hz frequency.
 *
 * Binary Characteristics:
 * - Size: 162 bytes
 * - Target: ATmega328P
 * - Function: Toggle PB5 every 100ms
 * - Storage: PROGMEM (stored in bootloader's flash space)
 *
 * The binary includes:
 * - Interrupt vector table (26 vectors for ATmega328P)
 * - Initialization code (stack pointer, zero register)
 * - Main application loop with GPIO control
 * - Delay timing loops
 *
 * @note Generated from compiled blinky application
 * @note Must match target MCU architecture
 */
static const uint8_t hardcoded_blinky_bin[] PROGMEM = {
    0x0C, 0x94, 0x34, 0x00, 0x0C, 0x94, 0x3E, 0x00, 0x0C, 0x94, 0x3E, 0x00, 0x0C, 0x94, 0x3E, 0x00, 0x0C, 0x94,
    0x3E, 0x00, 0x0C, 0x94, 0x3E, 0x00, 0x0C, 0x94, 0x3E, 0x00, 0x0C, 0x94, 0x3E, 0x00, 0x0C, 0x94, 0x3E, 0x00,
    0x0C, 0x94, 0x3E, 0x00, 0x0C, 0x94, 0x3E, 0x00, 0x0C, 0x94, 0x3E, 0x00, 0x0C, 0x94, 0x3E, 0x00, 0x0C, 0x94,
    0x3E, 0x00, 0x0C, 0x94, 0x3E, 0x00, 0x0C, 0x94, 0x3E, 0x00, 0x0C, 0x94, 0x3E, 0x00, 0x0C, 0x94, 0x3E, 0x00,
    0x0C, 0x94, 0x3E, 0x00, 0x0C, 0x94, 0x3E, 0x00, 0x0C, 0x94, 0x3E, 0x00, 0x0C, 0x94, 0x3E, 0x00, 0x0C, 0x94,
    0x3E, 0x00, 0x0C, 0x94, 0x3E, 0x00, 0x0C, 0x94, 0x3E, 0x00, 0x0C, 0x94, 0x3E, 0x00, 0x11, 0x24, 0x1F, 0xBE,
    0xCF, 0xEF, 0xD8, 0xE0, 0xDE, 0xBF, 0xCD, 0xBF, 0x0E, 0x94, 0x40, 0x00, 0x0C, 0x94, 0x4F, 0x00, 0x0C, 0x94,
    0x00, 0x00, 0x25, 0x9A, 0x90, 0xE2, 0x85, 0xB1, 0x89, 0x27, 0x85, 0xB9, 0x2F, 0xEF, 0x31, 0xEE, 0x84, 0xE0,
    0x21, 0x50, 0x30, 0x40, 0x80, 0x40, 0xE1, 0xF7, 0x00, 0xC0, 0x00, 0x00, 0xF3, 0xCF, 0xF8, 0x94, 0xFF, 0xCF};

// ============================================================================
// Flash Memory Functions
// ============================================================================

/**
 * @brief Check if user application exists in flash
 *
 * @details
 * Verifies if a valid user application is present by checking the reset
 * vector at address 0x0000. An empty flash location contains 0xFFFF.
 * A programmed application will have a jump instruction (typically 0x940C).
 *
 * @return true if application exists, false if flash is empty
 *
 * @note Only checks the reset vector, not the entire application
 * @note Assumes valid application has non-0xFFFF reset vector
 */
static bool user_app_exists(void)
{
    /**
     * Read the first word of user application area (reset vector)
     * If flash is empty, it will read 0xFFFF
     * If application exists, it will contain a jump instruction
     */
    uint16_t app_start_word = pgm_read_word(USER_APP_START_ADDR);
    return (app_start_word != FLASH_EMPTY_WORD);
}

/**
 * @brief Load hardcoded blinky application into flash memory
 *
 * @details
 * Programs the hardcoded blinky application from PROGMEM into the user
 * application area of flash memory. The process includes:
 * 1. Reading application size
 * 2. Programming data page by page
 * 3. Filling temporary page buffer
 * 4. Erasing and writing each page
 *
 * Programming sequence for each page:
 * - Fill temporary page buffer with data
 * - Erase page in flash
 * - Write buffer contents to flash
 * - Wait for completion
 *
 * @return true if loading successful, false on error
 *
 * @note Uses SPM (Store Program Memory) instructions
 * @note Must be executed from bootloader section
 * @note Interrupts should be disabled during programming
 */
static bool load_blinky_app(void)
{
    /**
     * Calculate application size from hardcoded binary array
     */
    uint16_t app_size = sizeof(hardcoded_blinky_bin);

    /**
     * Calculate number of pages to write
     * Round up to ensure all bytes are written
     */
    uint8_t num_pages = (app_size + SPM_PAGESIZE - 1) / SPM_PAGESIZE;

    /**
     * Program each page sequentially
     */
    for (uint8_t page = 0; page < num_pages; page++)
    {
        // Calculate page start address
        uint16_t page_address = page * SPM_PAGESIZE;

        /**
         * Fill temporary page buffer with application data
         * Buffer is filled word by word (2 bytes at a time)
         */
        for (uint16_t i = 0; i < SPM_PAGESIZE; i += WORD_SIZE_BYTES)
        {
            // Calculate source offset in hardcoded binary
            uint16_t byte_offset = page_address + i;
            uint16_t word_data;

            /**
             * Read word from PROGMEM, handling last page padding
             */
            if (byte_offset < app_size - 1)
            {
                // Read complete word from binary
                word_data = pgm_read_word(&hardcoded_blinky_bin[byte_offset]);
            }
            else if (byte_offset == app_size - 1)
            {
                // Last byte - pad with 0xFF for unprogrammed byte
                word_data = pgm_read_byte(&hardcoded_blinky_bin[byte_offset]) | 0xFF00;
            }
            else
            {
                // Beyond application data - fill with 0xFFFF
                word_data = FLASH_EMPTY_WORD;
            }

            // Write word to temporary page buffer
            boot_page_fill(page_address + i, word_data);
        }

        /**
         * Erase the target page in flash memory
         * This sets all bits in the page to 1 (0xFF bytes)
         */
        boot_page_erase(page_address);
        boot_spm_busy_wait(); // Wait for erase to complete

        /**
         * Write the temporary page buffer to flash memory
         * This programs the erased page with new data
         */
        boot_page_write(page_address);
        boot_spm_busy_wait(); // Wait for write to complete

        /**
         * Re-enable RWW section
         * Allows CPU to read from flash while programming
         */
        boot_rww_enable();

        /**
         * Visual feedback - blink LED after each page
         * Indicates programming progress
         */
        led_blink(1, 100);
    }

    // Programming completed successfully
    return true;
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

    // After reset, bootloader always runs first (due to BOOTRST fuse)
    // Check if user application exists:
    // - If yes: Jump directly to application
    // - If no: Load hardcoded application first
    if (!user_app_exists())
    {
        // ====================================================================
        // Application Loading Mode
        // ====================================================================

        // Initialize LED for visual feedback during loading
        led_init();

        // Attempt to load the hardcoded blinky application
        // Flash is empty, so we need to program it
        if (!load_blinky_app())
        {
            // ================================================================
            // Error Handling - Loading Failed
            // ================================================================

            // Loading failed - enter infinite error indication loop
            // This is a critical failure that requires user intervention
            // (power cycle or external programmer)
            while (true)
            {
                // Rapid blinking indicates error condition
                led_blink(FAILURE_BLINKS_COUNT, FAILURE_BLINK_DELAY_MS);
                _delay_ms(2000); // Pause between error indication sequences
            }
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