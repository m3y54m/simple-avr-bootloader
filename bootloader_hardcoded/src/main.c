/**
 * @brief A hardcoded bootloader for the ATmega328P.
 * @details This bootloader checks if a user application exists at
 * destination_address 0x0000. If not, it flashes a built-in "blinky"
 * application to that destination_address. It then jumps to the application
 * code.
 */

#include <avr/boot.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <util/delay.h>

// --- Hardcoded Application Binary ---

/**
 * @brief Binary code for the 'blinky' app.
 * @details Stored in PROGMEM (Flash memory) to save precious SRAM.
 * The 'blinky' app blinks the LED on PB5 at a 5Hz frequency.
 * Program size: 162 bytes.
 */
static const uint8_t hardcoded_blinky_bin[] PROGMEM = {
    0x0C, 0x94, 0x34, 0x00, 0x0C, 0x94, 0x3E, 0x00, 0x0C, 0x94, 0x3E, 0x00,
    0x0C, 0x94, 0x3E, 0x00, 0x0C, 0x94, 0x3E, 0x00, 0x0C, 0x94, 0x3E, 0x00,
    0x0C, 0x94, 0x3E, 0x00, 0x0C, 0x94, 0x3E, 0x00, 0x0C, 0x94, 0x3E, 0x00,
    0x0C, 0x94, 0x3E, 0x00, 0x0C, 0x94, 0x3E, 0x00, 0x0C, 0x94, 0x3E, 0x00,
    0x0C, 0x94, 0x3E, 0x00, 0x0C, 0x94, 0x3E, 0x00, 0x0C, 0x94, 0x3E, 0x00,
    0x0C, 0x94, 0x3E, 0x00, 0x0C, 0x94, 0x3E, 0x00, 0x0C, 0x94, 0x3E, 0x00,
    0x0C, 0x94, 0x3E, 0x00, 0x0C, 0x94, 0x3E, 0x00, 0x0C, 0x94, 0x3E, 0x00,
    0x0C, 0x94, 0x3E, 0x00, 0x0C, 0x94, 0x3E, 0x00, 0x0C, 0x94, 0x3E, 0x00,
    0x0C, 0x94, 0x3E, 0x00, 0x0C, 0x94, 0x3E, 0x00, 0x11, 0x24, 0x1F, 0xBE,
    0xCF, 0xEF, 0xD8, 0xE0, 0xDE, 0xBF, 0xCD, 0xBF, 0x0E, 0x94, 0x40, 0x00,
    0x0C, 0x94, 0x4F, 0x00, 0x0C, 0x94, 0x00, 0x00, 0x25, 0x9A, 0x90, 0xE2,
    0x85, 0xB1, 0x89, 0x27, 0x85, 0xB9, 0x2F, 0xEF, 0x31, 0xEE, 0x84, 0xE0,
    0x21, 0x50, 0x30, 0x40, 0x80, 0x40, 0xE1, 0xF7, 0x00, 0xC0, 0x00, 0x00,
    0xF3, 0xCF, 0xF8, 0x94, 0xFF, 0xCF};

// --- Configuration Constants ---

// LED blink settings in bootloader
#define LOADING_BLINK_DELAY_MS 1000U
#define LOADING_BLINKS_COUNT 3U
#define FAILURE_BLINK_DELAY_MS 200U
#define FAILURE_BLINKS_COUNT 5U

// The flash memory address where the user application starts
#define USER_APP_START_ADDR 0x0000

// Flash memory constants
#define FLASH_EMPTY_WORD 0xFFFFU
#define WORD_SIZE_BYTES 2U

// --- Helper Macros and Functions for Readability ---

#define SET_BIT(reg, bit) ((reg) |= _BV(bit))
#define CLEAR_BIT(reg, bit) ((reg) &= ~_BV(bit))

static inline void led_init(void)
{
    // Configure LED pin as output
    SET_BIT(DDRB, PB5);
}

static inline void led_on(void)
{
    SET_BIT(PORTB, PB5);
}

static inline void led_off(void)
{
    CLEAR_BIT(PORTB, PB5);
}

/**
 * @brief Blink LED specified number of times
 * @param count Number of blinks
 * @param delay_ms Delay between state changes in milliseconds
 */
static void led_blink(uint8_t count, uint16_t delay_ms)
{
    for (uint8_t i = 0; i < count; i++)
    {
        led_on();
        _delay_ms(100); // Short on pulse for visibility
        led_off();
        // Don't delay after last blink
        if (i < count - 1)
        {
            _delay_ms(delay_ms);
        }
    }
}

/**
 * @brief Check if user application exists in flash memory
 * @return true if application exists, false if flash is empty
 */
static bool user_app_exists(void)
{
    // Check reset vector at address 0x0000
    const uint16_t reset_vector = pgm_read_word(USER_APP_START_ADDR);
    return (reset_vector != FLASH_EMPTY_WORD);
}

/**
 * @brief Writes data from RAM to Flash memory.
 * @details This function handles erasing and writing pages to Flash memory. It
 * safely disables interrupts during the critical write operations.
 * @param destination_address The destination byte address in the user
 * application section. Must be page-aligned.
 * @param source_data Pointer to the source data in RAM.
 * @param source_data_size The size of the source data in bytes. Must be
 * word-aligned.
 * @note This function must be called from the bootloader section.
 * @return true on success, false on error
 */
static bool write_to_flash(const uint32_t destination_address,
                           const uint8_t* source_data,
                           const size_t   source_data_size)
{
    if ((source_data == NULL) || (source_data_size == 0))
    {
        return false;
    }

    if ((destination_address % SPM_PAGESIZE) != 0)
    {
        return false; // Address must be page-aligned
    }

    if ((source_data_size % WORD_SIZE_BYTES) != 0)
    {
        return false; // Size must be word-aligned
    }

    // Save interrupt state and disable interrupts for critical section.
    const uint8_t sreg_backup = SREG;
    cli();

    // Wait for any ongoing EEPROM operations to complete.
    eeprom_busy_wait();

    // Process each page
    for (uint32_t page_addr = destination_address;
         page_addr < (destination_address + source_data_size);
         page_addr += SPM_PAGESIZE)
    {
        // Erase page
        boot_page_erase(page_addr);
        // Wait for the erase operation to complete
        boot_spm_busy_wait();

        // Fill page buffer word by word
        for (uint16_t offset = 0; offset < SPM_PAGESIZE;
             offset += WORD_SIZE_BYTES)
        {
            uint16_t word_data = FLASH_EMPTY_WORD; // Default to erased state

            const uint32_t byte_addr = page_addr + offset - destination_address;
            if (byte_addr < source_data_size)
            {
                // Read word from program buffer (little-endian)
                word_data = source_data[byte_addr];
                if ((byte_addr + 1) < source_data_size)
                {
                    word_data |= (uint16_t)source_data[byte_addr + 1] << 8;
                }
            }

            // Fill word into page buffer
            boot_page_fill(page_addr + offset, word_data);
        }

        // Write page to flash
        boot_page_write(page_addr);
        // Wait for the write operation to complete
        boot_spm_busy_wait();
    }

    // Re-enable RWW section for application execution
    boot_rww_enable();

    // Restore interrupt state
    SREG = sreg_backup;

    return true;
}

/**
 * @brief Load blinky application into the user application section of Flash
 * @return true on success, false on failure
 */
static bool load_blinky_app(void)
{
    led_blink(LOADING_BLINKS_COUNT, LOADING_BLINK_DELAY_MS);

    const size_t app_size = sizeof(hardcoded_blinky_bin);
    uint8_t      app_bin_in_ram[app_size];

    // Copy from PROGMEM to RAM for writing
    for (size_t i = 0; i < app_size; i++)
    {
        app_bin_in_ram[i] = pgm_read_byte(&hardcoded_blinky_bin[i]);
    }

    // Write blinky app to the user application section of Flash
    return write_to_flash(USER_APP_START_ADDR, app_bin_in_ram, app_size);
}

/**
 * @brief Jump to user application
 * @note This function does not return
 */
static void __attribute__((noreturn)) jump_to_user_app(void)
{
    // Disable watchdog timer before jumping
    wdt_disable();

    // Clear any pending interrupts
    cli();

    // Jump to application start address
    // The user application expects r1 to be zero when it starts
    __asm__ __volatile__("clr r1\n\t" // Clear register r1 (zero register)
                         "jmp %0"     // Jump to application
                         :            // Output operands (empty)
                         : "i"(USER_APP_START_ADDR)); // Input operands

    // Should never reach here
    // This is a clean way to indicate non-returning function
    // to the compiler, instead of an infinite loop
    __builtin_unreachable();

    // Bootloader never regains control (unless next reset)
}

/**
 * @brief Main bootloader function
 * @return Should never return
 */
int main(void)
{
    // After a reset, the watchdog timer is enabled with a 16ms timeout
    // if it was enabled before the reset. We disable it here.
    wdt_disable();

    // After the reset always bootloader runs first
    // Here we check if user application exists
    // If yes, we jump to user application
    // If not, we load the hardcoded blinky application, then jump to it
    // If loading fails, we blink LED in an infinite loop
    if (!user_app_exists())
    {
        // Initialize the LED pin
        led_init();

        // Flash is empty, load default application
        if (!load_blinky_app())
        {
            // Loading failed - infinite error blink
            while (true)
            {
               //led_blink(FAILURE_BLINKS_COUNT, FAILURE_BLINK_DELAY_MS);
               //_delay_ms(2000);
            }
        }
    }

    // Jump to user application
    jump_to_user_app();
}