/* Connections

      PB5 ---> LED
*/

#ifndef __AVR_ATmega328P__
#define __AVR_ATmega328P__
#endif

// CPU main clock frequency in Hz
#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/boot.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

// This array contains the binary code for the `blinky_test` program
// that blinks LED (on PB5) fast (with 5Hz frequency)
// Program size: 162 bytes
uint8_t blinky_test_program_bin[] = {
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

/**
 * @brief Writes a program to a specified memory address.

 * @param address The memory address to write the program to.
 * This address must be PAGE-ALIGNED and valid for writing operations.

 * @param program_buffer A pointer to a buffer containing the program to be written.

 * @param program_buffer_size The size of the program buffer in bytes.
 * This value specifies the amount of data to be written from the `program_buffer`.
 * `program_buffer_size` needs to be a multiple of 2.

 * @retval None.

 * @details
 * The `write_program` function writes the contents of the `program_buffer` to the specified memory address.
 * It is typically used to write firmware or other executable code to embedded devices.

 * @warning Writing to invalid memory locations can lead to system instability or crashes. Ensure that the `address` points to a valid memory region where writing is allowed.
 */
void write_program(const uint32_t address, const uint8_t *program_buffer, const uint32_t program_buffer_size)
{
  // Disable interrupts.
  uint8_t sreg_last_state = SREG;
  cli();

  eeprom_busy_wait();

  // iterate through the program_buffer one page at a time
  for (uint32_t current_page_address = address;
       current_page_address < (address + program_buffer_size);
       current_page_address += SPM_PAGESIZE)
  {
    boot_page_erase(current_page_address);
    boot_spm_busy_wait(); // Wait until the memory is erased.

    // iterate through the page, one word (two bytes) at a time
    for (uint16_t i = 0; i < SPM_PAGESIZE; i += 2)
    {
      uint16_t current_word = 0;
      if ((current_page_address + i) < (address + program_buffer_size))
      {
        // Set up a little-endian word and point to the next word
        current_word = *program_buffer++;
        current_word |= (*program_buffer++) << 8;
      }
      else
      {
        current_word = 0xFFFF;
      }

      boot_page_fill(current_page_address + i, current_word);
    }

    boot_page_write(current_page_address); // Store buffer in a page of flash memory.
    boot_spm_busy_wait();                  // Wait until the page is written.
  }

  // Re-enable RWW-section. We need this to be able to jump back
  // to the application after bootloading.
  boot_rww_enable();

  // Re-enable interrupts (if they were ever enabled).
  SREG = sreg_last_state;
}

int main(void)
{
  // Configure LED pin as output
  DDRB |= (1 << PB5);

  // Check if a user program exists in flash memory
  if (pgm_read_word(0) == 0xFFFF)
  {
    /**********************************************************/
    // NOTE: This part of code is just to check if the MCU is
    //       executing the bootloader or the user program.
    //       You can remove it if you want.
    for (uint8_t i = 0; i < 2; i++)
    {
      // Blink LED 2 times slowly
      PORTB &= ~(1 << PB5); // Turn-off LED
      _delay_ms(2000);
      PORTB |= 1 << PB5; // Turn-on LED
      _delay_ms(100);
    }
    /**********************************************************/

    // Write the binary code of the user program (`blinky_test`) to flash memory at address 0x0000
    write_program(0x00000, blinky_test_program_bin, sizeof(blinky_test_program_bin));
  }

  // Jump to the start address of the user program (0x0000)
  __asm__ __volatile__("jmp 0");

  // Bootloader ends here
}