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

// This array contains the binary code of the LED (on PB5) fast blinking program
// program size: 162 bytes
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

#define BLINKY_PROGRAM_SIZE_IN_BYTES sizeof(blinky_test_program_bin)
#define BLINKY_PROGRAM_NUMBER_OF_PAGES ((BLINKY_PROGRAM_SIZE_IN_BYTES / SPM_PAGESIZE) + ((BLINKY_PROGRAM_SIZE_IN_BYTES % SPM_PAGESIZE == 0) ? 0 : 1))

void write_program_pages(uint32_t first_page_address, uint8_t *program_buffer)
{
  uint16_t b;
  uint16_t p;
  uint16_t current_page_size;
  uint32_t current_page_address;
  uint8_t sreg_last_state;

  // Disable interrupts.
  sreg_last_state = SREG;
  cli();

  eeprom_busy_wait();

  for (p = 0; p < BLINKY_PROGRAM_NUMBER_OF_PAGES; p++)
  {
    // Calculate current page size in bytes
    if (p == BLINKY_PROGRAM_NUMBER_OF_PAGES - 1)
    {
      // Last page size
      current_page_size = BLINKY_PROGRAM_SIZE_IN_BYTES - SPM_PAGESIZE * (BLINKY_PROGRAM_NUMBER_OF_PAGES - 1);
    }
    else
    {
      // Other page sizes
      current_page_size = SPM_PAGESIZE;
    }

    current_page_address = first_page_address + p * SPM_PAGESIZE;

    boot_page_erase(current_page_address);
    boot_spm_busy_wait(); // Wait until the memory is erased.

    for (b = 0; b < current_page_size; b += 2)
    {
      // Set up little-endian word
      uint16_t w = *program_buffer++;
      w += (*program_buffer++) << 8;

      boot_page_fill(current_page_address + b, w);
    }

    boot_page_write(current_page_address); // Store buffer in flash page.
    boot_spm_busy_wait();                  // Wait until the memory is written.
  }

  // Re-enable RWW-section again. We need this if we want to jump back
  // to the application after bootloading.
  boot_rww_enable();

  // Re-enable interrupts (if they were ever enabled).
  SREG = sreg_last_state;
}

int main(void)
{
  // Configure LED pin as output
  DDRB |= (1 << PB5);

  // Blink LED 2 times slowly to show that the bootloader program is starting
  for (uint8_t i = 0; i < 2; i++)
  {
    PORTB &= ~(1 << PB5); // Turn-off LED
    _delay_ms(2000);
    PORTB |= 1 << PB5; // Turn-on LED
    _delay_ms(100);
  }

  // Write the binary code of the blinky program to flash memory at address 0x0000
  write_program_pages(0x00000, blinky_test_program_bin);

  // Jump to the start address of the blinky program (0x0000)
  asm("jmp 0");

  // Bootloader ends here
}