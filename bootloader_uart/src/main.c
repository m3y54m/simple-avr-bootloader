/* Connections

RXD (PD0) <--- TTL Serial Port TX
TXD (PD1) ---> TTL Serial Port RX
      GND <--- TTL Serial Port GND

      PB5 ---> LED
*/

#ifndef __AVR_ATmega328P__
#define __AVR_ATmega328P__
#endif

// CPU main clock frequency in Hz
#define F_CPU 16000000UL
#define BAUD 9600UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/boot.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>

#define PAYLOAD_MAX_SIZE 255 // Maximum supported frame length
#define PAYLOAD_START_INDEX 3
#define FRAME_SOF 0xAA
#define FRAME_EOF 0xBB

// Message IDs
#define MSG_ID_FILE_SIZE 1
#define MSG_ID_FILE_CHUNK 2

volatile uint8_t payload_buffer[PAYLOAD_MAX_SIZE];
volatile uint8_t payload_size = 0;
volatile uint8_t frame_index = 0;
volatile uint8_t message_id = 0;
volatile uint8_t received_checksum = 0;
volatile uint8_t is_frame_received = 0;

void USART_Init(void)
{
  unsigned int ubrr = F_CPU / 16 / BAUD - 1;
  UBRR0H = (unsigned char)(ubrr >> 8);
  UBRR0L = (unsigned char)(ubrr);
  UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0); // Enable RX, TX, and RX Complete Interrupt
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);               // 8-bit data format
}

void USART_Transmit(unsigned char data)
{
  while (!(UCSR0A & (1 << UDRE0)))
  {
    // Wait for the transmit buffer to be empty
  }
  UDR0 = data;
}

ISR(USART_RX_vect)
{
  /*************************************************************************************
  *                                 UART Frame Format                                  *
  **************************************************************************************

      0           1              2             3 .. (N + 2)          (N + 3)   (N + 4)
  +--------+--------------+--------------+-------------------------+----------+--------+
  |  SOF   | Payload Size |  Message-ID  | Payload (Variable size) | Checksum |   EOF  |
  +--------+--------------+--------------+-------------------------+----------+--------+
  |  0xAA  |      N       |  Message-ID  |      Payload Bytes      | Checksum |  0xBB  |
  +--------+--------------+--------------+-------------------------+----------+--------+

  *************************************************************************************/

  static uint8_t calculated_checksum = 0;

  // Read the received byte from USART data register
  uint8_t received_byte = UDR0;

  if (!is_frame_received)
  {
    if (received_byte == FRAME_SOF) // Check for Start-of-Frame
    {

      frame_index = 0;
      payload_size = 0;
    }
    else if (frame_index == 1) // Get the payload size byte
    {
      payload_size = received_byte;
      calculated_checksum = payload_size;

      if (payload_size > PAYLOAD_MAX_SIZE)
      {
        is_frame_received = 0;
        return; // Ignore frame if length exceeds maximum supported length
      }
    }
    else if (frame_index == 2) // Get the message ID byte
    {
      message_id = received_byte;
      calculated_checksum ^= message_id;
    }
    else if (frame_index >= PAYLOAD_START_INDEX && frame_index < payload_size + PAYLOAD_START_INDEX) // Get frame payload (data bytes)
    {
      payload_buffer[frame_index - PAYLOAD_START_INDEX] = received_byte;
      calculated_checksum ^= received_byte;
    }
    else if (frame_index == payload_size + PAYLOAD_START_INDEX) // Get frame checksum
    {
      received_checksum = received_byte;
    }
    else if (frame_index == payload_size + PAYLOAD_START_INDEX + 1) // Check for End-of-Frame
    {
      if (received_checksum == calculated_checksum && received_byte == FRAME_EOF) // Checksum and end-of-frame verification
      {
        is_frame_received = 1; // Set flag to indicate that a complete frame has been received
      }
    }

    frame_index++;
  }
}

volatile uint16_t program_received_bytes = 0;
volatile uint16_t program_total_size = 0;
#define PROGRAM_START_ADDRESS 0x0000

void write_page_to_flash_memory(uint32_t page_address, uint8_t *chunk_buffer, uint8_t chunk_size)
{
  uint8_t sreg_last_state;

  // Disable interrupts.
  sreg_last_state = SREG;
  cli();

  eeprom_busy_wait();

  boot_page_erase(page_address);
  boot_spm_busy_wait(); // Wait until the memory is erased.

  for (uint8_t i = 0; i < chunk_size; i += 2)
  {
    // Set up little-endian word
    uint16_t w = chunk_buffer[i];
    w += chunk_buffer[i + 1] << 8;

    boot_page_fill(page_address + i, w);
  }

  boot_page_write(page_address); // Store buffer in flash page.
  boot_spm_busy_wait();          // Wait until the memory is written.

  // Re-enable RWW-section again. We need this if we want to jump back
  // to the application after bootloading.
  boot_rww_enable();

  // Re-enable interrupts (if they were ever enabled).
  SREG = sreg_last_state;
}

// Interrupts are disabled for bootloader by default
// This function enables interrupts
void enable_interrupts_for_bootloader(void)
{
  // Enable change of Interrupt Vectors
  MCUCR = (1 << IVCE);
  // Move interrupts to Boot Flash section
  MCUCR = (1 << IVSEL);
}

void disable_interrupts_for_bootloader(void)
{
  // Enable change of Interrupt Vectors
  MCUCR = (1 << IVCE);
  // Move interrupts to Program Flash section
  MCUCR &= ~(1 << IVSEL);
}

void jump_to_main_program(void)
{
  wdt_disable();
  
  disable_interrupts_for_bootloader();

  // Jump to the start address of the main program (0x0000)
  asm ("jmp 0x0000");
}

int main(void)
{
  // Check the reset source on the MCU.
  // Get initial value of MCU status register
  char status_register = MCUSR;

  // Clear MCUSR register (Because if we don't clear flags, as long as MCU is powered on reset flags won't be cleared)
  MCUSR = 0;

  // If it is not reset by external reset (RESET pin), jumps to the main program
  // but if it is reset by external reset, continues the bootloader program
  if (!(status_register & (1 << EXTRF)))
  {
    jump_to_main_program();
  }

  enable_interrupts_for_bootloader();

  // Configure LED pin as output
  DDRB |= (1 << PB5);

  // Configure USART
  USART_Init();

  // Enable global interrupts
  sei();

  // Configure watchdog timer for 2 seconds timeout
  // If no data is received through USART in 2 seconds, MCU is reset and jumps to main program
  wdt_enable(WDTO_2S);

  while (1)
  {
    if (is_frame_received)
    {
      // Postpone the watchdog reset (restart the timer of watchdog)
      wdt_reset();

      // Process the received frame according to message ID
      // The frame is available in payload_buffer[] with length payload_size

      if (message_id == MSG_ID_FILE_SIZE)
      {
        // File size received
        program_total_size = (uint16_t)payload_buffer[0] | (uint16_t)payload_buffer[1] << 8;
        program_received_bytes = 0;
      }
      else if (message_id == MSG_ID_FILE_CHUNK)
      {
        // File chunk received
        // Write the chunk data to the program flash memory
        uint32_t current_page_address = PROGRAM_START_ADDRESS + (program_received_bytes / SPM_PAGESIZE) * SPM_PAGESIZE;
        write_page_to_flash_memory(current_page_address, payload_buffer, payload_size);

        program_received_bytes += payload_size;

        if (program_received_bytes == program_total_size)
        {
          // Blink LED 2 times slowly to show that the bootloader program is starting
          for (uint8_t i = 0; i < 2; i++)
          {
            PORTB &= ~(1 << PB5); // Turn-off LED
            _delay_ms(400);
            PORTB |= 1 << PB5; // Turn-on LED
            _delay_ms(100);
          }

          break;
        }
      }

      // Reset is_frame_received flag for the next frame
      is_frame_received = 0;
    }
  }

  jump_to_main_program();

  // Bootloader ends here
}