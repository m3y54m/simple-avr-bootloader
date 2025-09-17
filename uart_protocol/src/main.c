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
#include <avr/interrupt.h>
#include <util/delay.h>

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

void LED_Blink(uint8_t message_id)
{
  if (message_id == MSG_ID_FILE_SIZE)
  {
    // Blink LED 3 times fast
    for (uint8_t i = 0; i < 3; i++)
    {
      PORTB &= ~(1 << PB5); // Turn-off LED
      _delay_ms(200);
      PORTB |= 1 << PB5; // Turn-on LED
      _delay_ms(100);
    }
    PORTB &= ~(1 << PB5); // Turn-off LED
  }
  else if (message_id == MSG_ID_FILE_CHUNK)
  {
    // Blink LED 2 times slowly
    for (uint8_t i = 0; i < 2; i++)
    {
      PORTB &= ~(1 << PB5); // Turn-off LED
      _delay_ms(900);
      PORTB |= 1 << PB5; // Turn-on LED
      _delay_ms(100);
    }
    PORTB &= ~(1 << PB5); // Turn-off LED
  }
}
int main(void)
{
  // Configure LED pin as output
  DDRB |= (1 << PB5);

  // Configure USART
  USART_Init();

  // Enable global interrupts
  sei();

  while (1)
  {
    if (is_frame_received)
    {
      // Process the received frame according to message ID
      // The frame is available in payload_buffer[] with length payload_size

      LED_Blink(message_id);

      // Reset is_frame_received flag for the next frame
      is_frame_received = 0;
    }
  }

  return 0;
}