/* Connections

      PB5 ---> LED
*/

// CPU main clock frequency in Hz
#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>

int main(void)
{
  DDRB |= (1 << PB5); // Configure LED pin as output

  while (1)
  {
    PORTB ^= (1 << PB5); // Toggle the LED

    _delay_ms(100); // Wait for 100 ms
  }

  return 0;
}
