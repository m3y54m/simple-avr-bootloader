/* Connections

      PB5 ---> LED
*/

// Project common config and definitions
// NOTE: put this before other includes
#include "my/project_config.h"

#include <avr/io.h>
#include <util/delay.h>

int main(void)
{
  // Configure LED pin as output
  DDRB |= (1 << PB5);

  while (1)
  {
    PORTB ^= (1 << PB5); // Toggle the LED

    _delay_ms(100); // Wait for 1 second
  }

  return 0;
}
