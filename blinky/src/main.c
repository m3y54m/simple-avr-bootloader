
// Blinky for AVR (ATmega328P)
// Board connection:
//   PB5 (Arduino Uno D13) ---> LED (with resistor)
#define LED_PIN PB5

#include <stdint.h>
#include <avr/io.h>
#include <util/delay.h>

int main(void)
{
    // Set LED pin as output
    DDRB |= (1U << LED_PIN);

    while (1)
    {
        // Toggle the LED
        PORTB ^= (1U << LED_PIN);
        // Wait for 100 ms
        _delay_ms(100U);
    }

    return 0;
}
