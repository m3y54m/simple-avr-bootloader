/**
 * @file main.c
 * @brief Blinky application
 *
 * @details This application blinks an LED connected to pin PB5
 * (Arduino Uno D13) at a 5Hz frequency.
 *
 * @warning This code is provided for educational purposes only and is not
 * intended for production use. Use at your own risk. No warranty is provided.
 */

#include <avr/io.h>
#include <stdint.h>
#include <util/delay.h>

#define LED_PIN PB5

int main(void)
{
    // Configure LED pin as output
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
