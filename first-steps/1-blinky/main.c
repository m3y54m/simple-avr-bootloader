/**
 * @file main.c
 * @brief Simple LED Blinky Application for AVR
 *
 * @details
 * This application demonstrates basic GPIO control on AVR microcontrollers
 * by blinking an LED connected to pin PB5 (Arduino Uno pin D13) at a 
 * frequency of 5Hz (100ms on, 100ms off).
 * 
 * Hardware Requirements:
 * - AVR microcontroller (tested on ATmega328P)
 * - LED connected to PB5 with appropriate current-limiting resistor
 * - Power supply (5V for ATmega328P)
 * 
 * Pin Configuration:
 * - PB5 (PORTB, Pin 5): LED output
 * - Arduino Uno: Digital pin 13 (has built-in LED)
 * - Active high: LED turns on when pin is HIGH
 * 
 * Timing Characteristics:
 * - Toggle Period: 200ms (100ms HIGH + 100ms LOW)
 * - Frequency: 5Hz
 * - Duty Cycle: 50%
 * 
 * Power Consumption Notes:
 * - LED on: ~20mA (depends on LED and resistor)
 * - LED off: ~5mA (MCU running)
 * - Consider sleep modes for battery-powered applications
 * 
 * @note This is a minimal example for educational purposes
 * @note No error handling or watchdog timer management included
 * @note Timing accuracy depends on F_CPU definition and clock source
 * 
 * @warning This code is provided for educational purposes only and is not
 * intended for production use. Use at your own risk. No warranty is provided.
 * @warning Ensure F_CPU matches actual clock frequency for accurate delays
 * 
 * @author m3y54m
 * @version 1.0.0
 * @date 2025
 * @license MIT License
 */

// ============================================================================
// Includes
// ============================================================================

#include <avr/io.h>        // AVR I/O port definitions
#include <util/delay.h>    // Delay functions (_delay_ms)

// ============================================================================
// Macros and Constants
// ============================================================================

/**
 * @brief LED pin definition
 * @details PB5 corresponds to Arduino Uno pin D13 with built-in LED
 */
#define LED_PIN PB5

// ============================================================================
// Main Application
// ============================================================================

/**
 * @brief Main application entry point
 * 
 * @details
 * Initializes the LED pin as an output and enters an infinite loop that
 * toggles the LED state every 100ms, creating a visible blinking effect.
 * 
 * The application uses direct register manipulation for efficiency:
 * - DDRB: Data Direction Register for Port B (1 = output, 0 = input)
 * - PORTB: Output register for Port B (1 = HIGH, 0 = LOW)
 * - ^= : XOR operation for toggling (flips bit state)
 * 
 * Execution Flow:
 * 1. Configure LED pin as output by setting corresponding DDR bit
 * 2. Enter infinite loop
 * 3. Toggle LED pin state using XOR operation
 * 4. Wait 100ms using busy-wait delay
 * 5. Repeat from step 3
 * 
 * @return 0 (never reached - infinite loop)
 * 
 * @note The _delay_ms() function uses CPU cycles for timing
 * @note Actual frequency depends on F_CPU preprocessor definition
 */
int main(void)
{
    // ========================================================================
    // Hardware Initialization
    // ========================================================================
    
    /**
     * Configure LED pin as output
     * Setting bit 5 in DDRB register makes PB5 an output pin
     * All other pins remain in their default state (inputs)
     */
    DDRB |= (1U << LED_PIN);

    // ========================================================================
    // Main Application Loop
    // ========================================================================
    
    /**
     * Infinite loop - fundamental pattern for embedded systems
     * Continuously toggles LED to create blinking effect
     */
    while (1)
    {
        /**
         * Toggle the LED state
         * XOR operation flips the bit: 0->1, 1->0
         * More efficient than reading, inverting, and writing back
         */
        PORTB ^= (1U << LED_PIN);

        /**
         * Delay for 100 milliseconds
         * Creates visible blink rate of 5Hz
         * Uses busy-wait loop - CPU cannot do other tasks during delay
         * 
         * @note Consider timer interrupts for more efficient delays
         * @note Accuracy depends on F_CPU matching actual clock frequency
         */
        _delay_ms(100U);
    }

    // Should never reach here due to infinite loop
    return 0;
}
