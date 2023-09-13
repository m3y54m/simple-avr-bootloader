# Simple AVR (ATmega328) Bootloader

To prepare your build environment first read this tutorial:

- [Getting started with AVR programming](https://github.com/m3y54m/start-avr)

## Looking Deeper at the Blinky Program

```c
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
```

**Compile and link the program**

```
cd blinky_test
mkdir build
avr-gcc -Wall -Os -mmcu=atmega328p -std=gnu99 -o build/main.o -c src/main.c
avr-gcc -Wall -Os -mmcu=atmega328p -std=gnu99  -o build/program.elf build/main.o
```

**Sample commands used to generate `.hex` and `.bin` files used for programming the microcontroller:**


Generate `.hex` (Intel Hex format) output file from `.elf` file:

```
avr-objcopy -j .text -j .data -O ihex build/program.elf build/program.hex
```

Generate `.bin` output file from `.elf` file:

```
avr-objcopy -j .text -j .data -O binary build/program.elf build/program.bin
```

Convert `.hex` file to `.bin` file:

```
avr-objcopy -I ihex -O binary build/program.hex build/program.bin
```

Convert `.bin` file to `.hex` file:

```
avr-objcopy -I binary -O ihex build/program.bin build/program.hex
```

This is the contents of th output `.hex` file for the [`blinky_test`](blinky_test) program:



- [Intel HEX File Format](https://microchipdeveloper.com/ipe:sqtp-hex-file-format)


This is the contents of the output `.bin` file for the [`blinky_test`](blinky_test) program:



## Resources

- [ATmega48A/PA/88A/PA/168A/PA/328/P Datasheet](https://ww1.microchip.com/downloads/en/DeviceDoc/ATmega48A-PA-88A-PA-168A-PA-328-P-DS-DS40002061B.pdf)
- [<avr/boot.h>: Bootloader Support Utilities](https://www.nongnu.org/avr-libc/user-manual/group__avr__boot.html)
- [AVR Libc - Memory Sections](https://www.nongnu.org/avr-libc/user-manual/mem_sections.html)
- [AVR109: Using Self Programming on tinyAVR and megaAVR devices](https://www.microchip.com/en-us/application-notes/an1644)
- [Basics to Developing Bootloader for Arduino](https://www.electronicwings.com/arduino/basics-to-developing-bootloader-for-arduino)
- [A simple bootloader example for AVR microcontrollers](https://github.com/radhoo/AVR-bootloader)
- [Simple AVR Bootloader tutorial](https://www.pocketmagic.net/simple-avr-bootloader-tutorial/)
- [How To Write a Simple Bootloader For AVR In C language- (Part 35/46)](https://www.engineersgarage.com/how-to-write-a-simple-bootloader-for-avr-in-c-language-part-35-46/)
- [AVR230: DES Bootloader on tinyAVR and megaAVR devices](https://www.microchip.com/en-us/application-notes/avr230)
- [AVR231: AES Bootloader](https://www.microchip.com/en-us/application-notes/an2462)
- [AN3341 - Basic Bootloader for the AVR MCU DA (AVR DA) Family](https://www.microchip.com/en-us/application-notes/an3341)
- [Basic Bootloader for the AVR-DA Family (Atmel Studio)](https://github.com/microchip-pic-avr-examples/avr128da48-cnano-bootloader-atmel-studio)
- [AN2634 - Bootloader for tinyAVR 0- and 1-series, and megaAVR 0-series](https://www.microchip.com/en-us/application-notes/an2634)
- [AVR Bootloader in C - eine einfache Anleitung](https://www.mikrocontroller.net/articles/AVR_Bootloader_in_C_-_eine_einfache_Anleitung)