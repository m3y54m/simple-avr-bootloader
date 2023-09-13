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

![image](https://github.com/m3y54m/simple-avr-bootloader/assets/1549028/43bb30ea-5cfd-4bab-9002-a3bfe7651469)


- [Intel HEX File Format](https://microchipdeveloper.com/ipe:sqtp-hex-file-format)


This is the contents of the output `.bin` file for the [`blinky_test`](blinky_test) program (shown in a Hex Viewer):

![image](https://github.com/m3y54m/simple-avr-bootloader/assets/1549028/3f6d5e52-2eb7-48e1-b77b-d7f25770e6fb)

The contents of binary file are exactly the bytes that will be programmed
into the flash memory of the microcontroller (each byte is shown as a 2-digit hexadecimal number).

```
0C 94 34 00 0C 94 3E 00 0C 94 3E 00 0C 94 3E 00
0C 94 3E 00 0C 94 3E 00 0C 94 3E 00 0C 94 3E 00
0C 94 3E 00 0C 94 3E 00 0C 94 3E 00 0C 94 3E 00
0C 94 3E 00 0C 94 3E 00 0C 94 3E 00 0C 94 3E 00
0C 94 3E 00 0C 94 3E 00 0C 94 3E 00 0C 94 3E 00
0C 94 3E 00 0C 94 3E 00 0C 94 3E 00 0C 94 3E 00
0C 94 3E 00 0C 94 3E 00 11 24 1F BE CF EF D8 E0
DE BF CD BF 0E 94 40 00 0C 94 4F 00 0C 94 00 00
25 9A 90 E2 85 B1 89 27 85 B9 2F EF 31 EE 84 E0
21 50 30 40 80 40 E1 F7 00 C0 00 00 F3 CF F8 94
FF CF
```




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
