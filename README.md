# Simple AVR (ATmega328) Bootloader Tutorial 

To prepare your build environment first read this tutorial:

- [Getting started with AVR programming](https://github.com/m3y54m/start-avr)

**⚠️ DISCALIMER ⚠️**

**A "bootloader" is a small program that resides in a dedicated section of the non-volatile memory of a computer. In microcontrollers it is mostly used to facilitate the updating of the main program by utilizing a communication peripheral, thereby eliminating the requirement for an external programmer. In more sophisticated computer systems, a bootloader is mostly employed to pre-configure the system clock and input/output interfaces.**

**With this definition in mind, what follows is not a practical bootloader. Instead, it is a tutorial designed to step-by-step illustrate the process of program compilation and configuration to show how a bootloader can self-program the microcontroller. This bootloader is literally hardcoding the binary data of the program you want to upload (**[**`blinky_test`**](blinky_test)**) in the bootloader itself. With some small changes in code you can modify it to receive binary of the program you want to upload through UART, I2C or SPI. To learn how to write a more sophisticated and secure bootloader study the** [**resources**](#resources).

*DONE:*
- Configure fuse bits settings for bootloader section size and reset vector
- Write a hardcoded blinky program to address `0x0000` of the flash memory and execute it by the bootloader
  
*TODO:*
- Get the program binary through UART

## Project Specifications:

- Compiler: **AVR-GCC**
- MCU: **ATmega328P** (with 16MHz external crystal)
- External Programmer: [**FT232H**](https://github.com/m3y54m/cjmcu-ft232hq-programmer) (you may use any other programmer supported by AVRDUDE)

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

**Useful commands used to generate `.hex` and `.bin` files used for programming the microcontroller:**

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

This is the contents of the output `.hex` file for the [`blinky_test`](blinky_test) program:

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

We can see the exact size of the compiled program using this command:

```
avr-size --format=avr --mcu=atmega328p build/program.elf
```

The result will be something like this:

```
AVR Memory Usage
----------------
Device: atmega328p

Program:     162 bytes (0.5% Full)
(.text + .data + .bootloader)

Data:          0 bytes (0.0% Full)
(.data + .bss + .noinit)
```

This means that the total size of the `blink_test` program is 162 bytes.

## Self Programming the Microcontroller Inside the Bootloader Program

In the [`bootloader`](bootloader) program we put the binary code of the [`blinky_test`](blinky_test) program in an array called `blinky_test_program_bin`.

At the begining of the program LED blinks 2 times slowly to show that the bootloader program is starting.

The function `write_program_pages()` writes the contents of the `blinky_test_program_bin` to the address `0x0000`
of the flash memory of the microcontroller.

Finally the program jumps to the address `0x0000` of the flash memory and runs the `blinky_test` program. Then LED blinks faster as long as microcontroller is not reset or powered off.

```c
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
}
```

## Fuse Bits Setting for Bootloader

Note that in order to configure the microcontroller to start running the bootloader program on RESET you should set `BOOTRST` fuse bit. Also in order to set the bootloader section size in flash memory large enough to ‌hold the bootloader program, we should configure `BOOTSZ1` and `BOOTSZ0` fuse bits.

![image](https://github.com/m3y54m/simple-avr-bootloader/assets/1549028/42900b44-b6f0-4371-b181-afd68e7d34f4)

First we compile the bootloader program. Then we can see size of compiled program using this command:

```
cd bootloader
make
avr-size --format=avr --mcu=atmega328p build/program.elf
```

The result will be something like this:

```
AVR Memory Usage
----------------
Device: atmega328p

Program:     614 bytes (1.9% Full)
(.text + .data + .bootloader)

Data:        162 bytes (7.9% Full)
(.data + .bss + .noinit)
```

This means that the total size of the `bootloader` program is 614 bytes. As you may noted that 162 bytes is exactly the size of `blinky_test` program stored in an array inside the `bootloader` program.

By setting the boot section size of flash memory to 512 words (1024 bytes) we can fit our bootloader program (614 bytes) in it. With this configuration the start address of the boot section becomes `0x3E00` (in words). By knowing that each word is equal to 2 bytes, the start address becomes `0x3E00 * 2 = 0x7C00`.

![image](https://github.com/m3y54m/simple-avr-bootloader/assets/1549028/974ef4eb-b016-4100-91b5-719db5d217f1)

![image](https://github.com/m3y54m/simple-avr-bootloader/assets/1549028/43a6f9f8-abd5-4ee9-9da1-82fe69b287c5)

```
avrdude -c ft232h -p m328p -U lfuse:w:0xFF:m -U hfuse:w:0xDC:m -U efuse:w:0xFD:m
```

[Bootloader fuse bits setting in AVR® Fuse Calculator](https://www.engbedded.com/fusecalc/?P=ATmega328P&V_LOW=0xFF&V_HIGH=0xDC&V_EXTENDED=0xFD&O_HEX=Apply+values)

Adding `-Wl,-section-start=.text=0x7C00` flags to linker options of AVR-GCC makes start address of the bootloader program to be set on the start address of boot section.

```
avr-gcc -Wall -Os -mmcu=atmega328p -std=gnu99 -o build/main.o -c src/main.c
avr-gcc -Wall -Os -mmcu=atmega328p -std=gnu99 -Wl,-section-start=.text=0x7C00 -o build/program.elf build/main.o
```

This is the contents of the output `.hex` file for the [`bootloader`](bootloader) program:

![image](https://github.com/m3y54m/simple-avr-bootloader/assets/1549028/5a172cae-d8e5-4d55-bdfd-b982f43de766)

With this settings every time the microcontroller resets, it first executes the `bootloader`, the `bootloader` writes the `blinky_test` to address `0` of the flash memory and it executes `blinky_test` until next reset.

## Resources

- [ATmega48A/PA/88A/PA/168A/PA/328/P Datasheet](https://ww1.microchip.com/downloads/en/DeviceDoc/ATmega48A-PA-88A-PA-168A-PA-328-P-DS-DS40002061B.pdf)
- [<avr/boot.h>: Bootloader Support Utilities](https://www.nongnu.org/avr-libc/user-manual/group__avr__boot.html)
- [AVR Libc - Memory Sections](https://www.nongnu.org/avr-libc/user-manual/mem_sections.html)
- [AVR109: Using Self Programming on tinyAVR and megaAVR devices](https://www.microchip.com/en-us/application-notes/an1644)
- [Basics to Developing Bootloader for Arduino](https://www.electronicwings.com/arduino/basics-to-developing-bootloader-for-arduino)
- [Optiboot Bootloader for Arduino and Atmel AVR](https://github.com/Optiboot/optiboot)
- [A simple bootloader example for AVR microcontrollers (buggy!)](https://github.com/radhoo/AVR-bootloader)
- [AVR Bootloader in C - eine einfache Anleitung](https://www.mikrocontroller.net/articles/AVR_Bootloader_in_C_-_eine_einfache_Anleitung)
- [How To Write a Simple Bootloader For AVR In C language- (Part 35/46)](https://www.engineersgarage.com/how-to-write-a-simple-bootloader-for-avr-in-c-language-part-35-46/)
- [AVR230: DES Bootloader on tinyAVR and megaAVR devices](https://www.microchip.com/en-us/application-notes/avr230)
- [AVR231: AES Bootloader](https://www.microchip.com/en-us/application-notes/an2462)
- [AN3341 - Basic Bootloader for the AVR MCU DA (AVR DA) Family](https://www.microchip.com/en-us/application-notes/an3341)
- [Basic Bootloader for the AVR-DA Family (Atmel Studio)](https://github.com/microchip-pic-avr-examples/avr128da48-cnano-bootloader-atmel-studio)
- [AN2634 - Bootloader for tinyAVR 0- and 1-series, and megaAVR 0-series](https://www.microchip.com/en-us/application-notes/an2634)
