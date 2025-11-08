# Simple AVR Bootloader Tutorial 

This repository is designed to teach you, step by step, how bootloaders work and how to write your own customized bootloader for an AVR microcontroller.

If you are new to AVR programming stop here and first read this: [Getting started with AVR programming](https://github.com/m3y54m/start-avr)

> [!CAUTION]
> The code and materials in this repository are provided for **educational purposes only**. They are **not intended for production use** and may lack necessary safety, security, or efficiency features. Use at your own risk.

## What is a Bootloader?

A **bootloader** is a small, specialized program that runs when your microcontroller first powers up or resets. Unlike your main application (which does the actual work), the bootloader has one primary job: decide what happens next. It can either jump to your main application to run normally, or enter a special mode to update the firmware over a communication interface like UART, USB, or Ethernet - no external programmer needed!

Think of it as the gatekeeper between power-on and your actual program. This separation is crucial because it provides a safe way to update your device's firmware remotely, even if the main application gets corrupted.


## Why Do We Need Bootloaders?

Imagine you've deployed hundreds of devices in the field and discover a bug in your firmware. Without a bootloader, you'd need to physically retrieve each device and reprogram it with a dedicated programmer (like an ISP programmer). With a bootloader, you can update the firmware remotely over UART, SPI, I2C, or even wirelessly!


## How Does an AVR Bootloader Work?

AVR microcontrollers have a clever feature called **self-programming** - they can modify their own flash memory while running. Here's the magic:

1. **Memory Layout**: Flash memory is divided into two sections:
   - **Application Section** (`0x0000` ~ `[Boot Reset Vector] - 1`): Your main program lives here
   - **Boot Section** (`[Boot Reset Vector]` ~ `[Flash End Address]`): The bootloader lives here (protected from accidental overwrites)

2. **Boot Process**: When the MCU resets, it can be configured (via fuse bits) to start from the bootloader section instead of address 0x0000.

3. **Decision Time**: The bootloader runs first and decides:
   - Should I update the firmware? → Enter programming mode
   - Everything looks good? → Jump to the user application

4. **Self-Programming**: Using special SPM (Store Program Memory) instructions, the bootloader can erase and write the application section page by page.


## What's in This Repository?

This repo demonstrates AVR bootloader concepts through progressive examples:

- **Blinky** (`first-steps/1-blinky`): Basic LED blinking to understand the foundation
- **Hardcoded Bootloader** (`first-steps/2-hardcoded-bootloader`): A bootloader that can install a pre-compiled application
- **UART Protocol** (`first-steps/3-simple-uart-protocol`): Communication protocol for bootloader-host interaction
- **Complete Bootloader** (`firmware/`): Full implementation with UART-based firmware updates

The final implementation uses a **Simple UART Protocol (SUP)** for reliable communication between a PC host and the microcontroller, allowing real firmware updates over a simple serial connection.


## Project Structure

```
.
├── .vscode
│   └── cmake-kits.json
├── CMakeLists.txt
├── firmware
│   ├── bootloader
│   │   └── main.c
│   ├── user-app
│   │   └── main.c
│   ├── boot_sync.h
│   └── CMakeLists.txt
├── firmware-upload-tool
│   ├── blinky.bin
│   ├── demo_sup.py
│   ├── main.py
│   └── sup.py
├── first-steps
│   ├── 1-blinky
│   │   ├── CMakeLists.txt
│   │   └── main.c
│   ├── 2-hardcoded-bootloader
│   │   ├── CMakeLists.txt
│   │   └── main.c
│   └── 3-simple-uart-protocol
│       ├── mcu
│       │   ├── CMakeLists.txt
│       │   ├── main.c
│       │   ├── sup.c
│       │   └── sup.h
│       └── pc
│           └── main.py
└── toolchain-avr.cmake
```

**Root Directory**

- `.vscode/cmake-kits.json` — VS Code CMake kits configuration (helps the CMake extension find the AVR-GCC toolchain)
- `CMakeLists.txt` — Main CMake project file (sets MCU, clock, programmer and other build options)
- `toolchain-avr.cmake` — CMake toolchain file that configures the AVR cross-compiler and tools

**`first-steps/` - Progressive Learning Examples**

This folder contains guided, minimal examples to get started quickly:

- `1-blinky/` — A basic "blinky" example (LED toggle) to verify your toolchain setup
- `2-hardcoded-bootloader/` — A bootloader that contains a hardcoded application binary and demonstrates self-programming flash memory
- `3-simple-uart-protocol/` — Example for the Simple UART Protocol (SUP):
  - `mcu/` — C code that runs on the AVR (SUP implementation)
  - `pc/` — Python script to run on the host PC that speaks SUP to the MCU

**`firmware/` - Complete Bootloader Implementation**

- `bootloader/main.c` — Full bootloader source with UART communication and firmware update capability
- `user-app/main.c` — Sample user application that can trigger bootloader updates
- `boot_sync.h` — Header that defines the shared memory/synchronization interface between the user app and the bootloader
- `CMakeLists.txt` — CMake file that builds both the bootloader and the user application with proper memory layout

**`firmware-upload-tool/` - PC-Side Communication Tools**

- `sup.py` — Python implementation of the Simple UART Protocol (SUP) used to communicate with the bootloader
- `main.py` — CLI/driver script that uses `sup.py` to upload firmware
- `demo_sup.py` — Demonstration script showing how to use `sup.py`
- `blinky.bin` — Pre-compiled blinky binary used for testing the upload process


## Learning Path: From Simple to Advanced

This repository is designed as a step-by-step learning experience. Each example builds on the previous concepts:

**1. Start Here: `first-steps/1-blinky`**

**Goal**: Verify your development environment works
- Simple LED blinking program
- Learn the compilation process from source to binary
- Understand memory usage and program structure

**2. Next: `first-steps/2-hardcoded-bootloader`** 

**Goal**: Understand self-programming fundamentals
- See how bootloaders can modify flash memory
- Learn about page-based flash programming
- Observe the boot process and application handoff

**3. Then: `first-steps/3-simple-uart-protocol`**

**Goal**: Learn bootloader communication
- Implement reliable serial communication protocol
- Understand frame-based data transfer
- See bidirectional host-MCU communication

**4. Finally: `firmware/` - Complete Implementation**

**Goal**: Build a production-ready bootloader system
- UART-based firmware updates
- Bootloader-application synchronization
- Real-world error handling and recovery


## Hardware and Software Requirements

- **Compiler**: AVR-GCC toolchain
- **Target MCU**: ATmega328P with 16MHz external crystal (Arduino Uno, Nano, or compatible ATmega328P board)
- **Programmer**: USBasp (or any AVRDUDE-compatible programmer)


## Getting Started: Understanding the Build Process

Let's walk through the complete journey from C source code to a binary that runs on your microcontroller. This process is fundamental to understanding how bootloaders work, as they manipulate these same binary files.

### Step 1: Writing Your First AVR Program

We'll start with the simplest possible AVR program - a blinking LED. This helps verify your toolchain is working and demonstrates the basic structure:

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

This program simply toggles pin PB5 (Arduino pin 13) every 100ms, creating a visible blink.

### Step 2: Compilation - Source to Object Code

The first step is **compilation** - converting your C source code into machine code that the AVR processor understands:

```bash
cd first-steps/1-blinky
mkdir build
avr-gcc -Wall -Os -mmcu=atmega328p -std=c11 -o build/main.o -c main.c
```

**What this does**: 
- `-mmcu=atmega328p` tells the compiler to generate code specifically for the ATmega328P
- `-Os` optimizes for size (crucial for microcontrollers with limited flash)
- `-c` compiles to object code without linking
- The output `main.o` contains machine code but isn't executable yet

### Step 3: Linking - Creating the Executable

**Linking** combines your object files and creates the final executable with proper memory addresses:

```bash
avr-gcc -Wall -Os -mmcu=atmega328p -std=c11 -o build/program.elf build/main.o
```

**What this does**:
- Links your object file with AVR standard libraries
- Resolves all function calls and memory addresses
- Creates an ELF (Executable and Linkable Format) file
- The `.elf` file contains your complete program with debugging information

### Step 4: Generating Programming Files

The ELF file contains extra information that the microcontroller doesn't need. We extract just the executable code in formats suitable for programming:

**Intel HEX format** (human-readable, includes addresses):
```bash
avr-objcopy -j .text -j .data -O ihex build/program.elf build/program.hex
```

**Raw binary format** (pure machine code bytes):
```bash
avr-objcopy -j .text -j .data -O binary build/program.elf build/program.bin
```

**What these do**:
- `-j .text -j .data` extracts only the program code and initialized data sections
- Intel HEX includes address information and checksums for reliable programming
- Binary format is the raw bytes that will sit in flash memory
- Most programmers use HEX format, but bootloaders often work with binary

### Step 5: Understanding Your Program's Memory Footprint

Before programming, let's see how much space our program uses:

```bash
avr-size --format=avr --mcu=atmega328p build/program.elf
```

**Typical output**:
```
AVR Memory Usage
----------------
Device: atmega328p

Program:     162 bytes (0.5% Full)
(.text + .data + .bootloader)

Data:          0 bytes (0.0% Full)
(.data + .bss + .noinit)
```

**What this tells us**:
- Our blinky program is only 162 bytes - tiny!
- It uses 0.5% of the ATmega328P's 32KB flash memory
- No RAM is used for variables (data = 0 bytes)

### Step 6: Examining the Generated Machine Code

Let's look at what the compiler actually generated. The raw binary contains these hex bytes:

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

**Understanding the structure**:
- The first several bytes are the **interrupt vector table** - jump instructions for different interrupts
- `0C 94 34 00` at the very beginning is the **reset vector** - where the processor starts when powered on
- This points to address 0x0034 (byte address 0x0068), where your main program begins
- The repeated `0C 94 3E 00` entries point to a default interrupt handler

**File format conversion** (useful for bootloader development):
```bash
# Convert .hex file to .bin file
avr-objcopy -I ihex -O binary build/program.hex build/program.bin

# Convert .bin file to .hex file
avr-objcopy -I binary -O ihex build/program.bin build/program.hex
```

Here is the content of the output `.bin` file for the blinky program (shown in a Hex Viewer):

![image](https://github.com/m3y54m/simple-avr-bootloader/assets/1549028/3f6d5e52-2eb7-48e1-b77b-d7f25770e6fb)

Here is the content of the output `.hex` file for the blinky program (see [Intel HEX File Format](https://microchipdeveloper.com/ipe:sqtp-hex-file-format)):

![image](https://github.com/m3y54m/simple-avr-bootloader/assets/1549028/43bb30ea-5cfd-4bab-9002-a3bfe7651469)

This build process is exactly what happens when you create firmware for bootloader updates - your application gets compiled into a binary file that the bootloader can program into flash memory.


## Implementing Self-Programming: The Hardcoded Bootloader

Now that we understand the build process, let's see how a bootloader can manipulate these binary files. Our hardcoded bootloader demonstrates **self-programming** - the ability for a microcontroller to modify its own flash memory while running.

### The Goal

This example shows how a bootloader can:
1. Check if an application exists in flash memory
2. If not, install a pre-compiled application (our 162-byte blinky)
3. Jump to the installed application

This is the foundation of all bootloader operations - the ability to write new code to flash and execute it.

### The Implementation

```c
#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/boot.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

// This array contains the exact binary we generated in the previous step
// It's the compiled blinky program stored as raw data
uint8_t hardcoded_blinky_bin[] = {
    0x0C, 0x94, 0x34, 0x00, 0x0C, 0x94, 0x3E, 0x00, 0x0C, 0x94, 0x3E, 0x00, 0x0C, 0x94, 0x3E, 0x00, 0x0C, 0x94,
    0x3E, 0x00, 0x0C, 0x94, 0x3E, 0x00, 0x0C, 0x94, 0x3E, 0x00, 0x0C, 0x94, 0x3E, 0x00, 0x0C, 0x94, 0x3E, 0x00,
    0x0C, 0x94, 0x3E, 0x00, 0x0C, 0x94, 0x3E, 0x00, 0x0C, 0x94, 0x3E, 0x00, 0x0C, 0x94, 0x3E, 0x00, 0x0C, 0x94,
    0x3E, 0x00, 0x0C, 0x94, 0x3E, 0x00, 0x0C, 0x94, 0x3E, 0x00, 0x0C, 0x94, 0x3E, 0x00, 0x0C, 0x94, 0x3E, 0x00,
    0x0C, 0x94, 0x3E, 0x00, 0x0C, 0x94, 0x3E, 0x00, 0x0C, 0x94, 0x3E, 0x00, 0x0C, 0x94, 0x3E, 0x00, 0x0C, 0x94,
    0x3E, 0x00, 0x0C, 0x94, 0x3E, 0x00, 0x0C, 0x94, 0x3E, 0x00, 0x0C, 0x94, 0x3E, 0x00, 0x11, 0x24, 0x1F, 0xBE,
    0xCF, 0xEF, 0xD8, 0xE0, 0xDE, 0xBF, 0xCD, 0xBF, 0x0E, 0x94, 0x40, 0x00, 0x0C, 0x94, 0x4F, 0x00, 0x0C, 0x94,
    0x00, 0x00, 0x25, 0x9A, 0x90, 0xE2, 0x85, 0xB1, 0x89, 0x27, 0x85, 0xB9, 0x2F, 0xEF, 0x31, 0xEE, 0x84, 0xE0,
    0x21, 0x50, 0x30, 0x40, 0x80, 0x40, 0xE1, 0xF7, 0x00, 0xC0, 0x00, 0x00, 0xF3, 0xCF, 0xF8, 0x94, 0xFF, 0xCF};
```

**What's happening**: We've embedded the complete blinky binary (from the previous section) as a data array in our bootloader. This is like having a firmware image built into the bootloader itself.

### The Self-Programming Function

The heart of any bootloader is the ability to write to flash memory:

```c
/**
 * @brief Programs binary data to flash memory using AVR self-programming
 * 
 * This demonstrates the core bootloader functionality - taking a binary
 * file and writing it to the microcontroller's flash memory.
 */
void write_program(const uint32_t address, const uint8_t *program_buffer, const uint32_t program_buffer_size)
{
  // Disable interrupts - flash programming is timing-critical
  uint8_t sreg_last_state = SREG;
  cli();

  eeprom_busy_wait(); // Ensure EEPROM operations are complete

  // Flash memory must be programmed page by page (128 bytes on ATmega328P)
  for (uint32_t current_page_address = address;
       current_page_address < (address + program_buffer_size);
       current_page_address += SPM_PAGESIZE)
  {
    // Step 1: Erase the page (set all bits to 1)
    boot_page_erase(current_page_address);
    boot_spm_busy_wait();

    // Step 2: Fill the temporary page buffer with new data
    for (uint16_t i = 0; i < SPM_PAGESIZE; i += 2)
    {
      uint16_t current_word = 0xFFFF; // Default to erased state
      
      if ((current_page_address + i) < (address + program_buffer_size))
      {
        // AVR flash is programmed in 16-bit words (little-endian)
        current_word = *program_buffer++;
        current_word |= (*program_buffer++) << 8;
      }

      // Load word into temporary page buffer
      boot_page_fill(current_page_address + i, current_word);
    }

    // Step 3: Write the page buffer to flash memory
    boot_page_write(current_page_address);
    boot_spm_busy_wait();
  }

  // Re-enable flash reading while programming (RWW section)
  boot_rww_enable();
  
  // Restore interrupt state
  SREG = sreg_last_state;
}
```

**Key concepts**:
- **Page-based programming**: Flash memory must be written in fixed-size pages (128 bytes for ATmega328P)
- **Erase before write**: Each page must be erased before new data can be written
- **Word-aligned data**: AVR processes flash data as 16-bit words, not individual bytes
- **Critical sections**: Interrupts must be disabled during flash operations

### The Bootloader Logic

```c
int main(void)
{
  DDRB |= (1 << PB5); // Configure LED for status indication

  // Check if user application exists by examining the reset vector
  // Empty flash reads as 0xFFFF, programmed flash will have actual code
  if (pgm_read_word(0) == 0xFFFF)
  {
    // Visual indication: bootloader is installing application
    // Slow blinks distinguish bootloader activity from application activity
    for (uint8_t i = 0; i < 2; i++)
    {
      PORTB &= ~(1 << PB5); // LED off
      _delay_ms(2000);
      PORTB |= 1 << PB5;    // LED on
      _delay_ms(100);
    }

    // Install the hardcoded application to address 0x0000
    // This is exactly what a real bootloader does with received firmware
    write_program(0x00000, hardcoded_blinky_bin, sizeof(hardcoded_blinky_bin));
  }

  // Transfer control to the user application
  // This is how bootloaders "launch" the main application
  __asm__ __volatile__("jmp 0");
}
```

**The process**:
1. **Check for existing application**: Read the reset vector at address 0x0000
2. **Install if needed**: If flash is empty (0xFFFF), program the hardcoded binary
3. **Jump to application**: Transfer control to address 0x0000 where the application starts

### Observing the Results

When you run this bootloader:

1. **First boot**: LED blinks slowly (2 times) → bootloader installs blinky → LED blinks fast continuously
2. **Subsequent boots**: LED immediately starts blinking fast (application already installed)

This demonstrates the complete bootloader cycle: detect missing firmware, install firmware, run firmware.


## Bootloader Memory Configuration

To make our bootloader work, we need to configure the microcontroller's memory layout and tell it to start from the bootloader section instead of the normal application area.

### Configuring Fuse Bits

**Fuse bits** are special configuration bytes that control how the microcontroller behaves. For bootloader operation, we need to set:

1. **BOOTRST = 0** (programmed): Start from bootloader section on reset instead of 0x0000
![image](https://github.com/m3y54m/simple-avr-bootloader/assets/1549028/42900b44-b6f0-4371-b181-afd68e7d34f4)

2. **BOOTSZ1:BOOTSZ0**: Configure boot section size
![image](https://github.com/m3y54m/simple-avr-bootloader/assets/1549028/974ef4eb-b016-4100-91b5-719db5d217f1)
![image](https://github.com/m3y54m/simple-avr-bootloader/assets/1549028/43a6f9f8-abd5-4ee9-9da1-82fe69b287c5)

Our bootloader is approximately 664 bytes, which requires a minimum boot section of 512 words (1024 bytes). To provide a safe margin, we have allocated 1024 words (2048 bytes) for it.

| BOOTSZ1 | BOOTSZ0 | Boot Section Size | Boot Start Address (in bytes) |
|---------|---------|-------------------|-------------------|
| 0       | 1       | 1024 words (2KB) | 0x7800 |

After setting the fuse bits (which determine the boot section size via the BOOTSZ bits), the resulting memory layout for the ATmega328P flash memory (addresses shown in bytes) will be as follows:

| Section | Address Range | Size | Purpose |
|---------|---------------|------|---------|
| **Application Section** | 0x0000 - 0x77FF | 30KB | Your main program |
| **Boot Section** | 0x7800 - 0x7FFF | 2KB | Bootloader code |

The boot section is **write-protected** by default, making it much safer from accidental corruption.

### Programming the Fuses

> [!WARNING]
> Incorrect fuse settings can brick your microcontroller! Double-check before programming.

**Check current fuse settings:**
```bash
avrdude -c usbasp -p m328p -U lfuse:r:-:h -U hfuse:r:-:h -U efuse:r:-:h
```

**Program bootloader-compatible fuses:**
```bash
avrdude -c usbasp -p m328p -U lfuse:w:0xFF:m -U hfuse:w:0xDA:m -U efuse:w:0xFD:m
```

**What these fuse values do**:
- `lfuse:0xFF` - Clock settings (external 16MHz crystal, no divide-by-8)
- `hfuse:0xDA` - **BOOTRST=0** (start from bootloader), **BOOTSZ=01** (2KB boot section)
- `efuse:0xFD` - Brown-out detection settings

You can verify these settings using the [AVR Fuse Calculator](https://www.engbedded.com/fusecalc/).

### Compiling for the Boot Section

The bootloader must be compiled to run from address 0x7800, not the default 0x0000:

```bash
# Compile bootloader source
avr-gcc -Wall -Os -mmcu=atmega328p -std=c11 -o build/main.o -c main.c

# Link with bootloader start address
avr-gcc -Wall -Os -mmcu=atmega328p -std=c11 -Wl,-section-start=.text=0x7800 -o bootloader.elf main.o
```

**What `-Wl,-section-start=.text=0x7800` does**:
- Tells the linker to place program code starting at address 0x7800
- This matches the boot section start address we configured with fuses
- Without this, the bootloader would be compiled for address 0x0000 and wouldn't work

## Frequently Asked Questions

### Q: Why can't the user application update itself?
**A:** Great question! In AVR microcontrollers like the ATmega328P, the user application is actually **not allowed** to perform self-programming - only code running from the bootloader section has this privilege. This is a hardware security feature that prevents accidental flash corruption. Beyond this technical restriction, having a separate bootloader is also safer: if something goes wrong during a self-update (power loss, corrupted data, bugs in update code), you could brick your device. The bootloader provides a safety net - it's a minimal, well-tested program that's less likely to fail, and it can always recover a corrupted application.

### Q: How does the bootloader know when to update vs. run the app?
**A:** There are several strategies:
- **External trigger**: Check a button press, pin state, or UART command on startup
- **Shared memory flag**: The application sets a magic value in RAM before resetting
- **Missing application**: If flash appears empty, enter programming mode
- **Timeout**: Wait briefly for update commands, then proceed to app

Our implementation uses the shared memory approach - the user app sets a flag in a special RAM location that survives software resets.

### Q: What if the bootloader itself gets corrupted?
**A:** The bootloader section is write-protected by default, making accidental corruption unlikely. However, if it does happen, you'd need an external programmer (ISP/ICSP) to recover - that's why bootloader code should be minimal and thoroughly tested.

### Q: Can I use this bootloader in production?
**A:** This is an educational implementation to teach concepts. For production use, you'd want to add features like:
- Encryption/authentication to prevent malicious firmware
- More robust error handling and recovery
- Backup/rollback mechanisms
- Watchdog timer integration
- Size optimization

### Q: Why UART instead of USB or Ethernet?
**A:** UART is simple, universally available, and requires minimal hardware. Most development boards have a USB-to-UART converter built-in. Once you understand UART bootloaders, the concepts easily extend to other communication methods.


## Resources

- [ATmega48A/PA/88A/PA/168A/PA/328/P Datasheet](https://ww1.microchip.com/downloads/en/DeviceDoc/ATmega48A-PA-88A-PA-168A-PA-328-P-DS-DS40002061B.pdf)
- [<avr/boot.h>: Bootloader Support Utilities](https://www.nongnu.org/avr-libc/user-manual/group__avr__boot.html)
- [AVR Libc - Memory Sections](https://www.nongnu.org/avr-libc/user-manual/mem_sections.html)
- [AVR109: Using Self Programming on tinyAVR and megaAVR devices](https://www.microchip.com/en-us/application-notes/an1644)
- [Basics to Developing Bootloader for Arduino](https://www.electronicwings.com/arduino/basics-to-developing-bootloader-for-arduino)
- [Optiboot Bootloader for Arduino and Atmel AVR](https://github.com/Optiboot/optiboot)
- [AVR Bootloader in C - eine einfache Anleitung](https://www.mikrocontroller.net/articles/AVR_Bootloader_in_C_-_eine_einfache_Anleitung)
- [How To Write a Simple Bootloader For AVR In C language- (Part 35/46)](https://www.engineersgarage.com/how-to-write-a-simple-bootloader-for-avr-in-c-language-part-35-46/)
- [AVR Fuse Calculator](https://www.engbedded.com/fusecalc/)
