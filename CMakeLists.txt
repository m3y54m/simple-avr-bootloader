cmake_minimum_required(VERSION 3.13)

# Use the AVR toolchain file (set before project())
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/toolchain-avr.cmake CACHE STRING "AVR toolchain file")

# Dynamically set project name from directory name
get_filename_component(PROJECT_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PROJECT_NAME} C)

# ---------------------------------------------------------------------------
# Common configuration
# ---------------------------------------------------------------------------

# MCU and clock settings
set(MCU atmega328p)

# This definition is essential for libraries like <util/delay.h>
set(F_CPU 16000000UL)

# UART Baud rate settings
set(BAUD 57600)

# Programmer for avrdude
set(AVRDUDE_PROGRAMMER usbasp)

# C standard settings (apply project-wide by default)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

# Set the start address for the bootloader (Boot Reset Vector) for ATmega328P
# Fuse bits must be set accordingly to enable the boot reset vector (BOOTRST = 0)
# and set bootloader section size.
# BOOTSZ0 = 0, BOOTSZ1 = 1 for 1024 words (0x3C00 * 2 = 0x7800 for ATmega328P)
set(BOOTLOADER_START_ADDRESS 0x7800)

# Fuse bits for ATmega328P
# Default values are set for a 16MHz external crystal with 1024 words bootloader size
# CAUTION: Setting incorrect fuse bits can brick your microcontroller.
# https://www.engbedded.com/fusecalc/
set(LFUSE 0xFF)
set(HFUSE 0xDA)
set(EFUSE 0xFD)

# Common compiler flags for AVR targets
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Os -mmcu=${MCU} -DF_CPU=${F_CPU} -DBAUD=${BAUD} -DMAX_USER_APP_SIZE=${BOOTLOADER_START_ADDRESS}")

# Add sub-projects
add_subdirectory(first-steps/1-blinky)
add_subdirectory(first-steps/2-hardcoded-bootloader)
add_subdirectory(first-steps/3-simple-uart-protocol/mcu)
add_subdirectory(firmware)

# Clean all build files and directories
add_custom_target(
  all_clean_build
  COMMAND ${CMAKE_COMMAND} -E remove_directory "${CMAKE_BINARY_DIR}"
  COMMENT "[[${PROJECT_NAME}]] Cleaning the entire build directory..."
)

# Fully erase the flash and EEPROM (if present) of the microcontroller
add_custom_target(
  all_erase_flash
  COMMAND avrdude -c ${AVRDUDE_PROGRAMMER} -p ${MCU} -e
  COMMENT "[[${PROJECT_NAME}]] Erasing the flash and EEPROM of \"${MCU}\" using \"${AVRDUDE_PROGRAMMER}\""
)

# Write fuse bits
add_custom_target(
  all_write_fusebits
  COMMAND avrdude -c ${AVRDUDE_PROGRAMMER} -p ${MCU} -U lfuse:w:${LFUSE}:m -U hfuse:w:${HFUSE}:m -U efuse:w:${EFUSE}:m
  COMMENT "[[${PROJECT_NAME}]] Writing fuse bits to \"${MCU}\" using \"${AVRDUDE_PROGRAMMER}\""
  COMMENT "[[${PROJECT_NAME}]] Fuse bits set to: L=${LFUSE}, H=${HFUSE}, E=${EFUSE}"
)
