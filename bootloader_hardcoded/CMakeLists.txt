
cmake_minimum_required(VERSION 3.10)

# Dynamically set project name from directory name
get_filename_component(PROJECT_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PROJECT_NAME} C)

# Set the start address for the bootloader (Boot Reset Vector) for ATmega328P
# Fuse bits must be set accordingly to enable the bootloader section
# BOOTSZ0 = 0, BOOTSZ1 = 1 for 1024 words (0x3C00 * 2 = 0x7800 for ATmega328P)
set(BOOTLOADER_START_ADDRESS 0x7800)

# Fuse bits for ATmega328P
# Default values are set for a 16MHz external crystal with 1024 words bootloader size
# CAUTION: Setting incorrect fuse bits can brick your microcontroller.
# https://www.engbedded.com/fusecalc/
set(LFUSE 0xFF)
set(HFUSE 0xDA)
set(EFUSE 0xFD)

set(CMAKE_EXE_LINKER_FLAGS
  "${CMAKE_EXE_LINKER_FLAGS} -Wl,-section-start=.text=${BOOTLOADER_START_ADDRESS}")

add_executable(${PROJECT_NAME}.elf src/main.c)

# Build the hex and bin files
# which are needed for flashing the bootloader
add_custom_target(
  ${PROJECT_NAME}_build ALL
  COMMAND avr-objcopy -j .text -j .data -O ihex ${PROJECT_NAME}.elf ${PROJECT_NAME}.hex
  COMMAND avr-objcopy -j .text -j .data -O binary ${PROJECT_NAME}.elf ${PROJECT_NAME}.bin
  COMMAND avr-size --format=avr --mcu=${MCU} ${PROJECT_NAME}.elf
  DEPENDS ${PROJECT_NAME}.elf
  COMMENT "[[${PROJECT_NAME}]] Building .hex and .bin files for \"${MCU}\""
)

# Write fuse bits
add_custom_target(
  ${PROJECT_NAME}_write_fusebits
  COMMAND avrdude -c ${AVRDUDE_PROGRAMMER} -p ${MCU} -U lfuse:w:${LFUSE}:m -U hfuse:w:${HFUSE}:m -U efuse:w:${EFUSE}:m
  COMMENT "[[${PROJECT_NAME}]] Writing fuse bits to \"${MCU}\" using \"${AVRDUDE_PROGRAMMER}\""
  COMMENT "[[${PROJECT_NAME}]] Fuse bits set to: L=${LFUSE}, H=${HFUSE}, E=${EFUSE}"
)

# Flash the bootloader to the microcontroller
add_custom_target(
  ${PROJECT_NAME}_flash
  COMMAND avrdude -c ${AVRDUDE_PROGRAMMER} -p ${MCU} -U flash:w:${PROJECT_NAME}.hex
  DEPENDS ${PROJECT_NAME}_build
  COMMENT "[[${PROJECT_NAME}]] Flashing the bootloader to \"${MCU}\" using \"${AVRDUDE_PROGRAMMER}\""
  COMMENT "[[${PROJECT_NAME}]] Bootloader start address: 0x${BOOTLOADER_START_ADDRESS}"
)
