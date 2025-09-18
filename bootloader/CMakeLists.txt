cmake_minimum_required(VERSION 3.10)
project(bootloader C)

set(CMAKE_EXE_LINKER_FLAGS
  "${CMAKE_EXE_LINKER_FLAGS} -Wl,-section-start=.text=0x7C00")

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

# Fuse bits for ATmega328P
# Default values are set for a 16MHz external crystal with 512 words bootloader size
# CAUTION: Setting incorrect fuse bits can brick your microcontroller.
# https://www.engbedded.com/fusecalc/
set(LFUSE 0xFF CACHE STRING "Low fuse byte")
set(HFUSE 0xDC CACHE STRING "High fuse byte")
set(EFUSE 0xFD CACHE STRING "Extended fuse byte")

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
)
