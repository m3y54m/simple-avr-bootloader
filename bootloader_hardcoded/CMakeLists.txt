
cmake_minimum_required(VERSION 3.13)

# Dynamically set project name from directory name
get_filename_component(PROJECT_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PROJECT_NAME} C)

# Linker flags to set the bootloader start address
set(CMAKE_EXE_LINKER_FLAGS
  "${CMAKE_EXE_LINKER_FLAGS} -Wl,-section-start=.text=${BOOTLOADER_START_ADDRESS}")

add_executable(${PROJECT_NAME}.elf main.c)

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

# Flash the bootloader to the microcontroller
add_custom_target(
  ${PROJECT_NAME}_flash
  COMMAND avrdude -c ${AVRDUDE_PROGRAMMER} -p ${MCU} -U flash:w:${PROJECT_NAME}.hex
  DEPENDS ${PROJECT_NAME}_build
  COMMENT "[[${PROJECT_NAME}]] Flashing the bootloader to \"${MCU}\" using \"${AVRDUDE_PROGRAMMER}\""
  COMMENT "[[${PROJECT_NAME}]] Bootloader start address: 0x${BOOTLOADER_START_ADDRESS}"
)
