
cmake_minimum_required(VERSION 3.13)

# Dynamically set project name from directory name
get_filename_component(PROJECT_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PROJECT_NAME} C)

add_executable(${PROJECT_NAME}_bootloader.elf bootloader/main.c ../simple_uart_protocol/sup.c)
add_executable(${PROJECT_NAME}_user_app.elf user_app/main.c ../simple_uart_protocol/sup.c)

# Use target_link_options to apply linker flags ONLY to the bootloader.
# This is the correct, modern CMake way to handle this.
target_link_options(${PROJECT_NAME}_bootloader.elf PRIVATE
  "-Wl,-section-start=.text=${BOOTLOADER_START_ADDRESS}"
)

# Place a small dedicated section ".bootflag" at a fixed SRAM address so that
# both bootloader and user application can reliably share a RAM-resident flag
# across software resets without using EEPROM. Address chosen near top of SRAM
# for ATmega328P; adjust if you use a different MCU.
target_link_options(${PROJECT_NAME}_bootloader.elf PRIVATE
  "-Wl,--section-start=.bootflag=0x08F8"
)

target_link_options(${PROJECT_NAME}_user_app.elf PRIVATE
  "-Wl,--section-start=.bootflag=0x08F8"
)

# Generate a linker map file next to each ELF. Use the output file name derived from
# the target name so the map is easy to find (e.g. final_bootloader.elf -> final_bootloader.map)
target_link_options(${PROJECT_NAME}_bootloader.elf PRIVATE
  "-Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_bootloader.map"
)

target_link_options(${PROJECT_NAME}_user_app.elf PRIVATE
  "-Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_user_app.map"
)

# Build the hex and bin files
# which are needed for flashing the bootloader
add_custom_target(
  ${PROJECT_NAME}_build_bootloader ALL
  COMMAND avr-objcopy -j .text -j .data -O ihex ${PROJECT_NAME}_bootloader.elf ${PROJECT_NAME}_bootloader.hex
  COMMAND avr-objcopy -j .text -j .data -O binary ${PROJECT_NAME}_bootloader.elf ${PROJECT_NAME}_bootloader.bin
  COMMAND avr-size --format=avr --mcu=${MCU} ${PROJECT_NAME}_bootloader.elf
  DEPENDS ${PROJECT_NAME}_bootloader.elf
  COMMENT "[[${PROJECT_NAME}]] Building bootloader .hex and .bin files for \"${MCU}\""
)

add_custom_target(
  ${PROJECT_NAME}_build_user_app ALL
  COMMAND avr-objcopy -j .text -j .data -O ihex ${PROJECT_NAME}_user_app.elf ${PROJECT_NAME}_user_app.hex
  COMMAND avr-objcopy -j .text -j .data -O binary ${PROJECT_NAME}_user_app.elf ${PROJECT_NAME}_user_app.bin
  COMMAND avr-size --format=avr --mcu=${MCU} ${PROJECT_NAME}_user_app.elf
  DEPENDS ${PROJECT_NAME}_user_app.elf
  COMMENT "[[${PROJECT_NAME}]] Building user application .hex and .bin files for \"${MCU}\""
)

add_custom_target(
  ${PROJECT_NAME}_build_all ALL
  DEPENDS ${PROJECT_NAME}_build_bootloader ${PROJECT_NAME}_build_user_app
  COMMENT "[[${PROJECT_NAME}]] Building all .hex and .bin files for \"${MCU}\""
)

# Flash the bootloader to the microcontroller
add_custom_target(
  ${PROJECT_NAME}_flash_bootloader
  COMMAND avrdude -c ${AVRDUDE_PROGRAMMER} -p ${MCU} -U flash:w:${PROJECT_NAME}_bootloader.hex:i
  DEPENDS ${PROJECT_NAME}_build_bootloader
  COMMENT "[[${PROJECT_NAME}]] Flashing the bootloader to \"${MCU}\" using \"${AVRDUDE_PROGRAMMER}\""
  COMMENT "[[${PROJECT_NAME}]] Bootloader start address: 0x${BOOTLOADER_START_ADDRESS}"
)

# Flash the user application to the microcontroller
add_custom_target(
  ${PROJECT_NAME}_flash_user_app
  COMMAND avrdude -c ${AVRDUDE_PROGRAMMER} -p ${MCU} -U flash:w:${PROJECT_NAME}_user_app.hex:i
  DEPENDS ${PROJECT_NAME}_build_user_app
  COMMENT "[[${PROJECT_NAME}]] Flashing user application to \"${MCU}\" using \"${AVRDUDE_PROGRAMMER}\""
)

# Flash both the bootloader and user application to the microcontroller
add_custom_target(
  ${PROJECT_NAME}_flash_all
  COMMAND avrdude -c ${AVRDUDE_PROGRAMMER} -p ${MCU} -U flash:w:${PROJECT_NAME}_user_app.hex:i -U flash:w:${PROJECT_NAME}_bootloader.hex:i
  DEPENDS ${PROJECT_NAME}_build_all
  COMMENT "[[${PROJECT_NAME}]] Flashing both bootloader and user application to \"${MCU}\" using \"${AVRDUDE_PROGRAMMER}\""
)
