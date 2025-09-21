
cmake_minimum_required(VERSION 3.13)

# Dynamically set project name from directory name
get_filename_component(PROJECT_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PROJECT_NAME} C)

set(BOOT_APP_DIR bootloader)
set(BOOT_APP_NAME ${PROJECT_NAME}_${BOOT_APP_DIR})

set(USER_APP_DIR user-app)
set(USER_APP_NAME ${PROJECT_NAME}_${USER_APP_DIR})

set(SUP_PROTOCOL_DIR ${CMAKE_SOURCE_DIR}/first-steps/3-simple-uart-protocol/mcu)
set(BOOT_SYNC_DIR ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(${BOOT_APP_NAME}.elf ${BOOT_APP_DIR}/main.c ${SUP_PROTOCOL_DIR}/sup.c)
add_executable(${USER_APP_NAME}.elf ${USER_APP_DIR}/main.c ${SUP_PROTOCOL_DIR}/sup.c)

# Make the SUP protocol headers available to the targets so that
# source files can use #include "sup.h" and "boot_sync.h" without a relative path.
target_include_directories(${BOOT_APP_NAME}.elf PRIVATE ${SUP_PROTOCOL_DIR} ${BOOT_SYNC_DIR})
target_include_directories(${USER_APP_NAME}.elf PRIVATE ${SUP_PROTOCOL_DIR} ${BOOT_SYNC_DIR})

# Use target_link_options to apply linker flags ONLY to the bootloader.
# This is the correct, modern CMake way to handle this.
target_link_options(${BOOT_APP_NAME}.elf PRIVATE
  "-Wl,-section-start=.text=${BOOTLOADER_START_ADDRESS}"
)

set(BOOT_FLAG_MEMORY_ADDR 0x08F8)

# Place a small dedicated section ".bootflag" at a fixed SRAM address so that
# both bootloader and user application can reliably share a RAM-resident flag
# across software resets. Address chosen near top of SRAM for ATmega328P;
# adjust if you use a different MCU.
target_link_options(${BOOT_APP_NAME}.elf PRIVATE
  "-Wl,--section-start=.bootflag=${BOOT_FLAG_MEMORY_ADDR}"
)

target_link_options(${USER_APP_NAME}.elf PRIVATE
  "-Wl,--section-start=.bootflag=${BOOT_FLAG_MEMORY_ADDR}"
)

# Generate a linker map file next to each ELF. Use the output file name derived from
# the target name so the map is easy to find (e.g. final_bootloader.elf -> final_bootloader.map)
target_link_options(${BOOT_APP_NAME}.elf PRIVATE
  "-Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${BOOT_APP_NAME}.map"
)

target_link_options(${USER_APP_NAME}.elf PRIVATE
  "-Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${USER_APP_NAME}.map"
)

# Build the hex and bin files
# which are needed for flashing the bootloader
add_custom_target(
  ${BOOT_APP_NAME}_build ALL
  COMMAND avr-objcopy -j .text -j .data -O ihex ${BOOT_APP_NAME}.elf ${BOOT_APP_NAME}.hex
  COMMAND avr-objcopy -j .text -j .data -O binary ${BOOT_APP_NAME}.elf ${BOOT_APP_NAME}.bin
  COMMAND avr-size --format=avr --mcu=${MCU} ${BOOT_APP_NAME}.elf
  DEPENDS ${BOOT_APP_NAME}.elf
  COMMENT "[[${PROJECT_NAME}]] Building bootloader .hex and .bin files for \"${MCU}\""
)

add_custom_target(
  ${USER_APP_NAME}_build ALL
  COMMAND avr-objcopy -j .text -j .data -O ihex ${USER_APP_NAME}.elf ${USER_APP_NAME}.hex
  COMMAND avr-objcopy -j .text -j .data -O binary ${USER_APP_NAME}.elf ${USER_APP_NAME}.bin
  COMMAND avr-size --format=avr --mcu=${MCU} ${USER_APP_NAME}.elf
  DEPENDS ${USER_APP_NAME}.elf
  COMMENT "[[${PROJECT_NAME}]] Building user application .hex and .bin files for \"${MCU}\""
)

add_custom_target(
  ${PROJECT_NAME}_build_all ALL
  DEPENDS ${BOOT_APP_NAME}_build ${USER_APP_NAME}_build
  COMMENT "[[${PROJECT_NAME}]] Building all .hex and .bin files for \"${MCU}\""
)

# Flash the bootloader to the microcontroller
add_custom_target(
  ${BOOT_APP_NAME}_flash
  COMMAND avrdude -c ${AVRDUDE_PROGRAMMER} -p ${MCU} -U flash:w:${BOOT_APP_NAME}.hex:i
  DEPENDS ${BOOT_APP_NAME}_build
  COMMENT "[[${PROJECT_NAME}]] Flashing the bootloader to \"${MCU}\" using \"${AVRDUDE_PROGRAMMER}\""
  COMMENT "[[${PROJECT_NAME}]] Bootloader start address: 0x${BOOTLOADER_START_ADDRESS}"
)

# Flash the user application to the microcontroller
add_custom_target(
  ${USER_APP_NAME}_flash
  COMMAND avrdude -c ${AVRDUDE_PROGRAMMER} -p ${MCU} -U flash:w:${USER_APP_NAME}.hex:i
  DEPENDS ${USER_APP_NAME}_build
  COMMENT "[[${PROJECT_NAME}]] Flashing user application to \"${MCU}\" using \"${AVRDUDE_PROGRAMMER}\""
)

# Flash both the bootloader and user application to the microcontroller
add_custom_target(
  ${PROJECT_NAME}_flash_all
  COMMAND avrdude -c ${AVRDUDE_PROGRAMMER} -p ${MCU} -U flash:w:${USER_APP_NAME}.hex:i -U flash:w:${BOOT_APP_NAME}.hex:i
  DEPENDS ${PROJECT_NAME}_build_all
  COMMENT "[[${PROJECT_NAME}]] Flashing both bootloader and user application to \"${MCU}\" using \"${AVRDUDE_PROGRAMMER}\""
)
