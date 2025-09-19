cmake_minimum_required(VERSION 3.10)

# Dynamically set project name from directory name
get_filename_component(PROJECT_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PROJECT_NAME} C)

add_executable(${PROJECT_NAME}.elf src/main.c)

add_custom_target(
  ${PROJECT_NAME}_build ALL
  COMMAND avr-objcopy -j .text -j .data -O ihex ${PROJECT_NAME}.elf ${PROJECT_NAME}.hex
  COMMAND avr-objcopy -j .text -j .data -O binary ${PROJECT_NAME}.elf ${PROJECT_NAME}.bin
  COMMAND avr-size --format=avr --mcu=${MCU} ${PROJECT_NAME}.elf
  DEPENDS ${PROJECT_NAME}.elf
  COMMENT "[[${PROJECT_NAME}]] Building .hex and .bin files for \"${MCU}\""
)

add_custom_target(
  ${PROJECT_NAME}_flash
  COMMAND avrdude -c ${AVRDUDE_PROGRAMMER} -p ${MCU} -U flash:w:${PROJECT_NAME}.hex
  DEPENDS ${PROJECT_NAME}_build
  COMMENT "[[${PROJECT_NAME}]] Flashing to \"${MCU}\" using \"${AVRDUDE_PROGRAMMER}\""
)
