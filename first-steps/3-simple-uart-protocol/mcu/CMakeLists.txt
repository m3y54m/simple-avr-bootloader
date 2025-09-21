cmake_minimum_required(VERSION 3.13)

project(3-simple-uart-protocol C)

add_executable(${PROJECT_NAME}.elf main.c sup.c)

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
