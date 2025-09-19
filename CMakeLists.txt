cmake_minimum_required(VERSION 3.10)

# Use the AVR toolchain file (set before project())
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/toolchain.cmake CACHE STRING "AVR toolchain file")
project("AVR Bootloader" C)

# ---------------------------------------------------------------------------
# Common configuration
# ---------------------------------------------------------------------------

# MCU and clock settings
set(MCU atmega328p)

# This definition is essential for libraries like <util/delay.h>
set(F_CPU 16000000UL)

# Programmer for avrdude
set(AVRDUDE_PROGRAMMER usbasp)

# C standard settings (apply project-wide by default)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

# Common compiler flags for AVR targets
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Os -mmcu=${MCU} -DF_CPU=${F_CPU}")

# Add subprojects
add_subdirectory(blinky)
add_subdirectory(bootloader_hardcoded)

# Clean all build files and directories
add_custom_target(
  clean_build
  COMMAND ${CMAKE_COMMAND} -E remove_directory "${CMAKE_BINARY_DIR}"
  COMMENT "[[${PROJECT_NAME}]] Cleaning the entire build directory..."
)

# Fully erase the flash and EEPROM (if present) of the microcontroller
add_custom_target(
  erase_flash
  COMMAND avrdude -c ${AVRDUDE_PROGRAMMER} -p ${MCU} -e
  COMMENT "[[${PROJECT_NAME}]] Erasing the flash and EEPROM of \"${MCU}\" using \"${AVRDUDE_PROGRAMMER}\""
)