# ---------------------------------------------------------------------------
# Toolchain file for AVR-GCC
# ---------------------------------------------------------------------------
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR avr)

# Specify the cross-compiler
set(CMAKE_C_COMPILER avr-gcc)
set(CMAKE_CXX_COMPILER avr-g++)
set(CMAKE_ASM_COMPILER avr-gcc)

# Set other toolchain utilities
set(CMAKE_OBJCOPY avr-objcopy)
set(CMAKE_OBJDUMP avr-objdump)
set(CMAKE_SIZE avr-size)

# Set the CMAKE_FIND_ROOT_PATH to the root of the toolchain
# This helps CMake find the compiler and other tools.
# If your AVR toolchain is not in your system's PATH, you'll need to specify the path here.
# For example: set(CMAKE_FIND_ROOT_PATH "C:/Program Files (x86)/Atmel/Studio/7.0/toolchain/avr8/avr8-gnu-toolchain")
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)