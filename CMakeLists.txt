cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME stmtest)

set(EXECUTABLE_ELF ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.elf)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message(STATUS "Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)


# GCC_ARM_CM33_NONSECURE           - Compiler: GCC           Target: ARM Cortex-M33 non-secure
# GCC_ARM_CM33_SECURE              - Compiler: GCC           Target: ARM Cortex-M33 secure
# GCC_ARM_CM33_NTZ_NONSECURE       - Compiler: GCC           Target: ARM Cortex-M33 non-trustzone non-secure
# GCC_ARM_CM33_TFM                 - Compiler: GCC           Target: ARM Cortex-M33 non-secure for TF-M

set(FREERTOS_PORT  GCC_ARM_CM33_NTZ_NONSECURE)
set(FREERTOS_HEAP  4)
add_subdirectory(freertos_config)
add_subdirectory(freertos)
target_link_libraries(freertos_kernel INTERFACE freertos_config PUBLIC stm32cubemx)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

add_compile_options(-O0)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

add_subdirectory(Core/Src/bg96)
add_subdirectory(Core/Src/drv8825)
add_subdirectory(Core/Src/fifo)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
                      stm32cubemx
                      freertos_kernel
                      drv8825
                      bg96
                      fifo
                      )


configure_file(cmake/flasher_commands.json.in
               ${CMAKE_CURRENT_BINARY_DIR}/flasher_commands.json)
