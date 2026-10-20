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
set(CMAKE_PROJECT_NAME balance_car)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    Core/Src/app_bat.c
    Core/Src/app_usart2.c
    Core/Src/app_button.c
    Core/Src/app_button.c
    Core/Src/app_pwm.c
    Core/Src/app_encoder.c
    Core/Src/app_mpu6050.c
    Core/Src/qmath.c
    Core/Src/pid.c
    Core/Src/app_motor.c
    Core/Src/app_control.c
    Core/Src/app_rc.c
    Core/Src/app_rgb.c
    Test/test_voltage.c
    Test/test_encoder.c
    Test/test_pwm.c
    Test/test_mpu6050.c
  
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/Test
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
# Empty uses APP_TEST_MODE in main.c; overrides support building each test mode.
set(BALANCE_CAR_TEST_MODE "" CACHE STRING "Test override: VOLTAGE, ENCODER, PWM, or empty")
set_property(CACHE BALANCE_CAR_TEST_MODE PROPERTY STRINGS "" VOLTAGE ENCODER PWM)
if(BALANCE_CAR_TEST_MODE STREQUAL "VOLTAGE")
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE APP_TEST_MODE=1)
elseif(BALANCE_CAR_TEST_MODE STREQUAL "ENCODER")
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE APP_TEST_MODE=2)
elseif(BALANCE_CAR_TEST_MODE STREQUAL "PWM")
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE APP_TEST_MODE=3)
elseif(NOT BALANCE_CAR_TEST_MODE STREQUAL "")
    message(FATAL_ERROR "BALANCE_CAR_TEST_MODE must be VOLTAGE, ENCODER, PWM, or empty")
endif()

list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
# Encoder position uses float degrees; enable floating point printf in newlib-nano.
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE -u _printf_float)

target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
