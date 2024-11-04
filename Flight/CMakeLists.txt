cmake_minimum_required(VERSION 3.10)
project(pico_lora_mpu6050)

# Set the compiler and linker flags
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -g")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -g")

# Find the Pico SDK
find_package(PicoSDK REQUIRED)

# Add the Pico SDK include directories
include_directories(${PicoSDK_INCLUDE_DIRS})

# Add the source files
add_executable(pico_lora_mpu6050 main.c)

# Link against the Pico SDK libraries
target_link_libraries(pico_lora_mpu6050 ${PicoSDK_LIBRARIES})

# Enable USB communication
target_link_libraries(pico_lora_mpu6050 pico_stdio_usb)
