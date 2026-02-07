cmake_minimum_required(VERSION 3.13)

# Initialize Pico SDK
include(pico_sdk_import.cmake)

project(pico_wheel_control C CXX ASM)

# Initialize the SDK
pico_sdk_init()

# --- Define the Executable ---
add_executable(pico_wheel_control
    main.c
    
    # RoboClaw Bridge (C++)
    lib/Basicmicro/Basicmicro.cpp
    lib/Basicmicro/roboclaw_bridge.cpp
)

# --- Include Directories ---
target_include_directories(pico_wheel_control PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    
    # RoboClaw Includes
    lib/Basicmicro
    
    # Protothreads Includes (Header only)
    lib/pt
)

# --- Link Libraries ---
target_link_libraries(pico_wheel_control PRIVATE
    pico_stdlib
    hardware_uart
)

# --- Standard IO Configuration ---
# Enable USB output for debug printf (Crucial for typing angles)
pico_enable_stdio_usb(pico_wheel_control 1)
# Disable UART stdio (UART1 is used for RoboClaw)
pico_enable_stdio_uart(pico_wheel_control 0)

# --- Output Generation ---
pico_add_extra_outputs(pico_wheel_control)