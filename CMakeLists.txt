cmake_minimum_required(VERSION 3.13)

# Initialize Pico SDK
include(pico_sdk_import.cmake)

project(pico_wheel_control C CXX ASM)

# Initialize the SDK
pico_sdk_init()

# --- Define the Executable ---
add_executable(pico_wheel_control
    main.c
    
    # Wiznet Core Libraries
    lib/ioLibrary_Driver/Ethernet/W5500/w5500.c
    lib/ioLibrary_Driver/Ethernet/wizchip_conf.c
    lib/ioLibrary_Driver/Ethernet/socket.c
    
    # Wiznet Port Layer (SPI & Timer)
    lib/port/ioLibrary_Driver/src/w5x00_spi.c
    lib/port/timer/timer.c
    
    # RoboClaw Bridge (C++)
    lib/Basicmicro/Basicmicro.cpp
    lib/Basicmicro/roboclaw_bridge.cpp
)

# --- Include Directories ---
target_include_directories(pico_wheel_control PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    
    # Wiznet Includes
    lib/ioLibrary_Driver/Ethernet
    lib/ioLibrary_Driver/Ethernet/W5500
    
    # Port Includes
    lib/port
    lib/port/ioLibrary_Driver/inc
    lib/port/timer
    
    # RoboClaw Includes
    lib/Basicmicro
    
    # Protothreads Includes
    lib/pt
)

# --- Link Libraries ---
target_link_libraries(pico_wheel_control PRIVATE
    pico_stdlib
    hardware_spi
    hardware_uart
    hardware_dma
    hardware_timer
)

# --- Standard IO Configuration ---
# Enable USB output for debug printf
pico_enable_stdio_usb(pico_wheel_control 1)
# Disable UART stdio (since UART is used for RoboClaw)
pico_enable_stdio_uart(pico_wheel_control 0)

# --- Output Generation ---
# Create .uf2, .hex, .bin, etc.
pico_add_extra_outputs(pico_wheel_control)