cmake_minimum_required(VERSION 3.13)

# Initialize Pico SDK
include(pico_sdk_import.cmake)

project(pico_wheel_control C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the SDK
pico_sdk_init()

# --------------------------------------------------------------------
# PATH DEFINITIONS
# --------------------------------------------------------------------
set(WIZNET_DIR "${CMAKE_CURRENT_LIST_DIR}/lib/ioLibrary_Driver")
set(PORT_DIR   "${CMAKE_CURRENT_LIST_DIR}/lib/port") 
set(ROBOCLAW_DIR "${CMAKE_CURRENT_LIST_DIR}/lib/Basicmicro")
set(PT_DIR     "${CMAKE_CURRENT_LIST_DIR}/lib/pt")

# --------------------------------------------------------------------
# EXECUTABLE & SOURCES
# --------------------------------------------------------------------
add_executable(pico_wheel_control
    main.c
    
<<<<<<< HEAD
    # Wiznet Core
    ${WIZNET_DIR}/Ethernet/wizchip_conf.c
    ${WIZNET_DIR}/Ethernet/socket.c
    ${WIZNET_DIR}/Ethernet/W5500/w5500.c
    
    # Wiznet Port (SPI/Timer)
    ${PORT_DIR}/ioLibrary_Driver/src/w5x00_spi.c
    ${PORT_DIR}/timer/timer.c
    
=======
>>>>>>> e183a6a256fc127aa7b341b48428abda968313af
    # RoboClaw Bridge (C++)
    ${ROBOCLAW_DIR}/Basicmicro.cpp
    ${ROBOCLAW_DIR}/roboclaw_bridge.cpp
)

# --------------------------------------------------------------------
# INCLUDE DIRECTORIES
# --------------------------------------------------------------------
target_include_directories(pico_wheel_control PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    
<<<<<<< HEAD
    # Wiznet
    ${WIZNET_DIR}/Ethernet
    ${WIZNET_DIR}/Ethernet/W5500
    ${PORT_DIR}/ioLibrary_Driver/inc
    ${PORT_DIR}/timer
    
    # RoboClaw
    ${ROBOCLAW_DIR}

    ${PORT_DIR}
    
    # Protothreads
    ${PT_DIR}
=======
    # RoboClaw Includes
    lib/Basicmicro
    
    # Protothreads Includes (Header only)
    lib/pt
>>>>>>> e183a6a256fc127aa7b341b48428abda968313af
)

# --------------------------------------------------------------------
# LIBRARIES
# --------------------------------------------------------------------
target_link_libraries(pico_wheel_control PRIVATE
    pico_stdlib
<<<<<<< HEAD
    hardware_spi
    hardware_dma
    hardware_uart
    hardware_gpio
    hardware_timer
)

# --------------------------------------------------------------------
# OUTPUTS
# --------------------------------------------------------------------
pico_enable_stdio_usb(pico_wheel_control 1)
pico_enable_stdio_uart(pico_wheel_control 0)
=======
    hardware_uart
)

# --- Standard IO Configuration ---
# Enable USB output for debug printf (Crucial for typing angles)
pico_enable_stdio_usb(pico_wheel_control 1)
# Disable UART stdio (UART1 is used for RoboClaw)
pico_enable_stdio_uart(pico_wheel_control 0)

# --- Output Generation ---
>>>>>>> e183a6a256fc127aa7b341b48428abda968313af
pico_add_extra_outputs(pico_wheel_control)