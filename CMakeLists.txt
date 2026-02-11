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
# WHEEL CONFIGURATIONS
# --------------------------------------------------------------------
set(WHEEL_CONFIGS
    "FL|30|0x30|200|1700|1" 
    "FR|32|0x32|266|1725|1"
    "RL|34|0x34|225|1684|1"
    "RR|36|0x36|200|1645|1"
)

# --------------------------------------------------------------------
# BUILD LOOP
# --------------------------------------------------------------------
foreach(CONFIG_STR IN LISTS WHEEL_CONFIGS)
    # 1. Convert the pipe '|' back to a standard list list ';'
    string(REPLACE "|" ";" CONFIG_LIST ${CONFIG_STR})

    # 2. Parse configuration list
    list(GET CONFIG_LIST 0 WHEEL_NAME)
    list(GET CONFIG_LIST 1 VAL_IP)
    list(GET CONFIG_LIST 2 VAL_MAC)
    list(GET CONFIG_LIST 3 VAL_ENC_MIN)
    list(GET CONFIG_LIST 4 VAL_ENC_MAX)
    list(GET CONFIG_LIST 5 VAL_INVERTED)

    # 3. Define a unique target name
    set(TARGET_NAME "wheel_${WHEEL_NAME}")

    message(STATUS "Configuring ${TARGET_NAME}: IP=...${VAL_IP}, MAC=...${VAL_MAC}, Inverted=${VAL_INVERTED}")

    # 4. Create Executable
    add_executable(${TARGET_NAME}
        main.c
        ${WIZNET_DIR}/Ethernet/wizchip_conf.c
        ${WIZNET_DIR}/Ethernet/socket.c
        ${WIZNET_DIR}/Ethernet/W5500/w5500.c
        ${PORT_DIR}/ioLibrary_Driver/src/w5x00_spi.c
        ${PORT_DIR}/timer/timer.c
        ${ROBOCLAW_DIR}/Basicmicro.cpp
        ${ROBOCLAW_DIR}/roboclaw_bridge.cpp
    )

    # 5. Inject Definitions
    target_compile_definitions(${TARGET_NAME} PRIVATE
        LAST_IP_OCTET=${VAL_IP}
        MAC_LAST_BYTE=${VAL_MAC}
        ENC_MIN=${VAL_ENC_MIN}
        ENC_MAX=${VAL_ENC_MAX}
        INVERTED=${VAL_INVERTED}
    )

    # 6. Include Directories
    target_include_directories(${TARGET_NAME} PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${WIZNET_DIR}/Ethernet
        ${WIZNET_DIR}/Ethernet/W5500
        ${PORT_DIR}/ioLibrary_Driver/inc
        ${PORT_DIR}/timer
        ${ROBOCLAW_DIR}
        ${PORT_DIR}
        ${PT_DIR}
    )

    # 7. Link Libraries
    target_link_libraries(${TARGET_NAME} PRIVATE
        pico_stdlib
        hardware_spi
        hardware_dma
        hardware_uart
        hardware_gpio
        hardware_timer
    )

    # 8. Generate Outputs
    pico_enable_stdio_usb(${TARGET_NAME} 1)
    pico_enable_stdio_uart(${TARGET_NAME} 0)
    pico_add_extra_outputs(${TARGET_NAME})

endforeach()