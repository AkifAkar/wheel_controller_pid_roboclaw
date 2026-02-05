#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/uart.h"

/* --- Library Includes --- */
// WIZnet dependencies (Ensure these are in your CMake path)
#include "port_common.h"
#include "wizchip_conf.h"
#include "w5x00_spi.h"
#include "socket.h"

// RoboClaw dependencies
#include "roboclaw_bridge.h"

/**
 * ----------------------------------------------------------------------------------------------------
 * Configuration & Definitions
 * ----------------------------------------------------------------------------------------------------
 */

/* System Clock */
#define PLL_SYS_KHZ (133 * 1000)

/* Network Configuration */
#define SOCKET_UDP        0
#define PORT_UDP          5000 
#define ETHERNET_BUF_SIZE 2048

// Static IP Configuration (10.42.0.32)
#define MY_IP_OCTET_1 10
#define MY_IP_OCTET_2 42
#define MY_IP_OCTET_3 0
#define MY_IP_OCTET_4 32

/* RoboClaw Configuration */
#define ROBOCLAW_UART_ID  uart1
#define ROBOCLAW_TX_PIN   8
#define ROBOCLAW_RX_PIN   9
#define ROBOCLAW_BAUD     38400 // Standard RoboClaw baud (adjust if your generic setup differs)
#define ROBOCLAW_ADDR     0x80
#define ROBOCLAW_TIMEOUT  10000

/* Global Objects */
static wiz_NetInfo g_net_info = {
    .mac = {0x00, 0x08, 0xDC, 0x12, 0x34, 0x32},
    .ip = {MY_IP_OCTET_1, MY_IP_OCTET_2, MY_IP_OCTET_3, MY_IP_OCTET_4},
    .sn = {255, 255, 255, 0},
    .gw = {10, 42, 0, 254},
    .dns = {8, 8, 8, 8},
    .dhcp = NETINFO_STATIC
};

static uint8_t g_udp_buf[ETHERNET_BUF_SIZE] = {0,};
Roboclaw_Handle* g_roboclaw = NULL;

/**
 * ----------------------------------------------------------------------------------------------------
 * Helper Functions
 * ----------------------------------------------------------------------------------------------------
 */

// Initialize the 133MHz System Clock
static void set_clock_khz(void) {
    set_sys_clock_khz(PLL_SYS_KHZ, true);
    clock_configure(clk_peri, 0, CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS, 
                    PLL_SYS_KHZ * 1000, PLL_SYS_KHZ * 1000);
}

// Master Hardware Initialization
void system_init(void) {
    /* 1. Basic System & Debug IO */
    set_clock_khz();
    stdio_init_all();
    sleep_ms(2000); // Wait for serial to settle
    printf("--- System Booting ---\n");

    /* 2. Initialize Wiznet W5500 (SPI0) */
    // Note: SPI pins (RX, TX, SCK, CS) are defined in your port_common.h 
    // or CMakeLists.txt. Ensure they match your hardware (e.g., GP16/17/18/19 or similar).
    printf("Initializing W5500 Ethernet...\n");
    wizchip_spi_initialize();
    wizchip_cris_initialize();
    
    wizchip_reset();
    wizchip_initialize();
    wizchip_check();
    
    network_initialize(g_net_info);
    print_network_information(g_net_info);

    /* 3. Initialize RoboClaw (UART1 on GP8/GP9) */
    printf("Initializing RoboClaw on UART1...\n");
    // We create the handle here to be used later by the threads
    g_roboclaw = roboclaw_create(ROBOCLAW_UART_ID, ROBOCLAW_TX_PIN, ROBOCLAW_RX_PIN, ROBOCLAW_TIMEOUT);
    if (g_roboclaw) {
        roboclaw_begin(g_roboclaw, ROBOCLAW_BAUD);
        printf("RoboClaw Initialized.\n");
    } else {
        printf("FAILED to create RoboClaw handle.\n");
    }
}

/**
 * ----------------------------------------------------------------------------------------------------
 * Main
 * ----------------------------------------------------------------------------------------------------
 */
int main() {
    int32_t ret = 0;
    uint8_t remote_ip[4] = {0};
    uint16_t remote_port = 0;

    // Run setup
    system_init();

    // Open UDP Socket
    if((ret = socket(SOCKET_UDP, Sn_MR_UDP, PORT_UDP, 0)) != SOCKET_UDP) {
        printf("Socket creation failed. Error: %ld\n", ret);
        while(1) tight_loop_contents();
    } else {
        printf("UDP Socket %d opened on Port %d.\n", SOCKET_UDP, PORT_UDP);
    }

    printf("Ready to receive JSON data...\n");

    // Main Loop (Test Only: Receive & Print)
    while (1) {
        // Check for incoming data size
        uint16_t size = getSn_RX_RSR(SOCKET_UDP);

        if (size > 0) {
            // Read data into buffer
            int32_t recv_len = recvfrom(SOCKET_UDP, g_udp_buf, ETHERNET_BUF_SIZE - 1, remote_ip, &remote_port);

            if (recv_len > 0) {
                // Null-terminate string for printing
                g_udp_buf[recv_len] = '\0';
                
                printf("\n[UDP RX] From %d.%d.%d.%d:%d\n", 
                       remote_ip[0], remote_ip[1], remote_ip[2], remote_ip[3], remote_port);
                printf("Payload: %s\n", g_udp_buf);
            }
        }
        
        // Minimal delay to prevent locking the core completely
        // In the next step, protothreads will manage this timing.
        sleep_ms(1); 
    }
}