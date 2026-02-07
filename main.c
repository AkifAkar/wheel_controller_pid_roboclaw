#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "pt.h"
#include "roboclaw_bridge.h"

// Wiznet Libraries
#include "port_common.h"
#include "wizchip_conf.h"
#include "w5x00_spi.h"
#include "socket.h"

// --- Configuration ---
#define PLL_SYS_KHZ (133 * 1000)
#define SOCKET_UDP        0
#define PORT_UDP          5000 
#define ETHERNET_BUF_SIZE 2048

// IP: 10.42.0.32
static wiz_NetInfo g_net_info = {
    .mac = {0x00, 0x08, 0xDC, 0x12, 0x34, 0x32},
    .ip = {10, 42, 0, 32},
    .sn = {255, 255, 255, 0},
    .gw = {10, 42, 0, 254},
    .dns = {8, 8, 8, 8},
    .dhcp = NETINFO_STATIC
};

// Buffers
static uint8_t g_rx_buf[ETHERNET_BUF_SIZE] = {0,};
static uint8_t g_tx_buf[ETHERNET_BUF_SIZE] = {0,};

// Host Info (Who are we talking to?)
static uint8_t g_dest_ip[4] = {0, 0, 0, 0};
static uint16_t g_dest_port = 0;
static bool g_has_connection = false;

// --- RoboClaw Config ---
#define ROBOCLAW_UART_ID  uart1
#define ROBOCLAW_TX_PIN   8
#define ROBOCLAW_RX_PIN   9
#define ROBOCLAW_BAUD     2400   
#define ROBOCLAW_ADDR     0x80
#define ROBOCLAW_TIMEOUT  100000 

// --- Motor Constants ---
#define M1_ACCEL          2000   
#define M1_SPEED          550   
#define M1_DECCEL         2000
#define M1_BUFFER_FLAG    1

// --- Encoder Logic ---
#define ENC_MIN 250
#define ENC_MAX 1700
#define ANGLE_MIN 90
#define ANGLE_MAX 270

// --- Globals ---
volatile int32_t g_desired_encoder_val = ENC_MIN;
volatile int32_t g_current_encoder_val = 0; 
volatile uint16_t g_battery_voltage = 0;
Roboclaw_Handle* g_roboclaw = NULL;

// --- Threads ---
static struct pt pt_motor_send;
static struct pt pt_motor_read;
static struct pt pt_udp_rx;
static struct pt pt_udp_tx;

// --- Helper Functions ---
static void set_clock_khz(void) {
    set_sys_clock_khz(PLL_SYS_KHZ, true);
    clock_configure(clk_peri, 0, CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS, 
                    PLL_SYS_KHZ * 1000, PLL_SYS_KHZ * 1000);
}

void update_encoder_target(int angle_input) {
    if (angle_input < ANGLE_MIN) angle_input = ANGLE_MIN;
    if (angle_input > ANGLE_MAX) angle_input = ANGLE_MAX;
    int32_t possible_movement = ENC_MAX - ENC_MIN;
    int32_t angle_range = ANGLE_MAX - ANGLE_MIN;
    g_desired_encoder_val = ENC_MIN + ((angle_input - ANGLE_MIN) * possible_movement) / angle_range;
}

/**
 * ----------------------------------------------------------------------------------------------------
 * JSON PARSER (Adapted from your example)
 * Expects format: {"angle": 180} or {"angle": 180, "speed": 400}
 * ----------------------------------------------------------------------------------------------------
 */
void parse_and_process_json(char* json_str) {
    // printf("Parsing: %s\n", json_str); // Debug print

    // 1. Check for "angle" key
    char* key_angle = strstr(json_str, "\"angle\"");
    if (key_angle) {
        // Find the colon after "angle"
        char* val_str = strstr(key_angle, ":");
        if (val_str) {
            val_str++; // Skip colon
            int angle = atoi(val_str);
            if (angle > 0) {
                update_encoder_target(angle);
                // printf("JSON CMD: Set Angle %d\n", angle);
            }
        }
    }

    // 2. Check for "speed" key (Optional: if you want to update speed dynamically)
    // You can add logic here to update M1_SPEED global if needed
}

// ----------------------------------------------------------
// THREAD: Motor Send (Control Loop)
// ----------------------------------------------------------
static int thread_motor_send(struct pt *pt) {
    static uint32_t timestamp;
    static int32_t last_sent_target = -1; 

    PT_BEGIN(pt);
    while(1) {
        if (g_roboclaw && (g_desired_encoder_val != last_sent_target)) {
            printf("[MOTOR] Moving to %d\n", g_desired_encoder_val);
            roboclaw_speed_accel_deccel_pos_m1(
                g_roboclaw, ROBOCLAW_ADDR, 
                M1_ACCEL, M1_SPEED, M1_DECCEL, 
                g_desired_encoder_val, M1_BUFFER_FLAG
            );
            last_sent_target = g_desired_encoder_val;
        }
        timestamp = to_ms_since_boot(get_absolute_time());
        PT_WAIT_UNTIL(pt, to_ms_since_boot(get_absolute_time()) - timestamp >= 100);
    }
    PT_END(pt);
}

// ----------------------------------------------------------
// THREAD: Motor Read (Status Loop)
// ----------------------------------------------------------
static int thread_motor_read(struct pt *pt) {
    static uint32_t timestamp;
    static bool valid_v, valid_enc;
    static uint32_t raw_enc;
    static uint8_t status_m1;

    PT_BEGIN(pt);
    while(1) {
        if (g_roboclaw) {
            g_battery_voltage = roboclaw_read_main_battery(g_roboclaw, ROBOCLAW_ADDR, &valid_v);
            PT_YIELD(pt); 
            raw_enc = roboclaw_read_enc_m1(g_roboclaw, ROBOCLAW_ADDR, &status_m1, &valid_enc);
            if (valid_enc) g_current_encoder_val = (int32_t)raw_enc;
        }
        timestamp = to_ms_since_boot(get_absolute_time());
        PT_WAIT_UNTIL(pt, to_ms_since_boot(get_absolute_time()) - timestamp >= 1000);
    }
    PT_END(pt);
}

// ----------------------------------------------------------
// THREAD: UDP Receive & Process (Non-blocking)
// ----------------------------------------------------------
static int thread_udp_rx(struct pt *pt) {
    static uint32_t timestamp;
    static int32_t recv_len;
    // Temp vars for parsing header
    static uint8_t remote_ip[4];
    static uint16_t remote_port;

    PT_BEGIN(pt);
    while(1) {
        // 1. Check if data is available in the W5500 buffer
        if (getSn_RX_RSR(SOCKET_UDP) > 0) {
            
            // 2. Read the packet
            recv_len = recvfrom(SOCKET_UDP, g_rx_buf, ETHERNET_BUF_SIZE - 1, remote_ip, &remote_port);

            if (recv_len > 0) {
                // Null terminate string
                g_rx_buf[recv_len] = '\0';

                // 3. Store who sent it (so TX thread knows where to reply)
                memcpy(g_dest_ip, remote_ip, 4);
                g_dest_port = remote_port;
                g_has_connection = true;

                // 4. Parse the JSON
                parse_and_process_json((char*)g_rx_buf);
            }
        }
        
        // Run as fast as possible, but yield to let other threads run
        PT_YIELD(pt);
    }
    PT_END(pt);
}

// ----------------------------------------------------------
// THREAD: UDP Transmit (Telemetry)
// ----------------------------------------------------------
static int thread_udp_tx(struct pt *pt) {
    static uint32_t timestamp;
    static int len;

    PT_BEGIN(pt);
    while(1) {
        // Only send if we have heard from a computer at least once
        if (g_has_connection) {
            
            // 1. Format JSON Telemetry
            // Example: {"voltage": 24.5, "encoder": 1200, "target": 1200}
            len = snprintf((char*)g_tx_buf, ETHERNET_BUF_SIZE, 
                "{\"voltage\": %d.%d, \"encoder\": %d, \"target\": %d}", 
                g_battery_voltage / 10, g_battery_voltage % 10, 
                g_current_encoder_val, g_desired_encoder_val
            );

            // 2. Send via UDP
            sendto(SOCKET_UDP, g_tx_buf, len, g_dest_ip, g_dest_port);
            
            // printf("[UDP TX] Sent Telemetry\n");
        }

        // Send telemetry every 200ms (5Hz)
        timestamp = to_ms_since_boot(get_absolute_time());
        PT_WAIT_UNTIL(pt, to_ms_since_boot(get_absolute_time()) - timestamp >= 200);
    }
    PT_END(pt);
}

// ----------------------------------------------------------
// MAIN
// ----------------------------------------------------------
void setup() {
    set_clock_khz();
    stdio_init_all();
    sleep_ms(2000);
    printf("--- Pico Wheel Controller (JSON UDP) ---\n");

    // Init Network
    wizchip_spi_initialize();
    wizchip_cris_initialize();
    wizchip_reset();
    wizchip_initialize();
    wizchip_check();
    network_initialize(g_net_info);
    
    if(socket(SOCKET_UDP, Sn_MR_UDP, PORT_UDP, 0) != SOCKET_UDP) {
        printf("Socket Fail!\n");
        while(1);
    }
    printf("UDP Port %d Open. Waiting for JSON...\n", PORT_UDP);

    // Init Motor
    gpio_set_function(ROBOCLAW_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(ROBOCLAW_RX_PIN, GPIO_FUNC_UART);
    g_roboclaw = roboclaw_create(ROBOCLAW_UART_ID, ROBOCLAW_TX_PIN, ROBOCLAW_RX_PIN, ROBOCLAW_TIMEOUT);
    if (g_roboclaw) roboclaw_begin(g_roboclaw, ROBOCLAW_BAUD);

    // Init Threads
    PT_INIT(&pt_motor_send);
    PT_INIT(&pt_motor_read);
    PT_INIT(&pt_udp_rx);
    PT_INIT(&pt_udp_tx);
}

int main() {
    setup();
    while (true) {
        thread_motor_send(&pt_motor_send);
        thread_motor_read(&pt_motor_read);
        thread_udp_rx(&pt_udp_rx);
        thread_udp_tx(&pt_udp_tx);
    }
}