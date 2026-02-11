#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h> 
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

#ifndef LAST_IP_OCTET
    #define LAST_IP_OCTET 32
#endif

#ifndef MAC_LAST_BYTE
    #define MAC_LAST_BYTE 0x32
#endif

// IP: 10.42.0.32
static wiz_NetInfo g_net_info = {
    .mac = {0x00, 0x08, 0xDC, 0x12, 0x34, MAC_LAST_BYTE},
    .ip = {10, 42, 0, LAST_IP_OCTET},
    .sn = {255, 255, 255, 0},
    .gw = {10, 42, 0, 254},
    .dns = {8, 8, 8, 8},
    .dhcp = NETINFO_STATIC
};

// Buffers
static uint8_t g_rx_buf[ETHERNET_BUF_SIZE] = {0,};
static uint8_t g_tx_buf[ETHERNET_BUF_SIZE] = {0,};

// Host Info
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
#ifndef ENC_MIN
    #define ENC_MIN 250
#endif
#ifndef ENC_MAX
    #define ENC_MAX 1700
#endif
#define ANGLE_MIN 90
#define ANGLE_MAX 270

// --- If the logic is inverted ---
#ifndef INVERTED
    #define INVERTED 1
#endif

// --- Globals ---
volatile int32_t g_desired_encoder_val = 1050;    // Motor 1 Target
volatile int32_t g_desired_speed_val = 0;         // Motor 2 Speed (-127 to 127)
volatile int32_t g_current_encoder_val = 0; 
volatile uint16_t g_battery_voltage = 0;
Roboclaw_Handle* g_roboclaw = NULL;

// --- Threads ---
static struct pt pt_motor_send;
static struct pt pt_read_encoders;
static struct pt pt_read_battery;
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
    if (INVERTED) angle_input = 360 - angle_input;
    
    int32_t possible_movement = ENC_MAX - ENC_MIN;
    int32_t angle_range = ANGLE_MAX - ANGLE_MIN;
    g_desired_encoder_val = ENC_MIN + ((angle_input - ANGLE_MIN) * possible_movement) / angle_range;
    
}

void update_speed_target(int speed_input) {
    // Clamp to valid RoboClaw simple serial range (-127 to 127)
    if (speed_input > 127) speed_input = 127;
    if (speed_input < -127) speed_input = -127;
    g_desired_speed_val = speed_input;
}

// --- JSON Parser ---
// Format: {"command_type": "control", "action": "led", "value": 0, "speed": 100, "angle": 180, "light": 1}
void parse_and_process_json(char* json_str) {
    // 1. Parse ANGLE
    char* key_angle = strstr(json_str, "\"angle\"");
    if (key_angle) {
        char* val_str = strstr(key_angle, ":");
        if (val_str) {
            val_str++; 
            int angle = atoi(val_str);
            if (angle > 0) update_encoder_target(angle);
        }
    }

    // 2. Parse SPEED
    char* key_speed = strstr(json_str, "\"speed\"");
    if (key_speed) {
        char* val_str = strstr(key_speed, ":");
        if (val_str) {
            val_str++;
            int speed = atoi(val_str);
            update_speed_target(speed);
        }
    }
    
    // 3. Parse LIGHT (Optional, just demonstrating parsing)
    char* key_light = strstr(json_str, "\"light\"");
    if (key_light) {
        // Can add logic here later
    }
}

// ----------------------------------------------------------
// THREAD: Motor Send (Controls M1 and M2)
// ----------------------------------------------------------
static int thread_motor_send(struct pt *pt) {
    // Timers
    static uint32_t last_tx_time_m1 = 0;
    static uint32_t last_tx_time_m2 = 0;
    
    // State Trackers
    static int32_t last_sent_target_m1 = (ENC_MIN+ENC_MAX)/2; 
    static int32_t last_sent_speed_m2 = 0; 

    // Baud Rate Limiter (2400 baud = ~105ms per packet)
    // We prevent sending faster than this to avoid blocking the CPU
    const uint32_t MIN_TX_INTERVAL = 110; 

    PT_BEGIN(pt);

    while(1) {
        uint32_t now = to_ms_since_boot(get_absolute_time());

        if (g_roboclaw) {
            
            // --- Motor 1: Steering (Priority) ---
            // If Target Changed AND Cooldown Passed
            if ((g_desired_encoder_val != last_sent_target_m1) && 
                (now - last_tx_time_m1 >= MIN_TX_INTERVAL)) {
                
                printf("[M1] Steering: %d\n", g_desired_encoder_val);
                
                roboclaw_speed_accel_deccel_pos_m1(
                    g_roboclaw, ROBOCLAW_ADDR, 
                    M1_ACCEL, M1_SPEED, M1_DECCEL, 
                    g_desired_encoder_val, M1_BUFFER_FLAG
                );
                
                last_sent_target_m1 = g_desired_encoder_val;
                last_tx_time_m1 = now; // Reset Cooldown
                
                // Yield to let other threads run while UART hardware works
                PT_YIELD(pt); 
            }

            // --- Motor 2: Drive ---
            // Update time again in case M1 took time
            now = to_ms_since_boot(get_absolute_time());

            if ((g_desired_speed_val != last_sent_speed_m2) && 
                (now - last_tx_time_m2 >= MIN_TX_INTERVAL)) {
                
                // Prevent sending M2 immediately after M1 to avoid collision
                if (now - last_tx_time_m1 > 20) { 
                    printf("[M2] Speed: %d\n", g_desired_speed_val);
                    
                    uint8_t speed_mag = (uint8_t)abs(g_desired_speed_val);
                    if (g_desired_speed_val >= 0) {
                        roboclaw_forward_m2(g_roboclaw, ROBOCLAW_ADDR, speed_mag);
                    } else {
                        roboclaw_backward_m2(g_roboclaw, ROBOCLAW_ADDR, speed_mag);
                    }
                    
                    last_sent_speed_m2 = g_desired_speed_val;
                    last_tx_time_m2 = now;
                }
            }
        }

        // CRITICAL CHANGE: 
        // Instead of waiting 100ms unconditionally, we just YIELD.
        // This allows the thread to check for new UDP packets immediately.
        PT_YIELD(pt);
    }
    PT_END(pt);
}

// ----------------------------------------------------------
// THREAD: Read Encoders (Fast - 50ms)
// ----------------------------------------------------------
static int thread_read_encoders(struct pt *pt) {
    static uint32_t timestamp;
    static bool valid_enc;
    static uint32_t raw_enc;
    static uint8_t status_m1;

    PT_BEGIN(pt);
    while(1) {
        if (g_roboclaw) {
            raw_enc = roboclaw_read_enc_m1(g_roboclaw, ROBOCLAW_ADDR, &status_m1, &valid_enc);
            if (valid_enc) g_current_encoder_val = (int32_t)raw_enc;
        }
        timestamp = to_ms_since_boot(get_absolute_time());
        // Read frequently for responsive feedback
        PT_WAIT_UNTIL(pt, to_ms_since_boot(get_absolute_time()) - timestamp >= 50);
    }
    PT_END(pt);
}

// ----------------------------------------------------------
// THREAD: Read Battery (Slow - 60s)
// ----------------------------------------------------------
static int thread_read_battery(struct pt *pt) {
    static uint32_t timestamp;
    static bool valid_v;

    PT_BEGIN(pt);
    while(1) {
        if (g_roboclaw) {
            g_battery_voltage = roboclaw_read_main_battery(g_roboclaw, ROBOCLAW_ADDR, &valid_v);
        }
        timestamp = to_ms_since_boot(get_absolute_time());
        // Read rarely to save bus bandwidth
        PT_WAIT_UNTIL(pt, to_ms_since_boot(get_absolute_time()) - timestamp >= 60000);
    }
    PT_END(pt);
}

// ----------------------------------------------------------
// THREAD: UDP Receive (Burst Mode / Low Latency)
// ----------------------------------------------------------
static int thread_udp_rx(struct pt *pt) {
    static int32_t recv_len;
    static uint8_t remote_ip[4];
    static uint16_t remote_port;
    
    // Safety: Don't get stuck in this loop forever if flooded
    static int burst_count; 

    PT_BEGIN(pt);
    while(1) {
        // Reset burst counter
        burst_count = 0;

        // BURST LOOP: Keep reading as long as data is waiting!
        // We process up to 10 packets in a row before yielding.
        // This ensures we always process the LATEST command if multiple arrive.
        while (getSn_RX_RSR(SOCKET_UDP) > 0 && burst_count < 10) {
            
            recv_len = recvfrom(SOCKET_UDP, g_rx_buf, ETHERNET_BUF_SIZE - 1, remote_ip, &remote_port);

            if (recv_len > 0) {
                g_rx_buf[recv_len] = '\0';
                
                // Update connection info
                memcpy(g_dest_ip, remote_ip, 4);
                g_dest_port = remote_port;
                g_has_connection = true;

                // Parse immediately
                parse_and_process_json((char*)g_rx_buf);
            }
            
            burst_count++;
        }
        
        // Only yield after we have cleared the buffer or hit the limit
        PT_YIELD(pt);
    }
    PT_END(pt);
}

// ----------------------------------------------------------
// THREAD: UDP Transmit
// ----------------------------------------------------------
static int thread_udp_tx(struct pt *pt) {
    static uint32_t timestamp;
    static int len;

    PT_BEGIN(pt);
    while(1) {
        if (g_has_connection) {
            // Echo back status + current speed target
            len = snprintf((char*)g_tx_buf, ETHERNET_BUF_SIZE, 
                "{\"voltage\": %d.%d, \"encoder\": %d, \"target_angle\": %d, \"target_speed\": %d}", 
                g_battery_voltage / 10, g_battery_voltage % 10, 
                g_current_encoder_val, g_desired_encoder_val, g_desired_speed_val
            );
            sendto(SOCKET_UDP, g_tx_buf, len, g_dest_ip, g_dest_port);
        }
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
    printf("--- Pico Rover Control (M1 Steering + M2 Drive) ---\n");

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

    // Init Motor
    gpio_set_function(ROBOCLAW_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(ROBOCLAW_RX_PIN, GPIO_FUNC_UART);
    g_roboclaw = roboclaw_create(ROBOCLAW_UART_ID, ROBOCLAW_TX_PIN, ROBOCLAW_RX_PIN, ROBOCLAW_TIMEOUT);
    if (g_roboclaw) roboclaw_begin(g_roboclaw, ROBOCLAW_BAUD);

    PT_INIT(&pt_motor_send);
    PT_INIT(&pt_read_encoders);
    PT_INIT(&pt_read_battery);
    PT_INIT(&pt_udp_rx);
    PT_INIT(&pt_udp_tx);
}

int main() {
    setup();
    while (true) {
        thread_motor_send(&pt_motor_send);
        thread_read_encoders(&pt_read_encoders);
        thread_read_battery(&pt_read_battery);
        thread_udp_rx(&pt_udp_rx);
        thread_udp_tx(&pt_udp_tx);
    }
}