#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
<<<<<<< HEAD
#include "hardware/spi.h"
#include "hardware/gpio.h"

// --- Libraries ---
#include "pt.h"
#include "roboclaw_bridge.h"

// Wiznet Libraries
#include "port_common.h"
#include "wizchip_conf.h"
#include "w5x00_spi.h"
#include "socket.h"

// --- System Clock ---
#define PLL_SYS_KHZ (133 * 1000)

// --- Network Configuration ---
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

static uint8_t g_udp_buf[ETHERNET_BUF_SIZE] = {0,};

// --- RoboClaw Configuration ---
#define ROBOCLAW_UART_ID  uart1
#define ROBOCLAW_TX_PIN   8
#define ROBOCLAW_RX_PIN   9
#define ROBOCLAW_BAUD     2400   
#define ROBOCLAW_ADDR     0x80
#define ROBOCLAW_TIMEOUT  100000 

// --- Motor Constants (Updated) ---
#define M1_ACCEL          0     // Instant/Default
#define M1_SPEED          550   // Max Speed
#define M1_DECCEL         0     // Instant/Default
#define M1_BUFFER_FLAG    0     // Buffered

// --- Encoder Logic ---
#define ENC_MIN 250
=======
#include "hardware/gpio.h"
#include "pt.h"
#include "roboclaw_bridge.h"

// --- Configuration ---
#define ROBOCLAW_UART_ID  uart1
#define ROBOCLAW_TX_PIN   8
#define ROBOCLAW_RX_PIN   9
#define ROBOCLAW_BAUD     2400   
#define ROBOCLAW_ADDR     0x80
#define ROBOCLAW_TIMEOUT  100000 

// --- Motor Constants ---
#define M1_ACCEL          0   // Lowered for smoother ramp
#define M1_SPEED          550   
#define M1_DECCEL         0   // Lowered for smoother stop
#define M1_BUFFER_FLAG    0     // 0 = Buffer (Smoother transitions)

// --- Encoder Logic ---
#define ENC_MIN 278
>>>>>>> e183a6a256fc127aa7b341b48428abda968313af
#define ENC_MAX 1700
#define ANGLE_MIN 90
#define ANGLE_MAX 270

// --- Globals ---
volatile int32_t g_desired_encoder_val = ENC_MIN;
volatile int32_t g_current_encoder_val = 0; 
Roboclaw_Handle* g_roboclaw = NULL;

// --- Threads ---
static struct pt pt_send;
static struct pt pt_read;

<<<<<<< HEAD
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

// ----------------------------------------------------------
// PROTOTHREAD 1: Send Target
// ----------------------------------------------------------
static int thread_roboclaw_send(struct pt *pt) {
    static uint32_t timestamp;
    static int32_t last_sent_target = -1; 

    PT_BEGIN(pt);

    while(1) {
        // Send only on change
        if (g_roboclaw && (g_desired_encoder_val != last_sent_target)) {
            
            printf("[SEND] Target: %d\n", g_desired_encoder_val);

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
// PROTOTHREAD 2: Read Status
// ----------------------------------------------------------
static int thread_roboclaw_read(struct pt *pt) {
    static uint32_t timestamp;
    static uint16_t voltage;
    static bool valid_v;
    static uint32_t raw_enc;
    static uint8_t status_m1;
    static bool valid_enc;

    PT_BEGIN(pt);

    while(1) {
        if (g_roboclaw) {
            voltage = roboclaw_read_main_battery(g_roboclaw, ROBOCLAW_ADDR, &valid_v);
            PT_YIELD(pt); 
            raw_enc = roboclaw_read_enc_m1(g_roboclaw, ROBOCLAW_ADDR, &status_m1, &valid_enc);

            if (valid_enc) g_current_encoder_val = (int32_t)raw_enc;
            
            // Minimal print to keep UART clean
            printf("Stat: %d V, Pos: %d\n", voltage/10, g_current_encoder_val);
        }

        timestamp = to_ms_since_boot(get_absolute_time());
        PT_WAIT_UNTIL(pt, to_ms_since_boot(get_absolute_time()) - timestamp >= 1000);
    }

    PT_END(pt);
}

// ----------------------------------------------------------
// MAIN
// ----------------------------------------------------------
void setup() {
    set_clock_khz(); // Set 133MHz for Wiznet stability
    stdio_init_all();
    sleep_ms(2000);
    printf("--- System Boot: UDP Input Mode ---\n");

    // 1. Initialize Wiznet (SPI0)
    printf("Init Wiznet...\n");
    wizchip_spi_initialize();
    wizchip_cris_initialize();
    wizchip_reset();
    wizchip_initialize();
    wizchip_check();
    network_initialize(g_net_info);
    print_network_information(g_net_info);

    // 2. Open UDP Socket
    if(socket(SOCKET_UDP, Sn_MR_UDP, PORT_UDP, 0) != SOCKET_UDP) {
        printf("FATAL: UDP Socket Init Failed!\n");
        while(1);
    }
    printf("UDP Listening on Port %d\n", PORT_UDP);

    // 3. Initialize RoboClaw (UART1)
    gpio_set_function(ROBOCLAW_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(ROBOCLAW_RX_PIN, GPIO_FUNC_UART);

    g_roboclaw = roboclaw_create(ROBOCLAW_UART_ID, ROBOCLAW_TX_PIN, ROBOCLAW_RX_PIN, ROBOCLAW_TIMEOUT);
    if (g_roboclaw) {
        roboclaw_begin(g_roboclaw, ROBOCLAW_BAUD);
        printf("RoboClaw Initialized.\n");
=======
// --- Helper ---
void update_encoder_target(int angle_input) {
    if (angle_input < ANGLE_MIN) angle_input = ANGLE_MIN;
    if (angle_input > ANGLE_MAX) angle_input = ANGLE_MAX;
    int32_t possible_movement = ENC_MAX - ENC_MIN;
    int32_t angle_range = ANGLE_MAX - ANGLE_MIN;
    g_desired_encoder_val = ENC_MIN + ((angle_input - ANGLE_MIN) * possible_movement) / angle_range;
}

// ----------------------------------------------------------
// PROTOTHREAD 1: Send Target (SMART VERSION)
// ----------------------------------------------------------
static int thread_roboclaw_send(struct pt *pt) {
    static uint32_t timestamp;
    static int32_t last_sent_target = -1; // Force send on first run

    PT_BEGIN(pt);

    while(1) {
        // ONLY send if the target has changed!
        // This prevents "stuttering" from re-sending the same command repeatedly.
        if (g_roboclaw && (g_desired_encoder_val != last_sent_target)) {
            
            roboclaw_speed_accel_deccel_pos_m1(
                g_roboclaw, ROBOCLAW_ADDR, 
                M1_ACCEL, M1_SPEED, M1_DECCEL, 
                g_desired_encoder_val, M1_BUFFER_FLAG
            );
            
            // Update our tracker so we don't send again until user asks
            last_sent_target = g_desired_encoder_val;
            printf("Command Sent: Go to %d\n", last_sent_target);
        }

        // Wait 100ms (Fast checks are fine now because we rarely transmit)
        timestamp = to_ms_since_boot(get_absolute_time());
        PT_WAIT_UNTIL(pt, to_ms_since_boot(get_absolute_time()) - timestamp >= 100);
    }

    PT_END(pt);
}

// ----------------------------------------------------------
// PROTOTHREAD 2: Read Status (1Hz)
// ----------------------------------------------------------
static int thread_roboclaw_read(struct pt *pt) {
    static uint32_t timestamp;
    static uint16_t voltage;
    static bool valid_v;
    static uint32_t raw_enc;
    static uint8_t status_m1;
    static bool valid_enc;

    PT_BEGIN(pt);

    while(1) {
        if (g_roboclaw) {
            voltage = roboclaw_read_main_battery(g_roboclaw, ROBOCLAW_ADDR, &valid_v);
            PT_YIELD(pt); // Yield to keep things fluid
            raw_enc = roboclaw_read_enc_m1(g_roboclaw, ROBOCLAW_ADDR, &status_m1, &valid_enc);

            if (valid_enc) g_current_encoder_val = (int32_t)raw_enc;

            // Optional: Comment this out if it spams your console too much
            printf("\n[Status] V: %d.%d | Pos: %d\n", voltage/10, voltage%10, g_current_encoder_val);
        }

        timestamp = to_ms_since_boot(get_absolute_time());
        PT_WAIT_UNTIL(pt, to_ms_since_boot(get_absolute_time()) - timestamp >= 1000);
    }

    PT_END(pt);
}

// ----------------------------------------------------------
// MAIN
// ----------------------------------------------------------
void setup() {
    stdio_init_all();
    sleep_ms(2000);
    printf("--- Smart Motion Mode ---\n");

    gpio_set_function(ROBOCLAW_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(ROBOCLAW_RX_PIN, GPIO_FUNC_UART);

    g_roboclaw = roboclaw_create(ROBOCLAW_UART_ID, ROBOCLAW_TX_PIN, ROBOCLAW_RX_PIN, ROBOCLAW_TIMEOUT);
    if (g_roboclaw) {
        roboclaw_begin(g_roboclaw, ROBOCLAW_BAUD);
>>>>>>> e183a6a256fc127aa7b341b48428abda968313af
    }

    PT_INIT(&pt_send);
    PT_INIT(&pt_read);
<<<<<<< HEAD
=======
    
    printf("Enter Angle (90-270): \n");
>>>>>>> e183a6a256fc127aa7b341b48428abda968313af
}

int main() {
    setup();
<<<<<<< HEAD

    uint8_t remote_ip[4] = {0};
    uint16_t remote_port = 0;
    int32_t recv_len;

    while (true) {
        // --- 1. Run Threads ---
        thread_roboclaw_send(&pt_send);
        thread_roboclaw_read(&pt_read);

        // --- 2. Poll UDP Socket ---
        // Check if data exists in Wiznet buffer
        if (getSn_RX_RSR(SOCKET_UDP) > 0) {
            
            recv_len = recvfrom(SOCKET_UDP, g_udp_buf, ETHERNET_BUF_SIZE - 1, remote_ip, &remote_port);

            if (recv_len > 0) {
                g_udp_buf[recv_len] = '\0'; // Null-terminate string
                
                // Parse Angle (Assuming payload is just "180" or "90")
                int angle = atoi((char*)g_udp_buf);
                
                if (angle > 0) {
                    update_encoder_target(angle);
                    printf("UDP RX: %d deg -> Target %d\n", angle, g_desired_encoder_val);
                } else {
                    printf("UDP RX (Invalid): %s\n", g_udp_buf);
                }
=======

    while (true) {
        thread_roboclaw_send(&pt_send);
        thread_roboclaw_read(&pt_read);

        // Input Handling
        int input_char = getchar_timeout_us(0); 
        static char buffer[10];
        static int buf_idx = 0;
        
        if (input_char != PICO_ERROR_TIMEOUT) {
            if (input_char == '\n' || input_char == '\r') {
                buffer[buf_idx] = 0; 
                int angle = atoi(buffer);
                if (angle > 0) { 
                    update_encoder_target(angle);
                    // printf handled in the thread now
                }
                buf_idx = 0; 
            } else if (buf_idx < 9 && input_char >= '0' && input_char <= '9') {
                buffer[buf_idx++] = (char)input_char;
>>>>>>> e183a6a256fc127aa7b341b48428abda968313af
            }
        }
    }
}