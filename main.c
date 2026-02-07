#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
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
    }

    PT_INIT(&pt_send);
    PT_INIT(&pt_read);
    
    printf("Enter Angle (90-270): \n");
}

int main() {
    setup();

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
            }
        }
    }
}