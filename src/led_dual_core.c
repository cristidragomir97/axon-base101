#include "led_dual_core.h"
#include "neopixel.h"
#include "pins.h"
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/time.h"

// LED timing constants
#define LED_ACTIVITY_DURATION_MS 50
#define LED_MAX_BRIGHTNESS 64  // Optimized for 3.3V operation

// Core 1 LED state
static bool led_states[LED_COUNT];
static uint32_t led_timeouts[LED_COUNT];
static absolute_time_t last_update_time;

// Predefined colors for each LED type
static const neopixel_color_t led_colors[LED_COUNT] = {
    [LED_RS485]    = {0, LED_MAX_BRIGHTNESS, 0},  // Green
    [LED_FEETECH]  = {0, LED_MAX_BRIGHTNESS, 0},  // Green
    [LED_UART0]    = {0, LED_MAX_BRIGHTNESS, 0},  // Green
    [LED_UART1]    = {0, LED_MAX_BRIGHTNESS, 0},  // Green
    [LED_I2C]      = {0, LED_MAX_BRIGHTNESS, 0},  // Green
    [LED_CAN]      = {0, LED_MAX_BRIGHTNESS, 0},  // Green
};

// Core 0 functions - send commands to Core 1 via FIFO
void led_init_dual_core(void) {
    // Core 0: Just prepare for multicore communication
    // Core 1 will initialize the NeoPixel hardware
}

void led_activity(led_id_t led) {
    if (led >= LED_COUNT) return;
    
    uint32_t msg = LED_MAKE_CMD(LED_CMD_ACTIVITY, led, LED_ACTIVITY_DURATION_MS);
    
    // Send to Core 1 via FIFO (non-blocking for Core 0)
    if (multicore_fifo_wready()) {
        multicore_fifo_push_timeout_us(msg, 0);
    }
}

void led_set(led_id_t led, bool on) {
    if (led >= LED_COUNT) return;
    
    uint32_t cmd = on ? LED_CMD_SET_ON : LED_CMD_SET_OFF;
    uint32_t msg = LED_MAKE_CMD(cmd, led, 0);
    
    if (multicore_fifo_wready()) {
        multicore_fifo_push_timeout_us(msg, 0);
    }
}

void led_all(bool on) {
    uint32_t cmd = on ? LED_CMD_ALL_ON : LED_CMD_ALL_OFF;
    uint32_t msg = LED_MAKE_CMD(cmd, 0, 0);
    
    if (multicore_fifo_wready()) {
        multicore_fifo_push_timeout_us(msg, 0);
    }
}

void led_flash_all(uint32_t count, uint32_t on_ms, uint32_t off_ms) {
    // For simplicity, use fixed timing for flash_all (can be enhanced later)
    uint32_t msg = LED_MAKE_CMD(LED_CMD_FLASH_ALL, 0, count);
    
    if (multicore_fifo_wready()) {
        multicore_fifo_push_timeout_us(msg, 0);
    }
}

// Core 1 LED controller main function
void led_core1_main(void) {
    // Initialize NeoPixel system on Core 1
    if (!neopixel_init(PIN_NEOPIXELS, LED_COUNT, 800000)) {
        // Blink SOS pattern if initialization fails
        while (true) {
            tight_loop_contents();
        }
    }
    
    // Clear all LEDs initially
    neopixel_clear();
    neopixel_show();
    
    // Initialize state
    for (int i = 0; i < LED_COUNT; i++) {
        led_states[i] = false;
        led_timeouts[i] = 0;
    }
    last_update_time = get_absolute_time();
    
    // Main LED control loop on Core 1
    while (true) {
        bool need_update = false;
        
        // Process commands from Core 0
        while (multicore_fifo_rvalid()) {
            uint32_t msg = multicore_fifo_pop_blocking();
            uint32_t cmd_type = LED_GET_CMD(msg);
            uint32_t led_id = LED_GET_LED(msg);
            uint32_t param = LED_GET_PARAM(msg);
            
            switch (cmd_type) {
                case LED_CMD_ACTIVITY:
                    if (led_id < LED_COUNT) {
                        led_states[led_id] = true;
                        led_timeouts[led_id] = param;  // Duration in ms
                        need_update = true;
                    }
                    break;
                    
                case LED_CMD_SET_ON:
                    if (led_id < LED_COUNT) {
                        led_states[led_id] = true;
                        led_timeouts[led_id] = 0;  // Cancel auto-off
                        need_update = true;
                    }
                    break;
                    
                case LED_CMD_SET_OFF:
                    if (led_id < LED_COUNT) {
                        led_states[led_id] = false;
                        led_timeouts[led_id] = 0;  // Cancel auto-off
                        need_update = true;
                    }
                    break;
                    
                case LED_CMD_ALL_ON:
                    for (int i = 0; i < LED_COUNT; i++) {
                        led_states[i] = true;
                        led_timeouts[i] = 0;
                    }
                    need_update = true;
                    break;
                    
                case LED_CMD_ALL_OFF:
                    for (int i = 0; i < LED_COUNT; i++) {
                        led_states[i] = false;
                        led_timeouts[i] = 0;
                    }
                    need_update = true;
                    break;
                    
                case LED_CMD_FLASH_ALL:
                    // Execute flash sequence (blocking on Core 1 is OK)
                    for (uint32_t i = 0; i < param; i++) {
                        // Turn all on
                        for (int j = 0; j < LED_COUNT; j++) {
                            neopixel_set_pixel_color(j, led_colors[j]);
                        }
                        neopixel_show();
                        sleep_ms(100);  // Fixed 100ms on
                        
                        // Turn all off
                        neopixel_clear();
                        neopixel_show();
                        if (i < param - 1) {  // Don't wait after last flash
                            sleep_ms(100);  // Fixed 100ms off
                        }
                    }
                    break;
            }
        }
        
        // Handle LED timeouts (activity auto-off)
        absolute_time_t current_time = get_absolute_time();
        uint32_t elapsed_ms = absolute_time_diff_us(last_update_time, current_time) / 1000;
        
        if (elapsed_ms >= 1) {  // Update every ~1ms
            for (int i = 0; i < LED_COUNT; i++) {
                if (led_timeouts[i] > 0) {
                    if (led_timeouts[i] <= elapsed_ms) {
                        led_timeouts[i] = 0;
                        led_states[i] = false;
                        need_update = true;
                    } else {
                        led_timeouts[i] -= elapsed_ms;
                    }
                }
            }
            last_update_time = current_time;
        }
        
        // Update NeoPixel strip if needed
        if (need_update) {
            for (int i = 0; i < LED_COUNT; i++) {
                if (led_states[i]) {
                    neopixel_set_pixel_color(i, led_colors[i]);
                } else {
                    neopixel_set_pixel_color(i, (neopixel_color_t){0, 0, 0});
                }
            }
            neopixel_show();
        }
        
        // Small delay to prevent Core 1 from spinning at 100%
        sleep_us(100);
    }
}