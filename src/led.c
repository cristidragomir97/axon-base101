#include "led.h"
#include "pins.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"

// LED GPIO pin mapping (indexed by led_id_t)
static const uint8_t led_pins[LED_COUNT] = {
    [LED_FEETECH]  = PIN_LED_FEETECH,
    [LED_I2C]      = PIN_LED_I2C,
    [LED_UART0]    = PIN_LED_UART0,
    [LED_UART1]    = PIN_LED_UART1,
    [LED_CAN]      = PIN_LED_CAN,
    [LED_RS485]    = PIN_LED_RS485,
};

// Activity timeout counters (in ms)
#define LED_ACTIVITY_DURATION_MS 30
#define LED_BRIGHTNESS_PERCENT 25  // 25% brightness to reduce blinding effect

static uint8_t led_timeout[LED_COUNT];
static uint led_slice[LED_COUNT];    // PWM slice for each LED
static uint led_chan[LED_COUNT];     // PWM channel for each LED

void led_init(void) {
    for (int i = 0; i < LED_COUNT; i++) {
        // Set up PWM for brightness control
        gpio_set_function(led_pins[i], GPIO_FUNC_PWM);
        led_slice[i] = pwm_gpio_to_slice_num(led_pins[i]);
        led_chan[i] = pwm_gpio_to_channel(led_pins[i]);
        
        // Configure PWM: 1kHz frequency, 100 steps for brightness
        pwm_set_wrap(led_slice[i], 99);  // 0-99 range for easy percentage
        pwm_set_chan_level(led_slice[i], led_chan[i], 0);  // Start off
        pwm_set_clkdiv(led_slice[i], 125.0f);  // ~1kHz @ 125MHz
        pwm_set_enabled(led_slice[i], true);
        
        led_timeout[i] = 0;
    }
}

void led_activity(led_id_t led) {
    if (led >= LED_COUNT) return;
    pwm_set_chan_level(led_slice[led], led_chan[led], LED_BRIGHTNESS_PERCENT);
    led_timeout[led] = LED_ACTIVITY_DURATION_MS;
}

void led_set(led_id_t led, bool on) {
    if (led >= LED_COUNT) return;
    pwm_set_chan_level(led_slice[led], led_chan[led], on ? LED_BRIGHTNESS_PERCENT : 0);
    led_timeout[led] = 0;  // Cancel any pending auto-off
}

void led_task(void) {
    // Called every ~1ms from main loop
    for (int i = 0; i < LED_COUNT; i++) {
        if (led_timeout[i] > 0) {
            led_timeout[i]--;
            if (led_timeout[i] == 0) {
                pwm_set_chan_level(led_slice[i], led_chan[i], 0);
            }
        }
    }
}

void led_all(bool on) {
    for (int i = 0; i < LED_COUNT; i++) {
        led_set(i, on);
    }
}

void led_flash_all(uint32_t count, uint32_t on_ms, uint32_t off_ms) {
    for (uint32_t i = 0; i < count; i++) {
        led_all(true);
        sleep_ms(on_ms);
        led_all(false);
        if (i < count - 1) {  // Don't wait after last flash
            sleep_ms(off_ms);
        }
    }
}
