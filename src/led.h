#ifndef LED_H
#define LED_H

#include <stdint.h>
#include <stdbool.h>

// Activity LED identifiers. Names mirror the logical buses used across the
// firmware (one classic GPIO LED each, pins in pins.h):
//   LED_FEETECH - servo bus activity     LED_I2C   - IMU / I2C
//   LED_UART0   - DDSM wheel motors       LED_UART1 - lidar
//   LED_CAN     - zenoh link status       LED_RS485 - spare
typedef enum {
    LED_FEETECH = 0,
    LED_I2C,
    LED_UART0,
    LED_UART1,
    LED_CAN,
    LED_RS485,
    LED_COUNT
} led_id_t;

// Initialize all activity LEDs
void led_init(void);

// Pulse an LED to indicate activity (non-blocking)
// LED will turn on and auto-off after ~50ms
void led_activity(led_id_t led);

// Set LED state directly
void led_set(led_id_t led, bool on);

// Must be called periodically from main loop (~1ms interval)
void led_task(void);

// Turn all LEDs on or off
void led_all(bool on);

// Flash all LEDs for success indication
void led_flash_all(uint32_t count, uint32_t on_ms, uint32_t off_ms);

#endif // LED_H
