#ifndef LED_DUAL_CORE_H
#define LED_DUAL_CORE_H

#include <stdint.h>
#include <stdbool.h>

// Activity LED identifiers (6 NeoPixels indexed 0-5)
typedef enum {
    LED_I2C = 0,
    LED_UART0,
    LED_UART1,
    LED_FEETECH,
    LED_CAN,
    LED_RS485,
    LED_COUNT = 6   // Only 6 physical NeoPixels
} led_id_t;

// Initialize dual-core LED system
// Must be called from Core 0 main() before starting Core 1
void led_init_dual_core(void);

// Start LED controller on Core 1
// Call with multicore_launch_core1(led_core1_main);
void led_core1_main(void);

// Pulse an LED to indicate activity (non-blocking, Core 0 safe)
// LED will turn on and auto-off after ~50ms
void led_activity(led_id_t led);

// Set LED state directly (non-blocking, Core 0 safe)
void led_set(led_id_t led, bool on);

// Turn all LEDs on or off (non-blocking, Core 0 safe)
void led_all(bool on);

// Flash all LEDs for success indication (non-blocking, Core 0 safe)
void led_flash_all(uint32_t count, uint32_t on_ms, uint32_t off_ms);

// Inter-core message encoding (32-bit FIFO messages)
// Bits 31-28: Command type (4 bits)
// Bits 27-24: LED ID (4 bits)  
// Bits 23-0:  Parameters (24 bits)
#define LED_CMD_ACTIVITY    (0x0 << 28)
#define LED_CMD_SET_ON      (0x1 << 28)
#define LED_CMD_SET_OFF     (0x2 << 28)
#define LED_CMD_ALL_ON      (0x3 << 28)
#define LED_CMD_ALL_OFF     (0x4 << 28)
#define LED_CMD_FLASH_ALL   (0x5 << 28)

#define LED_MAKE_CMD(cmd, led, param) ((cmd) | ((led) << 24) | ((param) & 0xFFFFFF))
#define LED_GET_CMD(msg)              ((msg) & 0xF0000000)
#define LED_GET_LED(msg)              (((msg) >> 24) & 0xF)
#define LED_GET_PARAM(msg)            ((msg) & 0xFFFFFF)

#endif // LED_DUAL_CORE_H