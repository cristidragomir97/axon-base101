#include "half_duplex.h"
#include "pins.h"
#include "led.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "tusb.h"

#include "uart_tx.pio.h"
#include "uart_rx.pio.h"

// Unified servo bus over TWO physical half-duplex channels.
//
// The Axon 2 board has two servo connectors (Feetech/STS and "Dynamixel"),
// both wired as half-duplex UARTs with a TX-enable line and both running the
// Feetech STS protocol. They are presented to the rest of the firmware as a
// single bus: every command is broadcast on both channels and replies from
// either channel are merged. Because servo IDs are unique across the two
// connectors, only the channel a given servo lives on ever answers.
//
// Both channels live on PIO0 (SM0/1 and SM2/3), sharing one copy of the
// uart_tx/uart_rx programs.

#define HD_PIO pio0

typedef struct {
    uint8_t sm_tx;
    uint8_t sm_rx;
    uint8_t tx_pin;
    uint8_t rx_pin;
    uint8_t txen_pin;
} hd_channel_t;

static const hd_channel_t channels[] = {
    { .sm_tx = 0, .sm_rx = 1, .tx_pin = PIN_SERVO_A_TX, .rx_pin = PIN_SERVO_A_RX, .txen_pin = PIN_SERVO_A_TXEN },
    { .sm_tx = 2, .sm_rx = 3, .tx_pin = PIN_SERVO_B_TX, .rx_pin = PIN_SERVO_B_RX, .txen_pin = PIN_SERVO_B_TXEN },
};
#define HD_CHANNELS (sizeof(channels) / sizeof(channels[0]))

#define HD_RX_BUF_SIZE 256
static uint8_t rx_buf[HD_RX_BUF_SIZE];
static volatile uint32_t rx_head = 0;
static volatile uint32_t rx_tail = 0;

static bool initialized = false;
static uint32_t current_baudrate = 0;
static uint tx_offset = 0;
static uint rx_offset = 0;

void half_duplex_init(uint32_t baudrate) {
    tx_offset = pio_add_program(HD_PIO, &pio_uart_tx_program);
    rx_offset = pio_add_program(HD_PIO, &pio_uart_rx_program);

    for (uint i = 0; i < HD_CHANNELS; i++) {
        const hd_channel_t *ch = &channels[i];
        gpio_init(ch->txen_pin);
        gpio_set_dir(ch->txen_pin, GPIO_OUT);
        gpio_put(ch->txen_pin, 0);  // default to RX mode

        pio_uart_tx_program_init(HD_PIO, ch->sm_tx, tx_offset, ch->tx_pin, baudrate);
        pio_uart_rx_program_init(HD_PIO, ch->sm_rx, rx_offset, ch->rx_pin, baudrate);
    }

    current_baudrate = baudrate;
    rx_head = rx_tail = 0;
    initialized = true;
}

void half_duplex_set_baudrate(uint32_t baudrate) {
    if (!initialized || baudrate == current_baudrate) return;

    float div = (float)clock_get_hz(clk_sys) / (baudrate * 8);
    for (uint i = 0; i < HD_CHANNELS; i++) {
        const hd_channel_t *ch = &channels[i];
        pio_sm_set_enabled(HD_PIO, ch->sm_tx, false);
        pio_sm_set_enabled(HD_PIO, ch->sm_rx, false);
        pio_sm_clear_fifos(HD_PIO, ch->sm_tx);
        pio_sm_clear_fifos(HD_PIO, ch->sm_rx);
        pio_sm_set_clkdiv(HD_PIO, ch->sm_tx, div);
        pio_sm_set_clkdiv(HD_PIO, ch->sm_rx, div);
        pio_sm_set_enabled(HD_PIO, ch->sm_tx, true);
        pio_sm_set_enabled(HD_PIO, ch->sm_rx, true);
    }
    current_baudrate = baudrate;
}

uint32_t half_duplex_write(const uint8_t *data, uint32_t len) {
    if (!initialized || len == 0) return 0;

    for (uint i = 0; i < HD_CHANNELS; i++) {
        gpio_put(channels[i].txen_pin, 1);  // drive both buses
    }

    // Interleave bytes across both TX state machines so the two channels
    // transmit in lock-step rather than one after the other.
    for (uint32_t j = 0; j < len; j++) {
        for (uint i = 0; i < HD_CHANNELS; i++) {
            pio_sm_put_blocking(HD_PIO, channels[i].sm_tx, (uint32_t)data[j]);
        }
    }

    // Wait for both TX FIFOs to drain and the last byte to finish shifting.
    for (uint i = 0; i < HD_CHANNELS; i++) {
        while (!pio_sm_is_tx_fifo_empty(HD_PIO, channels[i].sm_tx)) {
            tight_loop_contents();
        }
    }
    uint32_t byte_time_us = (10 * 1000000) / current_baudrate;
    if (byte_time_us < 10) byte_time_us = 10;
    sleep_us(byte_time_us * 2);

    for (uint i = 0; i < HD_CHANNELS; i++) {
        gpio_put(channels[i].txen_pin, 0);  // release both buses for RX
    }

    led_activity(LED_FEETECH);
    return len;
}

uint32_t half_duplex_read(uint8_t *data, uint32_t max_len) {
    if (!initialized) return 0;

    uint32_t count = 0;
    while (count < max_len && rx_head != rx_tail) {
        data[count++] = rx_buf[rx_tail];
        rx_tail = (rx_tail + 1) % HD_RX_BUF_SIZE;
    }

    if (count > 0) led_activity(LED_FEETECH);
    return count;
}

uint32_t half_duplex_available(void) {
    if (!initialized) return 0;
    return (rx_head - rx_tail + HD_RX_BUF_SIZE) % HD_RX_BUF_SIZE;
}

void half_duplex_task(void) {
    if (!initialized) return;

    for (uint i = 0; i < HD_CHANNELS; i++) {
        uint8_t sm_rx = channels[i].sm_rx;
        while (!pio_sm_is_rx_fifo_empty(HD_PIO, sm_rx)) {
            uint8_t c = (uint8_t)(pio_sm_get(HD_PIO, sm_rx) >> 24);
            uint32_t next_head = (rx_head + 1) % HD_RX_BUF_SIZE;
            if (next_head != rx_tail) {
                rx_buf[rx_head] = c;
                rx_head = next_head;
            }
        }
    }
}

int half_duplex_transact(const uint8_t *tx_data, uint32_t tx_len,
                         uint8_t *rx_data, uint32_t rx_max_len,
                         uint32_t timeout_us) {
    if (!initialized) return -1;

    rx_head = rx_tail = 0;
    for (uint i = 0; i < HD_CHANNELS; i++) {
        pio_sm_clear_fifos(HD_PIO, channels[i].sm_rx);
    }

    half_duplex_write(tx_data, tx_len);

    absolute_time_t deadline = make_timeout_time_us(timeout_us);
    uint32_t rx_count = 0;

    while (rx_count < rx_max_len) {
        if (absolute_time_diff_us(get_absolute_time(), deadline) <= 0) break;

        for (uint i = 0; i < HD_CHANNELS && rx_count < rx_max_len; i++) {
            uint8_t sm_rx = channels[i].sm_rx;
            while (!pio_sm_is_rx_fifo_empty(HD_PIO, sm_rx) && rx_count < rx_max_len) {
                rx_data[rx_count++] = (uint8_t)(pio_sm_get(HD_PIO, sm_rx) >> 24);
            }
        }
        // Keep USB serviced while we busy-wait on the servo bus, so probing
        // offline servos (each blocking up to timeout_us) can't starve
        // tud_task() long enough for the host to drop the device. Mirrors the
        // DDSM driver's ddsm_poll(). Safe in both config and normal mode.
        tud_task();
    }
    if (rx_count > 0) led_activity(LED_FEETECH);
    return (int)rx_count;
}
