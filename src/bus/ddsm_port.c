#include "ddsm_port.h"

#include "pins.h"
#include "led.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"

#include "uart_tx.pio.h"
#include "uart_rx.pio.h"

// State machine allocation (see pins.h for the full PIO map). PIO0 is fully
// taken by the two servo buses, so the four wheel motors live on PIO1 and
// PIO2 (the lidar moved to hardware uart1, and there is no NeoPixel):
//   FR: PIO1 SM0/SM1   FL: PIO1 SM2/SM3
//   BR: PIO2 SM0/SM1   BL: PIO2 SM2/SM3
// ddsm_port_init() loads the uart_tx/uart_rx programs into PIO1 and PIO2.

typedef struct {
    PIO pio;
    uint8_t sm_tx;
    uint8_t sm_rx;
    uint8_t tx_pin;
    uint8_t rx_pin;
    led_id_t led;
} ddsm_port_cfg_t;

static const ddsm_port_cfg_t port_cfg[DDSM_PORT_COUNT] = {
    [DDSM_FR] = {.pio = pio1, .sm_tx = 0, .sm_rx = 1,
                 .tx_pin = PIN_DDSM_FR_TX, .rx_pin = PIN_DDSM_FR_RX, .led = LED_UART0},
    [DDSM_FL] = {.pio = pio1, .sm_tx = 2, .sm_rx = 3,
                 .tx_pin = PIN_DDSM_FL_TX, .rx_pin = PIN_DDSM_FL_RX, .led = LED_UART0},
    [DDSM_BR] = {.pio = pio2, .sm_tx = 0, .sm_rx = 1,
                 .tx_pin = PIN_DDSM_BR_TX, .rx_pin = PIN_DDSM_BR_RX, .led = LED_UART0},
    [DDSM_BL] = {.pio = pio2, .sm_tx = 2, .sm_rx = 3,
                 .tx_pin = PIN_DDSM_BL_TX, .rx_pin = PIN_DDSM_BL_RX, .led = LED_UART0},
};

#define DDSM_RX_BUF_SIZE 128

typedef struct {
    uint8_t buffer[DDSM_RX_BUF_SIZE];
    volatile uint32_t head;
    volatile uint32_t tail;
} ddsm_port_state_t;

static ddsm_port_state_t port_state[DDSM_PORT_COUNT];
static bool initialized = false;

void ddsm_port_init(uint32_t baudrate) {
    // The DDSM motors are on GP32-39. On the RP2350/RP2354B a PIO instance can
    // only address a 32-pin window; the default base (0) covers GP0-31, so
    // PIO1/PIO2 cannot reach GP32-39 at all — the SMs silently wrap to GP0-7.
    // Shift both blocks to the GP16-47 window, before the SMs are configured.
    // (PIO0/servos stay at base 0 for GP5-8.)
    pio_set_gpio_base(pio1, 16);
    pio_set_gpio_base(pio2, 16);

    // Load one copy of the uart_tx/uart_rx programs into each PIO block we use
    // (PIO1 for the front wheels, PIO2 for the back wheels).
    uint tx_off[2], rx_off[2];
    tx_off[0] = pio_add_program(pio1, &pio_uart_tx_program);
    rx_off[0] = pio_add_program(pio1, &pio_uart_rx_program);
    tx_off[1] = pio_add_program(pio2, &pio_uart_tx_program);
    rx_off[1] = pio_add_program(pio2, &pio_uart_rx_program);

    for (int i = 0; i < DDSM_PORT_COUNT; i++) {
        const ddsm_port_cfg_t *cfg = &port_cfg[i];
        uint b = (cfg->pio == pio1) ? 0 : 1;
        pio_uart_tx_program_init(cfg->pio, cfg->sm_tx, tx_off[b], cfg->tx_pin, baudrate);
        pio_uart_rx_program_init(cfg->pio, cfg->sm_rx, rx_off[b], cfg->rx_pin, baudrate);
        port_state[i].head = 0;
        port_state[i].tail = 0;
    }
    initialized = true;
}

uint32_t ddsm_port_write(ddsm_port_id_t id, const uint8_t *data, uint32_t len) {
    if (!initialized || id >= DDSM_PORT_COUNT || len == 0) return 0;
    const ddsm_port_cfg_t *cfg = &port_cfg[id];

    for (uint32_t i = 0; i < len; i++) {
        pio_sm_put_blocking(cfg->pio, cfg->sm_tx, (uint32_t)data[i]);
    }
    while (!pio_sm_is_tx_fifo_empty(cfg->pio, cfg->sm_tx)) {
        tight_loop_contents();
    }
    led_activity(cfg->led);
    return len;
}

uint32_t ddsm_port_read(ddsm_port_id_t id, uint8_t *data, uint32_t max_len) {
    if (!initialized || id >= DDSM_PORT_COUNT) return 0;
    ddsm_port_state_t *st = &port_state[id];

    uint32_t count = 0;
    while (count < max_len && st->head != st->tail) {
        data[count++] = st->buffer[st->tail];
        st->tail = (st->tail + 1) % DDSM_RX_BUF_SIZE;
    }
    return count;
}

uint32_t ddsm_port_available(ddsm_port_id_t id) {
    if (!initialized || id >= DDSM_PORT_COUNT) return 0;
    ddsm_port_state_t *st = &port_state[id];
    return (st->head - st->tail + DDSM_RX_BUF_SIZE) % DDSM_RX_BUF_SIZE;
}

void ddsm_port_task(void) {
    if (!initialized) return;

    for (int i = 0; i < DDSM_PORT_COUNT; i++) {
        const ddsm_port_cfg_t *cfg = &port_cfg[i];
        ddsm_port_state_t *st = &port_state[i];

        while (!pio_sm_is_rx_fifo_empty(cfg->pio, cfg->sm_rx)) {
            uint8_t c = (uint8_t)(pio_sm_get(cfg->pio, cfg->sm_rx) >> 24);
            uint32_t next_head = (st->head + 1) % DDSM_RX_BUF_SIZE;
            if (next_head != st->tail) {
                st->buffer[st->head] = c;
                st->head = next_head;
            }
        }
    }
}
