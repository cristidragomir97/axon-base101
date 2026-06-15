/*
 * Axon ROS Node Firmware
 * ======================
 *
 * The board (Axon 2 / RP2354B) is a native ROS 2 node: motor control for the
 * LLMy robot runs in firmware and is exposed over zenoh (Pico-ROS + zenoh-pico,
 * rmw_zenoh compatible) carried on USB CDC #0.
 *
 *   USB Interface     Function
 *   ------------------------------------------------------
 *   CDC #0            zenoh serial transport (Pico-ROS)
 *   CDC #1            Lidar UART passthrough
 *   CDC #2            Debug log (init info, IDs, status)
 *
 *   Bus               Hardware
 *   ------------------------------------------------------
 *   Servo bus         Feetech half-duplex, TWO channels merged in software (PIO0)
 *   DDSM210 x4        one UART per wheel motor (PIO1/PIO2)
 *   Lidar UART        hardware uart1 (GPIO4/5)
 *   IMU               BNO055 on i2c1 (GPIO26/27)
 *   Activity LEDs     classic GPIO (no NeoPixels, single core)
 *
 * ROS topics (via zenohd + rmw_zenoh on the host):
 *   sub  /motor_manager/base_cmd      std_msgs/Float64MultiArray
 *   sub  /motor_manager/arm_cmd       std_msgs/Float64MultiArray
 *   pub  /motor_manager/joint_states  sensor_msgs/JointState
 *   pub  /motor_telemetry/<joint>/*   Float32 / Int32
 *   pub  /imu/data, /imu/mag, /imu/temperature
 */

#include <stdio.h>

#include "pico/stdlib.h"
#include "tusb.h"

#include "picoros.h"

#include "pins.h"
#include "usb_descriptors.h"
#include "led.h"
#include "dbg.h"
#include "bus/half_duplex.h"
#include "bus/lidar_uart.h"
#include "bus/ddsm_port.h"
#include "config/axon_cfg.h"
#include "config/cfg_console.h"
#include "ros/axon_config.h"
#include "ros/axon_node.h"
#include "zenoh_port/axon_zenoh.h"

//--------------------------------------------------------------------
// Lidar passthrough: CDC #1 <-> lidar UART
//--------------------------------------------------------------------
static void lidar_bridge(void) {
    uint8_t buf[64];

    // USB -> Lidar
    if (tud_cdc_n_available(CDC_IDX_LIDAR)) {
        uint32_t count = tud_cdc_n_read(CDC_IDX_LIDAR, buf, sizeof(buf));
        if (count > 0) lidar_uart_write(buf, count);
    }

    // Lidar -> USB
    while (lidar_uart_available() > 0 && tud_cdc_n_write_available(CDC_IDX_LIDAR) > 0) {
        uint32_t want = tud_cdc_n_write_available(CDC_IDX_LIDAR);
        if (want > sizeof(buf)) want = sizeof(buf);
        uint32_t count = lidar_uart_read(buf, want);
        if (count == 0) break;
        tud_cdc_n_write(CDC_IDX_LIDAR, buf, count);
    }
    tud_cdc_n_write_flush(CDC_IDX_LIDAR);
}

//--------------------------------------------------------------------
// Background I/O poll
//
// Runs from the main loop and from inside the zenoh port whenever it
// waits on serial data or sleeps. Never touches zenoh.
//--------------------------------------------------------------------
static void io_poll(void) {
    tud_task();
    half_duplex_task();
    ddsm_port_task();
    lidar_uart_task();
    lidar_bridge();
    led_task();
}

//--------------------------------------------------------------------
// Config mode
//
// Entered when the sniff/config button (PIN_CONFIG_BUTTON) is held LOW at
// boot. Brings up USB and serves the interactive JSON console on CDC #2;
// motors, lidar and zenoh are intentionally NOT started, so the robot stays
// still while it is reconfigured. Exits only by reboot (e.g. the console's
// {"cmd":"reboot"}). Never returns.
//--------------------------------------------------------------------
// noinline + a volatile capture so the optimizer treats the button state as a
// genuine runtime value. (Without this, -O3 inlined the noreturn config loop
// and collapsed the branch, dropping the entire normal-mode tail of main.)
static bool __attribute__((noinline)) config_button_pressed(void) {
    gpio_init(PIN_CONFIG_BUTTON);
    gpio_set_dir(PIN_CONFIG_BUTTON, GPIO_IN);
    gpio_pull_up(PIN_CONFIG_BUTTON);
    sleep_ms(2);  // let the pull-up settle
    volatile bool pressed = (gpio_get(PIN_CONFIG_BUTTON) == 0);
    gpio_deinit(PIN_CONFIG_BUTTON);  // release the pull-up after the read
    return pressed;
}

static void __attribute__((noinline)) run_config_mode(void) {
    // Bring up the motor buses so the bench test commands ("motors"/"wheel"/
    // "servo"/"stop") can drive motors. zenoh/lidar/IMU stay off.
    half_duplex_init(AXON_ST_BAUD);
    ddsm_port_init(AXON_DDSM_BAUD);

    tusb_init();
    uint32_t start = time_us_32();
    while (!tud_ready() && (time_us_32() - start) < 10000000) {
        tud_task();
        sleep_ms(10);
    }

    axon_cfg_load();
    axon_node_motors_init();  // detect motors + set modes (servos per config)
    dbg_printf("\n[axon] CONFIG MODE (sniff button held) — JSON console on CDC #2\n");
    dbg_printf("[axon] config cmds: get|set|save|defaults|reboot ; "
               "test cmds: motors|wheel|servo|setid|stop\n");
    dbg_printf("[axon] servos %s, %u configured\n",
               g_cfg.servos_enabled ? "enabled" : "disabled", g_cfg.servo_count);

    bool on = false;
    uint64_t next_blink = 0;
    while (true) {
        tud_task();
        cfg_console_task();
        led_task();
        uint64_t now = time_us_64();
        if (now >= next_blink) {  // slow ~0.5 Hz blink signals config mode
            next_blink = now + 1000000;  // 1 s on, 1 s off
            on = !on;
            led_all(on);
        }
    }
}

// Host changed the baudrate of a CDC port: forward to the lidar UART.
void tud_cdc_line_coding_cb(uint8_t itf, cdc_line_coding_t const *coding) {
    if (itf == CDC_IDX_LIDAR) {
        lidar_uart_set_baudrate(coding->bit_rate);
    }
}

//--------------------------------------------------------------------
// Main
//--------------------------------------------------------------------
int main(void) {
    stdio_init_all();

    // Classic GPIO activity LEDs (single core).
    led_init();
    led_all(false);

    // Sniff/config button (SNIFF_ENABLE, GP29) held at boot -> interactive
    // config mode (never returns). GP29 is dedicated, so unlike the old GP32
    // assignment it does not collide with any motor bus. Otherwise fall
    // through to normal operation.
    if (config_button_pressed()) {
        run_config_mode();
    }

    // Buses. half_duplex_init() loads the PIO0 uart programs for the servo
    // bus; ddsm_port_init() loads its own on PIO1/PIO2; the lidar is on the
    // uart1 hardware peripheral.
    half_duplex_init(AXON_ST_BAUD);
    led_set(LED_FEETECH, true);
    lidar_uart_init(AXON_LIDAR_DEFAULT_BAUD);
    led_set(LED_UART1, true);
    ddsm_port_init(AXON_DDSM_BAUD);
    led_set(LED_UART0, true);

    // USB
    tusb_init();
    dbg_printf("Waiting for USB connection...\n");
    uint32_t usb_start = time_us_32();
    while (!tud_ready() && (time_us_32() - usb_start) < 5000000) {
        tud_task();
        sleep_ms(10);
    }
    dbg_printf(tud_ready() ? "USB enumerated\n" : "USB enumeration timeout (continuing)\n");
    led_flash_all(2, 100, 100);
    led_all(false);

    // Runtime config (servo list / enable) from flash, else compiled defaults.
    bool cfg_stored = axon_cfg_load();
    dbg_printf("[axon] config: %s, servos %s (%u configured)\n",
               cfg_stored ? "loaded from flash" : "defaults",
               g_cfg.servos_enabled ? "enabled" : "disabled", g_cfg.servo_count);

    // Motors and IMU can be initialized regardless of the host being up.
    axon_node_motors_init();
    axon_imu_init();

    dbg_printf("\nAxon ROS Node v3.0 (Axon 2)\n");
    dbg_printf("CDC #0: zenoh (%s)   CDC #1: lidar   CDC #2: debug\n", AXON_ZENOH_LOCATOR);

    // zenoh session over CDC #0. Generous poll timeout during the
    // handshake; the idle poll keeps USB + lidar + motor buses alive
    // while zenoh blocks.
    axon_zenoh_set_idle_poll(io_poll);
    axon_zenoh_set_poll_timeout_ms(1000);

    picoros_interface_t ifx = {
        .mode = AXON_ZENOH_MODE,
        .locator = AXON_ZENOH_LOCATOR,
    };

    while (picoros_interface_init(&ifx) != PICOROS_OK) {
        dbg_printf("zenoh session not ready (is zenohd with the serial endpoint up?), retrying...\n");
        led_activity(LED_CAN);
        for (int i = 0; i < 100; i++) {  // ~1 s, keeping I/O alive
            io_poll();
            sleep_ms(10);
        }
    }
    dbg_printf("zenoh session up\n");
    led_set(LED_CAN, true);

    axon_zenoh_set_poll_timeout_ms(AXON_ZENOH_POLL_TIMEOUT_MS);

    if (!axon_node_declare()) {
        dbg_printf("FATAL: node declaration failed\n");
        while (true) {
            led_flash_all(1, 100, 400);
            io_poll();
            sleep_ms(100);
        }
    }

    // Main loop: zenoh rx (bounded by the poll timeout) + telemetry.
    while (true) {
        picoros_single_threaded_loop(&ifx);
        io_poll();
        axon_node_spin();
    }
}
