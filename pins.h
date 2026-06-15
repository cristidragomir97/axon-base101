#ifndef AXON_PINOUT_H
#define AXON_PINOUT_H

// ============================================================
// RoboCore Axon — RP2350 Pin Assignments
// Generated from KiCad schematic, GPIO0–GPIO28
// ============================================================

// --- RS485 Bus (isolated, ADM2587E or similar) ---
#define PIN_RS485_TX         0
#define PIN_RS485_RX         1
#define PIN_RS485_DE         2   // direction enable

// --- UART0 (general purpose / DDSM?) ---
#define PIN_UART0_TX         3
#define PIN_UART0_RX         4

// --- UART1 (general purpose / lidar passthrough?) ---
#define PIN_UART1_TX         5
#define PIN_UART1_RX         6

// --- UART2 (optional additional general purpose) ---
#ifdef ENABLE_UART2
#define PIN_UART2_TX        19
#define PIN_UART2_RX        20
#endif

// --- Motor Bus (unified Feetech/Dynamixel half-duplex) ---
#define PIN_MOTOR_TX         7
#define PIN_MOTOR_RX         8
#define PIN_MOTOR_TXEN      16   // TX enable (direction control)

// --- SPI (mikroBUS / MCP2518FD CAN controller) ---
#define PIN_CAN_INT          9   // CAN interrupt pin (was PIN_SPI_RST)
#define PIN_SPI_SCK         10
#define PIN_SPI_MOSI        11
#define PIN_SPI_MISO        12
#define PIN_SPI_CS          13

// --- I2C (Qwiic sensors) ---
#define PIN_SDA             14
#define PIN_SCL             15

// --- NeoPixel Activity LEDs (6 LEDs on single data line) ---
#define PIN_NEOPIXELS       18   // GPIO18 - NeoPixel Data Out

// --- GPIO Pins (general purpose) ---
#define PIN_SNIFF_ENABLE    17   // GPIO17 - Bus Sniffer Enable
#ifndef ENABLE_UART2
#define PIN_GP19            19   // GPIO19 - General Purpose (or UART2_TX if enabled)
#define PIN_GP20            20   // GPIO20 - General Purpose (or UART2_RX if enabled)
#endif
#define PIN_GP21            21   // GPIO21 - General Purpose
#define PIN_GP22            22   // GPIO22 - General Purpose

// ============================================================
// PIO Analysis — RP2350 has 3 PIO blocks × 4 SM = 12 total
//
// Hardware UART candidates:
//   GPIO0/1 (RS485) → HW UART0 TX/RX ✓ (exact match)
//   All others: pin assignments don't align with HW UART func select
//
// Assuming RS485 uses HW UART0:
//
//   PIO0:
//     SM0: MOTOR TX  (GPIO7)
//     SM1: MOTOR RX  (GPIO8)
//     SM2: free
//     SM3: free
//
//   PIO1:
//     SM0: UART0 TX  (GPIO3)
//     SM1: UART0 RX  (GPIO4)
//     SM2: UART1 TX  (GPIO5)
//     SM3: UART1 RX  (GPIO6)
//
//   PIO2 (Core 1 - LED Controller):
//     SM0: NeoPixel WS2812 (GPIO18)
//     SM1: [UART2 TX (GPIO19) if ENABLE_UART2]
//     SM2: [UART2 RX (GPIO20) if ENABLE_UART2] 
//     SM3: free
//
//   Total: 7 of 12 state machines used (9 if UART2 enabled), 5 free (3 if UART2 enabled)
//
// If RS485 also needs PIO (e.g. for timing-critical DE control):
//     Move RS485 TX/RX to PIO0 SM2/SM3 → 9 used, 3 free
//
// UART1 moved from GPIO19/20 to GPIO5/6 in new board revision
// (GPIO5/6 were previously Feetech pins)

// ============================================================

#endif // AXON_PINOUT_H