/*
 * Debug logging to USB CDC #2.
 *
 * For low-rate, human-readable status: initialization banners, which motor
 * IDs were found, bus/zenoh state. NOT for per-message traffic. Output goes
 * to the debug CDC port (when a host has it open) and also to the low-level
 * stdio UART, so a wired console still sees it.
 */

#ifndef AXON_DBG_H
#define AXON_DBG_H

void dbg_printf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

#endif // AXON_DBG_H
