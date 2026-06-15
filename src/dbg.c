#include "dbg.h"

#include <stdarg.h>
#include <stdio.h>

#include "tusb.h"
#include "usb_descriptors.h"

void dbg_printf(const char *fmt, ...) {
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n < 0) return;
    if (n >= (int)sizeof(buf)) n = sizeof(buf) - 1;

    // Wired stdio (debug UART) always gets it.
    fwrite(buf, 1, (size_t)n, stdout);

    // The debug CDC gets it only when a host has the port open, so an
    // unread port can never block or back up the firmware.
    if (tud_cdc_n_connected(CDC_IDX_DEBUG)) {
        tud_cdc_n_write(CDC_IDX_DEBUG, buf, (uint32_t)n);
        tud_cdc_n_write_flush(CDC_IDX_DEBUG);
    }
}
