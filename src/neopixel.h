#ifndef NEOPIXEL_H
#define NEOPIXEL_H

#include <stdint.h>
#include <stdbool.h>
#include "hardware/pio.h"

// Simple NeoPixel library for Pico SDK
// Inspired by NeoPixelConnect but adapted for C and Pico SDK

typedef struct {
    uint8_t r, g, b;
} neopixel_color_t;

// Initialize NeoPixel strip
// pin: GPIO pin number
// num_pixels: Number of LEDs in strip
// frequency: LED communication frequency (e.g., 800000 for 800kHz)
bool neopixel_init(uint pin, uint num_pixels, uint32_t frequency);

// Set individual pixel color (does not update strip automatically)
void neopixel_set_pixel(uint pixel_index, uint8_t red, uint8_t green, uint8_t blue);

// Set individual pixel color using color struct
void neopixel_set_pixel_color(uint pixel_index, neopixel_color_t color);

// Fill all pixels with same color
void neopixel_fill(uint8_t red, uint8_t green, uint8_t blue);

// Clear all pixels (set to black/off)
void neopixel_clear(void);

// Update the LED strip (send buffered data to LEDs)
void neopixel_show(void);

// Get number of pixels
uint neopixel_get_count(void);

// Change frequency after initialization
bool neopixel_set_frequency(uint32_t frequency);

// Get current frequency
uint32_t neopixel_get_frequency(void);

// Predefined colors - dimmed for 3.3V operation and missing decoupling caps
#define NEOPIXEL_COLOR_OFF     ((neopixel_color_t){0, 0, 0})
#define NEOPIXEL_COLOR_RED     ((neopixel_color_t){64, 0, 0})
#define NEOPIXEL_COLOR_GREEN   ((neopixel_color_t){0, 64, 0})
#define NEOPIXEL_COLOR_BLUE    ((neopixel_color_t){0, 0, 64})
#define NEOPIXEL_COLOR_WHITE   ((neopixel_color_t){64, 64, 64})
#define NEOPIXEL_COLOR_YELLOW  ((neopixel_color_t){64, 64, 0})
#define NEOPIXEL_COLOR_CYAN    ((neopixel_color_t){0, 64, 64})
#define NEOPIXEL_COLOR_MAGENTA ((neopixel_color_t){64, 0, 64})

#endif // NEOPIXEL_H