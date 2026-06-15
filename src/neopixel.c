#include "neopixel.h"
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "ws2812.pio.h"
#include <stdio.h>
#include <stdlib.h>

// NeoPixel state - use PIO2 for Core 1 LED controller
static PIO neo_pio = pio2;
static uint neo_sm = 0;  // Use SM0 on PIO2 (dedicated to Core 1)
static uint neo_offset;
static uint neo_pin;
static uint neo_num_pixels;
static uint32_t neo_frequency;
static neopixel_color_t *neo_pixel_buffer = NULL;

// Convert RGB to GRB 32-bit value (WS2812 format)
static inline uint32_t neopixel_rgb_to_grb(uint8_t r, uint8_t g, uint8_t b) {
    return ((uint32_t)(g) << 16) | ((uint32_t)(r) << 8) | (uint32_t)(b);
}

bool neopixel_init(uint pin, uint num_pixels, uint32_t frequency) {
    neo_pin = pin;
    neo_num_pixels = num_pixels;
    
    // Allocate pixel buffer
    if (neo_pixel_buffer) {
        free(neo_pixel_buffer);
    }
    neo_pixel_buffer = malloc(num_pixels * sizeof(neopixel_color_t));
    if (!neo_pixel_buffer) {
        printf("Failed to allocate pixel buffer\n");
        return false;
    }
    
    // Clear buffer
    for (uint i = 0; i < num_pixels; i++) {
        neo_pixel_buffer[i] = NEOPIXEL_COLOR_OFF;
    }
    
    // Enable pull-up resistor on data pin for better signal integrity at 3.3V
    gpio_set_pulls(pin, true, false);
    
    // Add PIO program to PIO memory
    neo_offset = pio_add_program(neo_pio, &ws2812_program);
    
    // Store frequency and initialize PIO state machine
    neo_frequency = frequency;
    ws2812_program_init(neo_pio, neo_sm, neo_offset, pin, frequency, false);
    
    printf("NeoPixel initialized: %d pixels on GPIO%d at %lu Hz\n", 
           num_pixels, pin, frequency);
    
    // Initial clear
    neopixel_show();
    
    return true;
}

void neopixel_set_pixel(uint pixel_index, uint8_t red, uint8_t green, uint8_t blue) {
    if (pixel_index >= neo_num_pixels || !neo_pixel_buffer) {
        return;
    }
    
    neo_pixel_buffer[pixel_index].r = red;
    neo_pixel_buffer[pixel_index].g = green;
    neo_pixel_buffer[pixel_index].b = blue;
}

void neopixel_set_pixel_color(uint pixel_index, neopixel_color_t color) {
    neopixel_set_pixel(pixel_index, color.r, color.g, color.b);
}

void neopixel_fill(uint8_t red, uint8_t green, uint8_t blue) {
    if (!neo_pixel_buffer) return;
    
    for (uint i = 0; i < neo_num_pixels; i++) {
        neo_pixel_buffer[i].r = red;
        neo_pixel_buffer[i].g = green;
        neo_pixel_buffer[i].b = blue;
    }
}

void neopixel_clear(void) {
    neopixel_fill(0, 0, 0);
}

void neopixel_show(void) {
    if (!neo_pixel_buffer) return;
    
    // Send pixel data to PIO with delays to reduce power spikes
    for (uint i = 0; i < neo_num_pixels; i++) {
        uint32_t grb_value = neopixel_rgb_to_grb(
            neo_pixel_buffer[i].r,
            neo_pixel_buffer[i].g, 
            neo_pixel_buffer[i].b
        );
        pio_sm_put_blocking(neo_pio, neo_sm, grb_value << 8u);
        
        // Small delay between LEDs to reduce simultaneous switching current
        if (i < neo_num_pixels - 1) {
            sleep_us(10);
        }
    }
    
    // Longer reset pulse for reliability at 3.3V
    sleep_us(500);
}

uint neopixel_get_count(void) {
    return neo_num_pixels;
}

bool neopixel_set_frequency(uint32_t frequency) {
    if (!neo_pixel_buffer) return false;
    
    neo_frequency = frequency;
    
    // Reinitialize PIO state machine with new frequency
    ws2812_program_init(neo_pio, neo_sm, neo_offset, neo_pin, frequency, false);
    
    printf("NeoPixel frequency changed to %lu Hz\n", frequency);
    return true;
}

uint32_t neopixel_get_frequency(void) {
    return neo_frequency;
}