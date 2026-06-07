#ifndef BLADE_H
#define BLADE_H

#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <libbladeRF.h>
#include <math.h>

typedef struct BLADE BLADE;

struct BLADE {
    struct bladerf *dev;
    int             index;
    uint64_t        center_freq;
    uint32_t        sample_rate;
    uint32_t        bandwidth;
    int             gain;        /* TX gain in dB */

    /* Test tone buffer (SC16_Q11, interleaved I/Q) */
    int16_t        *tone_buf;
    uint32_t        tone_samples; /* total number of I/Q pairs */

    /* Function pointers for polymorphing */
    int  (*transmit)(BLADE *blade, int16_t *buf, uint32_t n);
};

/* Count connected bladeRF devices */
int blade_count(void);

/* Open a device by index and configure TX.
 * Returns 0 on success, -1 on failure. */
int blade_open(BLADE *blade, int index, uint64_t center_freq,
               uint32_t sample_rate, uint32_t bandwidth, int gain);

/* Close device and release resources. */
void blade_close(BLADE *blade);

/* Generate a test tone into blade->tone_buf.
 * tone_hz: baseband offset frequency in Hz (e.g. 10000 for 10 kHz above center)
 * on_ms / off_ms: on and off durations in milliseconds (use 1, 1 for standard blip)
 * Allocates blade->tone_buf — freed by blade_close().
 * Returns 0 on success, -1 on failure. */
int create_test_tone(BLADE *blade, float tone_hz, uint32_t on_ms, uint32_t off_ms);

/* Continuously transmit blade->tone_buf in a loop until *stop_flag is set.
 * Designed to run in its own thread. */
int blade_transmit_loop(BLADE *blade, volatile int *stop_flag);

/* Transmit n I/Q sample pairs from buf.
 * buf is 16-bit signed interleaved SC16_Q11 format.
 * Returns 0 on success, -1 on failure. */
int blade_transmit(BLADE *blade, int16_t *buf, uint32_t n);

#endif /* BLADE_H */
