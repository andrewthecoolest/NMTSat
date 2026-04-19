#ifndef SDR_H
#define SDR_H

#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <rtl-sdr.h>
#include "process.h"

#define SDR_BUFFER_SIZE (50 * 1024 * 1024)  /* 50 MB per device */

typedef struct SDR SDR;

struct SDR {
    rtlsdr_dev_t *dev;
    int           index;
    uint32_t      center_freq;
    uint32_t      sample_rate;

    /* Raw I/Q stream buffer (8-bit interleaved: I0 Q0 I1 Q1 ...) */
    uint8_t      *buf;
    size_t        buf_size;
    size_t        buf_used;

    /* Per-device output file */
    FILE         *file;
    char          filename[256];

    /* Streaming ring buffer (set before calling sdr_stream_start) */
    ring_buf_t  *ring;

    /* Function pointers */
    int (*gather_samples)(SDR *sdr, uint32_t n);
    void (*flush_to_file)(SDR *sdr);
};

/* Count connected RTL-SDR devices */
int sdr_count(void);

/* Open a device by index and populate an SDR struct.
 * Allocates the 50 MB buffer and opens output file.
 * gain: tuner gain in tenths of dB (e.g. 200 = 20.0 dB), or 0 for AGC.
 * Returns 0 on success, -1 on failure. */
int sdr_open(SDR *sdr, int index, uint32_t center_freq, uint32_t sample_rate,
             int gain, const char *output_filename);

/* Close device, flush remaining buffer to file, free resources. */
void sdr_close(SDR *sdr);

/* Gather exactly n raw bytes (I/Q pairs) into sdr->buf.
 * Overwrites previous buffer contents.
 * Returns 0 on success, -1 on failure. */
int sdr_gather_samples(SDR *sdr, uint32_t n);

/* Write sdr->buf[0..buf_used] to sdr->file. */
void sdr_flush_to_file(SDR *sdr);

/* Start continuous async streaming into sdr->ring.
 * Must set sdr->ring before calling.
 * Spawns an internal thread — returns immediately.
 * Returns 0 on success, -1 on failure. */
int sdr_stream_start(SDR *sdr);

/* Stop the async stream and join the internal thread. */
void sdr_stream_stop(SDR *sdr);

#endif /* SDR_H */
