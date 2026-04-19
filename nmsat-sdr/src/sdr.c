#include "sdr.h"
#include "log.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <pthread.h>
#include <sched.h>

// gives count of sdrs available
int sdr_count(void) {
    return (int)rtlsdr_get_device_count();
}

int sdr_open(SDR *sdr, int index, uint32_t center_freq, uint32_t sample_rate,
             const char *output_filename)
{
    memset(sdr, 0, sizeof(SDR));

    LOG_INFO(TAG_RTL " Opening device %d...\n", index);
    if (rtlsdr_open(&sdr->dev, (uint32_t)index) < 0) {
        LOG_ERR(TAG_RTL " Device %d: FAILED\n", index);
        return -1;
    }
    LOG_INFO(TAG_RTL " Device %d: ", index);
    LOG_OK("OK\n");

    sdr->index       = index;
    sdr->center_freq = center_freq;
    sdr->sample_rate = sample_rate;

    rtlsdr_set_center_freq(sdr->dev, center_freq);
    rtlsdr_set_sample_rate(sdr->dev, sample_rate);
    rtlsdr_set_tuner_gain_mode(sdr->dev, 1);  /* 1 = manual gain */
    rtlsdr_set_tuner_gain(sdr->dev, 496);     /* 49.6 dB — adjust as needed */
    rtlsdr_reset_buffer(sdr->dev);

    /* Allocate 50 MB stream buffer */
    sdr->buf_size = SDR_BUFFER_SIZE;
    LOG_INFO(TAG_RTL " Allocating buffer for device %d... ", index);
    sdr->buf      = (uint8_t *)malloc(sdr->buf_size);
    if (!sdr->buf) {
        LOG_ERR("FAILED\n");
        rtlsdr_close(sdr->dev);
        return -1;
    }
    sdr->buf_used = 0;
    LOG_OK("OK\n");

    /* Open output file */
    strncpy(sdr->filename, output_filename, sizeof(sdr->filename) - 1);
    LOG_INFO(TAG_RTL " Opening output file '%s'... ", sdr->filename);
    sdr->file = fopen(sdr->filename, "wb");
    if (!sdr->file) {
        LOG_ERR("FAILED\n");
        free(sdr->buf);
        rtlsdr_close(sdr->dev);
        return -1;
    }
    LOG_OK("OK\n");

    /* Bind function pointers */
    sdr->gather_samples = sdr_gather_samples;
    sdr->flush_to_file  = sdr_flush_to_file;

    return 0;
}

void sdr_close(SDR *sdr)
{
    if (!sdr) return;
    LOG_INFO(TAG_RTL " Closing device %d... ", sdr->index);

    if (sdr->buf_used > 0 && sdr->file)
        sdr_flush_to_file(sdr);

    if (sdr->file)  { fclose(sdr->file);      sdr->file = NULL; }
    if (sdr->buf)   { free(sdr->buf);         sdr->buf  = NULL; }
    if (sdr->dev)   { rtlsdr_close(sdr->dev); sdr->dev  = NULL; }
    LOG_OK("OK\n");
}

int sdr_gather_samples(SDR *sdr, uint32_t n)
{
    if (!sdr || !sdr->dev || !sdr->buf) return -1;

    if (n > sdr->buf_size) {
        LOG_ERR(TAG_RTL " gather_samples: requested %u bytes exceeds buffer size %zu\n",
                n, sdr->buf_size);
        return -1;
    }

    int n_read = 0;
    int ret = rtlsdr_read_sync(sdr->dev, sdr->buf, (int)n, &n_read);
    if (ret < 0) {
        LOG_ERR(TAG_RTL " gather_samples: read error on device %d\n", sdr->index);
        sdr->buf_used = 0;
        return -1;
    }

    sdr->buf_used = (size_t)n_read;
    return 0;
}

void sdr_flush_to_file(SDR *sdr)
{
    if (!sdr || !sdr->file || sdr->buf_used == 0) return;

    size_t written = fwrite(sdr->buf, 1, sdr->buf_used, sdr->file);
    if (written != sdr->buf_used)
        LOG_ERR(TAG_RTL " flush_to_file: short write on device %d (%zu of %zu bytes)\n",
                sdr->index, written, sdr->buf_used);

    sdr->buf_used = 0;
}

/* ------------------------------------------------------------------ */
/* Async streaming                                                      */
/* ------------------------------------------------------------------ */

static void stream_callback(unsigned char *buf, uint32_t len, void *ctx)
{
    SDR *sdr = (SDR *)ctx;
    if (sdr->ring)
        ring_write(sdr->ring, buf, len);
}

static void *stream_thread(void *ctx)
{
    SDR *sdr = (SDR *)ctx;

    struct sched_param sp = { .sched_priority = sched_get_priority_max(SCHED_FIFO) };
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);

    /*
    buf_num = 8, 8 USB transfer buffers in flight
    buf_len = 16384, 16KB per callback (must be a multiple of 512)
    */
    rtlsdr_read_async(sdr->dev, stream_callback, sdr, 8, 2048);
    return NULL;
}

int sdr_stream_start(SDR *sdr)
{
    if (!sdr || !sdr->ring) {
        LOG_ERR(TAG_RTL " sdr_stream_start: ring not set on device %d\n", sdr->index);
        return -1;
    }
    LOG_INFO(TAG_RTL " Starting async stream on device %d\n", sdr->index);
    pthread_t *t = (pthread_t *)malloc(sizeof(pthread_t));
    if (!t) return -1;
    /* store thread handle in buf (repurposed — not used during streaming) */
    sdr->buf = (uint8_t *)t;
    pthread_create(t, NULL, stream_thread, sdr);
    return 0;
}

void sdr_stream_stop(SDR *sdr)
{
    if (!sdr || !sdr->dev) return;
    LOG_INFO(TAG_RTL " Stopping async stream on device %d\n", sdr->index);
    rtlsdr_cancel_async(sdr->dev);
    pthread_t *t = (pthread_t *)sdr->buf;
    if (t) {
        pthread_join(*t, NULL);
        free(t);
        sdr->buf = NULL;
    }
}
