#include "blade.h"
#include "log.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

int blade_count(void)
{
    struct bladerf_devinfo *list = NULL;
    int count = bladerf_get_device_list(&list);
    if (count > 0)
        bladerf_free_device_list(list);
    return count < 0 ? 0 : count;
}

int blade_open(BLADE *blade, int index, uint64_t center_freq,
               uint32_t sample_rate, uint32_t bandwidth, int gain)
{
    memset(blade, 0, sizeof(BLADE));

    struct bladerf_devinfo *list = NULL;
    int count = bladerf_get_device_list(&list);
    if (count <= 0 || index >= count) {
        LOG_ERR(TAG_BLADE " blade_open: device %d not found\n", index);
        return -1;
    }

    LOG_INFO(TAG_BLADE " Opening device %d... ", index);
    int ret = bladerf_open_with_devinfo(&blade->dev, &list[index]);
    bladerf_free_device_list(list);
    if (ret < 0) {
        LOG_ERR("FAILED (%s)\n", bladerf_strerror(ret));
        return -1;
    }
    LOG_OK("OK\n");

    blade->index       = index;
    blade->center_freq = center_freq;
    blade->sample_rate = sample_rate;
    blade->bandwidth   = bandwidth;
    blade->gain        = gain;

    /* Configure TX channel */
    struct bladerf_rational_rate rate = {0, sample_rate, 1};
    bladerf_set_rational_sample_rate(blade->dev, BLADERF_CHANNEL_TX(0), &rate, NULL);
    bladerf_set_frequency(blade->dev, BLADERF_CHANNEL_TX(0), center_freq);
    bladerf_set_bandwidth(blade->dev, BLADERF_CHANNEL_TX(0), bandwidth, NULL);
    bladerf_set_gain(blade->dev, BLADERF_CHANNEL_TX(0), gain);

    const struct bladerf_range *gain_range = NULL;
    if (bladerf_get_gain_range(blade->dev, BLADERF_CHANNEL_TX(0), &gain_range) == 0 && gain_range)
        LOG_INFO(TAG_BLADE " TX gain range: [%.2f, %.2f] dB (step %.4f)\n",
                 gain_range->min  * gain_range->scale,
                 gain_range->max  * gain_range->scale,
                 gain_range->step * gain_range->scale);

    LOG_INFO(TAG_BLADE " Configuring TX sync interface... ");
    ret = bladerf_sync_config(blade->dev,
                              BLADERF_TX_X1,
                              BLADERF_FORMAT_SC16_Q11,
                              16,    /* num buffers */
                              8192,  /* buffer size (samples) */
                              8,     /* num transfers */
                              3500); /* timeout ms */
    if (ret < 0) {
        LOG_ERR("FAILED (%s)\n", bladerf_strerror(ret));
        bladerf_close(blade->dev);
        return -1;
    }
    LOG_OK("OK\n");

    bladerf_enable_module(blade->dev, BLADERF_CHANNEL_TX(0), true);

    blade->transmit = blade_transmit;

    return 0;
}

void blade_close(BLADE *blade)
{
    if (!blade) return;
    LOG_INFO(TAG_BLADE " Closing device %d... ", blade->index);

    if (blade->dev) {
        bladerf_enable_module(blade->dev, BLADERF_CHANNEL_TX(0), false);
        bladerf_close(blade->dev);
        blade->dev = NULL;
    }
    if (blade->tone_buf) { free(blade->tone_buf); blade->tone_buf = NULL; }
    LOG_OK("OK\n");
}

int create_test_tone(BLADE *blade, float tone_hz, uint32_t on_ms, uint32_t off_ms)
{
    if (!blade) return -1;

    uint32_t on_samples  = (uint32_t)((on_ms  / 1000.0f) * blade->sample_rate);
    uint32_t off_samples = (uint32_t)((off_ms / 1000.0f) * blade->sample_rate);
    uint32_t total       = on_samples + off_samples;

    /* 2 int16_t per I/Q pair */
    int16_t *buf = (int16_t *)malloc(total * 2 * sizeof(int16_t));
    if (!buf) {
        LOG_ERR(TAG_BLADE " create_test_tone: malloc failed\n");
        return -1;
    }

    /* SC16_Q11: full scale amplitude is 2047 */
    const float amp   = 2047.0f;
    const float twopi = 2.0f * (float)M_PI;

    /* ON: sine wave at tone_hz offset */
    for (uint32_t i = 0; i < on_samples; i++) {
        float t      = (float)i / (float)blade->sample_rate;
        buf[i * 2]     = (int16_t)(amp * cosf(twopi * tone_hz * t));  /* I */
        buf[i * 2 + 1] = (int16_t)(amp * sinf(twopi * tone_hz * t));  /* Q */
    }

    /* OFF: silence */
    for (uint32_t i = on_samples; i < total; i++) {
        buf[i * 2]     = 0;
        buf[i * 2 + 1] = 0;
    }

    if (blade->tone_buf) free(blade->tone_buf);
    blade->tone_buf     = buf;
    blade->tone_samples = total;

    LOG_INFO(TAG_BLADE " Test tone: %.1f Hz offset, %u ms on / %u ms off, %u samples\n",
             tone_hz, on_ms, off_ms, total);

    return 0;
}

int blade_transmit_loop(BLADE *blade, volatile int *stop_flag)
{
    if (!blade || !blade->tone_buf) {
        LOG_ERR(TAG_BLADE " blade_transmit_loop: no tone loaded\n");
        return -1;
    }
    while (!(*stop_flag)) {
        int ret = bladerf_sync_tx(blade->dev, blade->tone_buf, blade->tone_samples, NULL, 3500);
        if (ret < 0) {
            LOG_ERR(TAG_BLADE " transmit_loop: tx error on device %d: %s\n",
                    blade->index, bladerf_strerror(ret));
            return -1;
        }
    }
    return 0;
}

int blade_transmit(BLADE *blade, int16_t *buf, uint32_t n)
{
    if (!blade || !blade->dev || !buf) return -1;

    int ret = bladerf_sync_tx(blade->dev, buf, n, NULL, 3500);
    if (ret < 0) {
        LOG_ERR(TAG_BLADE " transmit: tx error on device %d: %s\n",
                blade->index, bladerf_strerror(ret));
        return -1;
    }

    return 0;
}
