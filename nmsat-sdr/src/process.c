#include "process.h"
#include "log.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

/* ------------------------------------------------------------------ */
/* Ring Buffer                                                          */
/* ------------------------------------------------------------------ */

int ring_init(ring_buf_t *r)
{
    r->data = (uint8_t *)malloc(RING_SIZE);
    if (!r->data) {
        LOG_ERR("[process] ring_init: malloc failed\n");
        return -1;
    }
    r->size      = RING_SIZE;
    r->write_pos = 0;
    r->read_pos  = 0;
    pthread_mutex_init(&r->lock, NULL);
    pthread_cond_init(&r->not_empty, NULL);
    return 0;
}

void ring_free(ring_buf_t *r)
{
    if (r->data) { free(r->data); r->data = NULL; }
    pthread_mutex_destroy(&r->lock);
    pthread_cond_destroy(&r->not_empty);
}

void ring_write(ring_buf_t *r, const uint8_t *src, size_t len)
{
    pthread_mutex_lock(&r->lock);
    for (size_t i = 0; i < len; i++) {
        r->data[r->write_pos & (r->size - 1)] = src[i];
        r->write_pos++;
        /* If full, advance read_pos to discard oldest byte */
        if (r->write_pos - r->read_pos > r->size)
            r->read_pos++;
    }
    pthread_cond_signal(&r->not_empty);
    pthread_mutex_unlock(&r->lock);
}

void ring_read(ring_buf_t *r, uint8_t *dst, size_t len)
{
    pthread_mutex_lock(&r->lock);
    while (r->write_pos - r->read_pos < len)
        pthread_cond_wait(&r->not_empty, &r->lock);
    for (size_t i = 0; i < len; i++) {
        dst[i] = r->data[r->read_pos & (r->size - 1)];
        r->read_pos++;
    }
    pthread_mutex_unlock(&r->lock);
}

/* ------------------------------------------------------------------ */
/* Sliding Window                                                       */
/* ------------------------------------------------------------------ */

void window_init(window_t *w)
{
    memset(w, 0, sizeof(window_t));
}

int window_push(window_t *w, uint8_t m0, uint8_t m1)
{
    uint32_t idx = w->pos % WINDOW_SIZE;

    w->mag0[idx] = m0;
    w->mag1[idx] = m1;

    int16_t current_diff = (int16_t)m0 - (int16_t)m1;
    w->diff[idx] = current_diff;

    /* Update rolling sum of last DIFF_AVG_LEN samples.
     * Subtract the sample that's DIFF_AVG_LEN steps behind the current position. */
    uint32_t old_idx = (idx + WINDOW_SIZE - DIFF_AVG_LEN) % WINDOW_SIZE;
    w->diff_avg += (int32_t)current_diff - (int32_t)w->diff[old_idx];

    int desync = 0;
    if (w->pos >= WARMUP_SAMPLES) {
        /* Compare current diff against rolling average of last DIFF_AVG_LEN samples.
         * Require DESYNC_DEBOUNCE consecutive above-threshold samples to fire —
         * prevents false triggers when both signals pass through zero. */
        int32_t baseline  = w->diff_avg / DIFF_AVG_LEN;
        int32_t delta     = (int32_t)current_diff - baseline;
        if (delta < 0) delta = -delta;
        int32_t threshold = (baseline < 0 ? -baseline : baseline) << 3;
        if (threshold < MIN_DIFF_THRESH) threshold = MIN_DIFF_THRESH;

        if (delta > threshold) {
            w->desync_streak++;
            if (w->desync_streak >= DESYNC_DEBOUNCE)
                desync = 1;
        } else {
            w->desync_streak = 0;
        }
    }

    w->last_diff = current_diff;
    w->pos++;
    return desync;
}

/* ------------------------------------------------------------------ */
/* Processing Thread                                                    */
/* ------------------------------------------------------------------ */

void *processing_thread(void *arg)
{
    proc_args_t *a   = (proc_args_t *)arg;
    lag_state_t *lag = a->lag;

    window_t win;
    window_init(&win);

    /* Sample pair: 2 bytes per SDR (I+Q) */
    uint8_t pair0[2], pair1[2];

    int      post_collecting  = 0;
    uint32_t post_target      = 0;

    char filename[64];

    while (!(*a->stop_flag)) {
        ring_read(a->ring0, pair0, 2);
        ring_read(a->ring1, pair1, 2);

        int i0 = (int)pair0[0] - 127;
        int q0 = (int)pair0[1] - 127;
        int i1 = (int)pair1[0] - 127;
        int q1 = (int)pair1[1] - 127;

        uint8_t m0 = fast_mag(i0, q0);
        uint8_t m1 = fast_mag(i1, q1);

        int event = window_push(&win, m0, m1);

        /* --- Phase 1: lag calibration (disable with ENABLE_ALIGNMENT 0) --- */
#if ENABLE_ALIGNMENT
        if (!lag->calibrated) {
            if (event && !post_collecting) {
                post_collecting = 1;
                post_target     = win.pos + (WINDOW_SIZE / 2);
                LOG_INFO("[process] First event at sample %u — collecting context...\n", win.pos);
            }

            if (post_collecting && win.pos >= post_target) {
                post_collecting = 0;

                snprintf(filename, sizeof(filename), "desync_first.bin");
                if (a->on_window_event) a->on_window_event(&win, filename);

                /* ---- LAG METHOD: cross-correlation ----
                 * To revert to rising-edge detection, replace this block with
                 * the commented-out RISING EDGE block below.
                 *
                 * Flatten the ring window into linear buffers, then slide mag1
                 * against mag0 across ±XCORR_MAX_LAG offsets. The offset that
                 * maximises the dot product is the lag: positive = sdr0 leads. */
                uint8_t flat0[WINDOW_SIZE], flat1[WINDOW_SIZE];
                for (int i = 0; i < WINDOW_SIZE; i++) {
                    uint32_t idx = (win.pos + (uint32_t)i) % WINDOW_SIZE;
                    flat0[i] = win.mag0[idx];
                    flat1[i] = win.mag1[idx];
                }

                int32_t best_score = INT32_MIN;
                int     best_lag   = 0;
                for (int d = -XCORR_MAX_LAG; d <= XCORR_MAX_LAG; d++) {
                    int32_t score = 0;
                    for (int i = 0; i < WINDOW_SIZE; i++) {
                        int j = i + d;
                        if (j < 0 || j >= WINDOW_SIZE) continue;
                        score += (int32_t)flat0[i] * (int32_t)flat1[j];
                    }
                    if (score > best_score) {
                        best_score = score;
                        best_lag   = d;
                    }
                }
                /* ---- END cross-correlation ---- */

                /* ---- RISING EDGE METHOD (alternative — swap in to revert) ----
                 * int edge0 = -1, edge1 = -1;
                 * for (int i = 1; i < WINDOW_SIZE; i++) {
                 *     uint32_t idx  = (win.pos + (uint32_t)i)       % WINDOW_SIZE;
                 *     uint32_t prev = (win.pos + (uint32_t)(i - 1)) % WINDOW_SIZE;
                 *     if (edge0 < 0 && (int)win.mag0[idx] - (int)win.mag0[prev] >= RISE_THRESH)
                 *         edge0 = i;
                 *     if (edge1 < 0 && (int)win.mag1[idx] - (int)win.mag1[prev] >= RISE_THRESH)
                 *         edge1 = i;
                 *     if (edge0 >= 0 && edge1 >= 0) break;
                 * }
                 * int best_lag = (edge0 >= 0 && edge1 >= 0) ? (edge0 - edge1) : INT32_MIN;
                 * if (best_lag == (int)INT32_MIN) {
                 *     LOG_ERR("[process] Rising edges not found — retrying\n");
                 *     continue; (goto next iteration)
                 * }
                 * ---- END rising edge ---- */

                {
                    int32_t delta    = (int32_t)best_lag;
                    lag->lag_samples = delta;
                    lag->calibrated  = 1;

                    /* xcorr: score peaks at d where flat0[i] ~ flat1[i+d]
                     * d > 0 → flat1 is ahead → sdr1 leads → advance ring1
                     * d < 0 → flat0 is ahead → sdr0 leads → advance ring0 */
                    LOG_INFO("[process] Xcorr lag: %d samples (sdr%s leads)\n",
                             delta, delta > 0 ? "1" : "0");

                    int32_t abs_lag = delta < 0 ? -delta : delta;
                    LOG_INFO("[process] Applying persistent lag offset: %d samples on sdr%s\n",
                             (int)abs_lag, delta > 0 ? "1" : "0");

                    if (delta > 0) {
                        pthread_mutex_lock(&a->ring1->lock);
                        a->ring1->read_pos += (size_t)abs_lag * 2;
                        pthread_mutex_unlock(&a->ring1->lock);
                    } else if (delta < 0) {
                        pthread_mutex_lock(&a->ring0->lock);
                        a->ring0->read_pos += (size_t)abs_lag * 2;
                        pthread_mutex_unlock(&a->ring0->lock);
                    }

                    window_init(&win);
                    LOG_INFO("[process] Streams aligned — desync detection active\n");
                }
            }
            continue;
        }
#endif /* ENABLE_ALIGNMENT */

        /* --- Phase 2: desync detection --- */
        if (event && !post_collecting) {
            lag->desync_count++;
            post_collecting = 1;
            post_target     = win.pos + (WINDOW_SIZE / 2);

            LOG_ERR("[process] Desync #%u at sample %u — collecting context...\n",
                    lag->desync_count, win.pos);
        }

        if (post_collecting && win.pos >= post_target) {
            post_collecting = 0;

            const char *label = (lag->desync_count == 1) ? "second" :
                                (lag->desync_count == 2) ? "third" : NULL;
            if (label)
                snprintf(filename, sizeof(filename), "desync_%s.bin", label);
            else
                snprintf(filename, sizeof(filename), "desync_%u.bin", lag->desync_count);
            if (a->on_window_event) a->on_window_event(&win, filename);

            *a->stop_flag = 1;
        }
    }

    return NULL;
}
