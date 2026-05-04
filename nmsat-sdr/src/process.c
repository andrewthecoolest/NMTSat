#include "process.h"
#include "log.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <errno.h>
#include <time.h>

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

int ring_read_timed(ring_buf_t *r, uint8_t *dst, size_t len, uint32_t timeout_ms)
{
    struct timespec deadline;
    clock_gettime(CLOCK_REALTIME, &deadline);
    deadline.tv_sec  += (time_t)(timeout_ms / 1000);
    deadline.tv_nsec += (long)(timeout_ms % 1000) * 1000000L;
    if (deadline.tv_nsec >= 1000000000L) {
        deadline.tv_sec++;
        deadline.tv_nsec -= 1000000000L;
    }

    pthread_mutex_lock(&r->lock);
    while (r->write_pos - r->read_pos < len) {
        if (pthread_cond_timedwait(&r->not_empty, &r->lock, &deadline) == ETIMEDOUT) {
            pthread_mutex_unlock(&r->lock);
            return -1;
        }
    }
    for (size_t i = 0; i < len; i++) {
        dst[i] = r->data[r->read_pos & (r->size - 1)];
        r->read_pos++;
    }
    pthread_mutex_unlock(&r->lock);
    return 0;
}

/* ------------------------------------------------------------------ */
/* Sliding Window                                                       */
/* ------------------------------------------------------------------ */

int window_init(window_t *w, int n_sdrs)
{
    memset(w, 0, sizeof(window_t));
    w->n_sdrs = n_sdrs;
    int n_pairs = n_sdrs - 1;

    w->mags = calloc((size_t)n_sdrs, sizeof(uint8_t *));
    if (!w->mags) goto fail;
    for (int i = 0; i < n_sdrs; i++) {
        w->mags[i] = calloc(WINDOW_SIZE, sizeof(uint8_t));
        if (!w->mags[i]) goto fail;
    }

    if (n_pairs > 0) {
        w->diffs = calloc((size_t)n_pairs, sizeof(int16_t *));
        if (!w->diffs) goto fail;
        for (int p = 0; p < n_pairs; p++) {
            w->diffs[p] = calloc(WINDOW_SIZE, sizeof(int16_t));
            if (!w->diffs[p]) goto fail;
        }
        w->diff_avg = calloc((size_t)n_pairs, sizeof(int32_t));
        if (!w->diff_avg) goto fail;
        w->desync_streak = calloc((size_t)n_pairs, sizeof(int));
        if (!w->desync_streak) goto fail;
    }

    return 0;
fail:
    window_free(w);
    return -1;
}

void window_free(window_t *w)
{
    if (w->mags) {
        for (int i = 0; i < w->n_sdrs; i++)
            free(w->mags[i]);
        free(w->mags);
        w->mags = NULL;
    }
    int n_pairs = w->n_sdrs - 1;
    if (w->diffs) {
        for (int p = 0; p < n_pairs; p++)
            free(w->diffs[p]);
        free(w->diffs);
        w->diffs = NULL;
    }
    free(w->diff_avg);      w->diff_avg      = NULL;
    free(w->desync_streak); w->desync_streak = NULL;
}

void window_reset(window_t *w)
{
    int n_pairs = w->n_sdrs - 1;
    for (int i = 0; i < w->n_sdrs; i++)
        memset(w->mags[i], 0, WINDOW_SIZE);
    for (int p = 0; p < n_pairs; p++) {
        memset(w->diffs[p], 0, WINDOW_SIZE * sizeof(int16_t));
        w->diff_avg[p]      = 0;
        w->desync_streak[p] = 0;
    }
    w->pos = 0;
}

int window_push(window_t *w, const uint8_t *mags)
{
    uint32_t idx    = w->pos % WINDOW_SIZE;
    int      n_pairs = w->n_sdrs - 1;
    int      event_mask = 0;

    for (int i = 0; i < w->n_sdrs; i++)
        w->mags[i][idx] = mags[i];

    for (int p = 0; p < n_pairs; p++) {
        int16_t current_diff = (int16_t)mags[0] - (int16_t)mags[p + 1];
        w->diffs[p][idx] = current_diff;

        /* Subtract oldest sample from rolling sum, add current. */
        uint32_t old_idx = (idx + WINDOW_SIZE - DIFF_AVG_LEN) % WINDOW_SIZE;
        w->diff_avg[p] += (int32_t)current_diff - (int32_t)w->diffs[p][old_idx];

        if (w->pos >= WARMUP_SAMPLES) {
            int32_t baseline  = w->diff_avg[p] / DIFF_AVG_LEN;
            int32_t delta     = (int32_t)current_diff - baseline;
            if (delta < 0) delta = -delta;
            int32_t floor     = w->min_diff_thresh ? w->min_diff_thresh : MIN_DIFF_THRESH;
            int32_t threshold = (baseline < 0 ? -baseline : baseline) << 3;
            if (threshold < floor) threshold = floor;

            if (delta > threshold) {
                w->desync_streak[p]++;
                if (w->desync_streak[p] >= DESYNC_DEBOUNCE)
                    event_mask |= (1 << p);
            } else {
                w->desync_streak[p] = 0;
            }
        }
    }

    w->pos++;
    return event_mask;
}

/* ------------------------------------------------------------------ */
/* Lag State                                                            */
/* ------------------------------------------------------------------ */

int lag_state_init(lag_state_t *l, int n_sdrs)
{
    memset(l, 0, sizeof(lag_state_t));
    l->n_sdrs = n_sdrs;
    int n_pairs = n_sdrs - 1;
    if (n_pairs > 0) {
        l->calibrated  = calloc((size_t)n_pairs, sizeof(int));
        l->lag_samples = calloc((size_t)n_pairs, sizeof(int32_t));
        if (!l->calibrated || !l->lag_samples) {
            lag_state_free(l);
            return -1;
        }
    }
    return 0;
}

void lag_state_free(lag_state_t *l)
{
    free(l->calibrated);   l->calibrated  = NULL;
    free(l->lag_samples);  l->lag_samples = NULL;
}

int lag_all_calibrated(const lag_state_t *l)
{
    for (int p = 0; p < l->n_sdrs - 1; p++)
        if (!l->calibrated[p]) return 0;
    return 1;
}

/* ------------------------------------------------------------------ */
/* Processing Thread                                                    */
/* ------------------------------------------------------------------ */

void *processing_thread(void *arg)
{
    proc_args_t *a    = (proc_args_t *)arg;
    lag_state_t *lag  = a->lag;
    int          n_sdrs  = a->n_sdrs;
    int          n_pairs = n_sdrs - 1;

    window_t win;
    if (window_init(&win, n_sdrs) < 0) return NULL;
    win.min_diff_thresh = a->min_diff_thresh;
    a->active_window = &win;

    /* 2 bytes (I+Q) per SDR per sample */
    uint8_t *pair = malloc(2 * (size_t)n_sdrs);
    uint8_t *mags = malloc((size_t)n_sdrs);
    if (!pair || !mags) {
        free(pair); free(mags);
        a->active_window = NULL;
        window_free(&win);
        return NULL;
    }

    int      post_collecting = 0;
    uint32_t post_target     = 0;
    char     filename[64];

    a->exit_reason = PROC_EXIT_STOPPED;
    a->dropout_sdr = -1;

    while (!(*a->stop_flag)) {
        int dropped = -1;
        if (a->read_timeout_ms > 0) {
            for (int i = 0; i < n_sdrs; i++) {
                if (ring_read_timed(a->rings[i], &pair[i * 2], 2, a->read_timeout_ms) < 0) {
                    dropped = i;
                    break;
                }
            }
        } else {
            for (int i = 0; i < n_sdrs; i++)
                ring_read(a->rings[i], &pair[i * 2], 2);
        }

        if (dropped >= 0) {
            LOG_ERR("[process] SDR%d stopped delivering samples\n", dropped);
            a->exit_reason = PROC_EXIT_DROPOUT;
            a->dropout_sdr = dropped;
            break;
        }

        for (int i = 0; i < n_sdrs; i++) {
            int iv = (int)pair[i * 2]     - 127;
            int qv = (int)pair[i * 2 + 1] - 127;
            mags[i] = fast_mag(iv, qv);
        }

        int event_mask = window_push(&win, mags);

        /* --- Phase 1: lag calibration (disable with ENABLE_ALIGNMENT 0) --- */
#if ENABLE_ALIGNMENT
        if (!lag_all_calibrated(lag)) {
            if (win.pos >= WARMUP_SAMPLES) {
                LOG_INFO("[process] Running xcorr at sample %u\n", win.pos);

                /* ---- LAG METHOD: cross-correlation ----
                 * Flatten each SDR's magnitude ring into a linear buffer, then
                 * slide sdr_i against sdr_0 over ±XCORR_MAX_LAG offsets.
                 * The offset that maximises the dot product is the lag:
                 *   d > 0 → sdr_i is ahead → advance ring_i
                 *   d < 0 → sdr_0 is ahead → advance ring_0
                 * All rings are then shifted to the slowest common reference. */
                uint8_t *flat    = malloc((size_t)n_sdrs * WINDOW_SIZE);
                int32_t *offsets = calloc((size_t)n_sdrs, sizeof(int32_t));

                if (flat && offsets) {
                    for (int i = 0; i < n_sdrs; i++)
                        for (int j = 0; j < WINDOW_SIZE; j++)
                            flat[i * WINDOW_SIZE + j] =
                                win.mags[i][(win.pos + (uint32_t)j) % WINDOW_SIZE];

                    for (int p = 0; p < n_pairs; p++) {
                        if (lag->calibrated[p]) {
                            offsets[p + 1] = lag->lag_samples[p];
                            continue;
                        }
                        const uint8_t *f0 = &flat[0];
                        const uint8_t *fi = &flat[(size_t)(p + 1) * WINDOW_SIZE];

                        /* Mean-subtract before correlating — removes the DC term so
                         * the peak is driven by actual signal variation, not average level. */
                        int64_t sum0 = 0, sumi = 0;
                        for (int k = 0; k < WINDOW_SIZE; k++) { sum0 += f0[k]; sumi += fi[k]; }
                        int32_t mean0 = (int32_t)(sum0 / WINDOW_SIZE);
                        int32_t meani = (int32_t)(sumi / WINDOW_SIZE);

                        int32_t best_score = INT32_MIN;
                        int     best_lag   = 0;
                        for (int d = -XCORR_MAX_LAG; d <= XCORR_MAX_LAG; d++) {
                            int32_t score = 0;
                            for (int k = 0; k < WINDOW_SIZE; k++) {
                                int j = k + d;
                                if (j < 0 || j >= WINDOW_SIZE) continue;
                                score += ((int32_t)f0[k] - mean0) * ((int32_t)fi[j] - meani);
                            }
                            if (score > best_score) { best_score = score; best_lag = d; }
                        }
                        lag->lag_samples[p] = (int32_t)best_lag;
                        lag->calibrated[p]  = 1;
                        offsets[p + 1]      = (int32_t)best_lag;
                        LOG_INFO("[process] Xcorr sdr0 vs sdr%d: %d samples (sdr%d leads)\n",
                                 p + 1, best_lag, best_lag > 0 ? (p + 1) : 0);
                    }

                    /* Advance each ring by (its offset - min_offset) so all SDRs
                     * align to the slowest receiver without discarding any data. */
                    int32_t min_off = 0;
                    for (int i = 1; i < n_sdrs; i++)
                        if (offsets[i] < min_off) min_off = offsets[i];

                    for (int i = 0; i < n_sdrs; i++) {
                        int32_t advance = offsets[i] - min_off;
                        if (advance > 0) {
                            LOG_INFO("[process] Advancing ring%d by %d samples\n", i, (int)advance);
                            pthread_mutex_lock(&a->rings[i]->lock);
                            a->rings[i]->read_pos += (size_t)advance * 2;
                            pthread_mutex_unlock(&a->rings[i]->lock);
                        }
                    }
                }

                free(flat);
                free(offsets);
                window_reset(&win);
                LOG_INFO("[process] Streams aligned — desync detection active\n");
            }
            continue;
        }
#endif /* ENABLE_ALIGNMENT */

        /* --- Phase 2: desync detection --- */
        if (event_mask && !post_collecting) {
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

            a->exit_reason = PROC_EXIT_DESYNC;
            *a->stop_flag = 1;
        }
    }

    a->active_window = NULL;
    free(pair);
    free(mags);
    window_free(&win);
    return NULL;
}
