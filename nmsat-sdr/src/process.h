#ifndef PROCESS_H
#define PROCESS_H

#include <stdint.h>
#include <stddef.h>
#include <pthread.h>

/* ------------------------------------------------------------------ */
/* Ring Buffer                                                          */
/* ------------------------------------------------------------------ */

#define RING_SIZE (4 * 1024 * 1024)  /* 4 MB — power of 2 */

typedef struct {
    uint8_t         *data;
    size_t           size;       /* always RING_SIZE */
    size_t           write_pos;
    size_t           read_pos;
    pthread_mutex_t  lock;
    pthread_cond_t   not_empty;
} ring_buf_t;

int  ring_init(ring_buf_t *r);
void ring_free(ring_buf_t *r);

/* Write len bytes into ring. Blocks producer if full (overwrites oldest). */
void ring_write(ring_buf_t *r, const uint8_t *src, size_t len);

/* Read exactly len bytes out of ring. Blocks until data is available. */
void ring_read(ring_buf_t *r, uint8_t *dst, size_t len);

/* ------------------------------------------------------------------ */
/* Fast Magnitude (Alpha-Max Beta-Min, ~5% max error)                 */
/* ------------------------------------------------------------------ */

static inline uint8_t fast_mag(int i, int q)
{
    int ai = i < 0 ? -i : i;
    int aq = q < 0 ? -q : q;
    int mx = ai > aq ? ai : aq;
    int mn = ai < aq ? ai : aq;
    return (uint8_t)(mx + (mn >> 2) + (mn >> 3));
}

/* ------------------------------------------------------------------ */
/* Sliding Window                                                       */
/* ------------------------------------------------------------------ */

/* Set to 1 to enable lag calibration and stream alignment, 0 to skip straight to desync detection */
#define ENABLE_ALIGNMENT 1

#define WINDOW_SIZE      1000
#define WARMUP_SAMPLES   2000   /* samples to collect before any detection */
#define MIN_DIFF_THRESH  70      /* minimum delta to prevent zero-threshold firing */
#define MIN_LAG_WINDOW   20     /* minimum samples between init event and lag calibration */
#define RISE_THRESH      20    /* minimum per-sample magnitude increase to count as a rising edge */
                               /* NOTE: only used by rising-edge lag method (see process.c Phase 1) */
#define XCORR_MAX_LAG    200   /* cross-correlation search range in samples (±) */
#define DIFF_AVG_LEN     32    /* rolling average length for desync baseline (power of 2) */
#define DESYNC_DEBOUNCE  2    /* consecutive samples above threshold required to declare desync */

typedef struct {
    uint8_t  mag0[WINDOW_SIZE];
    uint8_t  mag1[WINDOW_SIZE];
    int16_t  diff[WINDOW_SIZE];
    uint32_t pos;
    int16_t  last_diff;        /* previous single sample diff (kept for compat) */
    int32_t  diff_avg;         /* running sum of last DIFF_AVG_LEN diffs */
    int      desync_streak;    /* consecutive samples above threshold (debounce counter) */
} window_t;

void window_init(window_t *w);

/* Push one I/Q magnitude pair into the window.
 * Returns 1 if a desync is detected, 0 otherwise. */
int window_push(window_t *w, uint8_t m0, uint8_t m1);

/* ------------------------------------------------------------------ */
/* Lag State                                                            */
/* ------------------------------------------------------------------ */

typedef struct {
    int      calibrated;     /* 0 = awaiting first desync, 1 = lag applied */
    int32_t  lag_samples;    /* positive = sdr0 leads sdr1 */
    uint32_t desync_count;
} lag_state_t;

/* ------------------------------------------------------------------ */
/* Processing Thread                                                    */
/* ------------------------------------------------------------------ */

typedef struct {
    ring_buf_t  *ring0;
    ring_buf_t  *ring1;
    lag_state_t *lag;
    volatile int *stop_flag;   /* set to 1 to stop TX and processing */
    void (*on_window_event)(const window_t *w, const char *filename);
} proc_args_t;

void *processing_thread(void *arg);

#endif /* PROCESS_H */
