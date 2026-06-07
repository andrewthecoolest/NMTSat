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

/* Write len bytes into ring. Overwrites oldest data if full. */
void ring_write(ring_buf_t *r, const uint8_t *src, size_t len);

/* Read exactly len bytes. Blocks until data is available. */
void ring_read(ring_buf_t *r, uint8_t *dst, size_t len);

/* Read exactly len bytes with a timeout. Returns 0 on success, -1 on timeout. */
int ring_read_timed(ring_buf_t *r, uint8_t *dst, size_t len, uint32_t timeout_ms);

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

/* Set to 1 to enable lag calibration and stream alignment, 0 to skip */
#define ENABLE_ALIGNMENT 1

#define WINDOW_SIZE      65536
#define WARMUP_SAMPLES   65536  /* wait for a full window of signal before xcorr */
#define MIN_DIFF_THRESH  70     /* minimum delta to prevent zero-threshold firing */
#define MIN_LAG_WINDOW   20     /* minimum samples between init event and lag calibration */
#define RISE_THRESH      20     /* minimum per-sample magnitude increase for rising-edge method */
#define XCORR_MAX_LAG    2048   /* cross-correlation search range in samples (±1ms at 2.048MSPS) */
#define DIFF_AVG_LEN     32     /* rolling average length for desync baseline (power of 2) */
#define DESYNC_DEBOUNCE  2      /* consecutive above-threshold samples to declare desync */

/* processing_thread exit reasons (written to proc_args_t.exit_reason) */
#define PROC_EXIT_STOPPED  0    /* stop_flag set externally */
#define PROC_EXIT_DESYNC   1    /* magnitude desync detected */
#define PROC_EXIT_DROPOUT  2    /* ring_read_timed returned -1 */

typedef struct {
    int       n_sdrs;
    uint8_t **mags;          /* [n_sdrs][WINDOW_SIZE] magnitude histories */
    int16_t **diffs;         /* [n_sdrs-1][WINDOW_SIZE] diff vs SDR 0, one per pair */
    int32_t  *diff_avg;      /* [n_sdrs-1] rolling sums */
    int      *desync_streak; /* [n_sdrs-1] debounce counters */
    uint32_t  pos;
    int32_t   min_diff_thresh; /* runtime override; set to 0 to use MIN_DIFF_THRESH */
} window_t;

/* Allocates internal arrays for n_sdrs receivers. Call window_free() when done. */
int  window_init(window_t *w, int n_sdrs);
void window_free(window_t *w);

/* Zero all state without reallocating (used to restart after lag calibration). */
void window_reset(window_t *w);

/* Push one magnitude per SDR (array of n_sdrs values).
 * Returns bitmask: bit p set if pair (sdr0, sdr_{p+1}) triggered a desync. */
int window_push(window_t *w, const uint8_t *mags);

/* ------------------------------------------------------------------ */
/* Lag State                                                            */
/* ------------------------------------------------------------------ */

typedef struct {
    int       n_sdrs;
    int      *calibrated;    /* [n_sdrs-1]: 1 once xcorr lag applied for that pair */
    int32_t  *lag_samples;   /* [n_sdrs-1]: positive = sdr_{p+1} leads sdr0 */
    uint32_t  desync_count;
} lag_state_t;

int  lag_state_init(lag_state_t *l, int n_sdrs);
void lag_state_free(lag_state_t *l);

/* Returns 1 if all n_sdrs-1 pairs are calibrated. */
int  lag_all_calibrated(const lag_state_t *l);

/* ------------------------------------------------------------------ */
/* Processing Thread                                                    */
/* ------------------------------------------------------------------ */

typedef struct {
    int           n_sdrs;
    ring_buf_t  **rings;
    lag_state_t  *lag;
    volatile int *stop_flag;
    void (*on_window_event)(const window_t *w, const char *filename);
    int32_t       min_diff_thresh;  /* override MIN_DIFF_THRESH; 0 = use compile-time default */
    uint32_t      read_timeout_ms;  /* 0 = blocking (default); >0 enables dropout detection */
    volatile int  exit_reason;      /* PROC_EXIT_* written by thread before returning */
    volatile int  dropout_sdr;      /* index of timed-out SDR; -1 if not a dropout */
    window_t * volatile active_window; /* set by thread once init succeeds; NULL otherwise */
} proc_args_t;

void *processing_thread(void *arg);

#endif /* PROCESS_H */
