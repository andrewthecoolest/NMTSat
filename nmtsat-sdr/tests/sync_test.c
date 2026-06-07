#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <pthread.h>
#include <sched.h>
#include <time.h>
#include "log.h"
#include "sdr.h"
#include "blade.h"
#include "process.h"

#define CENTER_FREQ    1000000000UL  /* 1 GHz */
#define SAMPLE_RATE    2048000
#define BLADE_BW       1500000
#define BLADE_GAIN     64
#define INITIAL_GAIN   496           /* 49.6 dB starting gain */

#define MAG_TOL        0.5f
/* 4 full tone cycles (10ms on + 10ms off = 40960 samples/cycle) for a
 * stable average that is independent of where in the cycle we start. */
#define GAIN_SAMPLE    (4 * 40960)
#define MAX_GAIN_ITER  8

#define DUMP_SAMPLES   9000
#define CALIB_TIMEOUT  200           /* iterations × 50ms = 10s max */

/* ------------------------------------------------------------------ */
/* Helpers                                                              */
/* ------------------------------------------------------------------ */

static void set_fifo_max(void)
{
    struct sched_param sp = { .sched_priority = sched_get_priority_max(SCHED_FIFO) };
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);
}

/* ------------------------------------------------------------------ */
/* SDR start thread: waits for RTC trigger then starts stream          */
/* ------------------------------------------------------------------ */

typedef struct {
    SDR             *sdr;
    struct timespec *trigger;
} sdr_start_args_t;

static void *sdr_start_thread(void *arg)
{
    sdr_start_args_t *a = arg;
    set_fifo_max();
    clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, a->trigger, NULL);
    sdr_stream_start(a->sdr);
    return NULL;
}

/* ------------------------------------------------------------------ */
/* TX thread                                                            */
/* ------------------------------------------------------------------ */

typedef struct {
    BLADE           *blade;
    volatile int    *stop_flag;
    struct timespec *trigger;
} tx_args_t;

static void *tx_thread(void *arg)
{
    tx_args_t *a = arg;
    set_fifo_max();
    clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, a->trigger, NULL);
    blade_transmit_loop(a->blade, a->stop_flag);
    return NULL;
}

/* ------------------------------------------------------------------ */
/* Processing thread wrapper                                            */
/* ------------------------------------------------------------------ */

static void *proc_thread_wrapper(void *arg)
{
    set_fifo_max();
    processing_thread(arg);
    return NULL;
}

/* ------------------------------------------------------------------ */
/* Gain adjustment                                                      */
/* ------------------------------------------------------------------ */

/* Discard all but the most recently written samples so we measure
 * at the current gain, not whatever filled the ring before it settled. */
static void ring_flush(ring_buf_t *r)
{
    pthread_mutex_lock(&r->lock);
    if (r->write_pos > r->read_pos)
        r->read_pos = r->write_pos;
    pthread_mutex_unlock(&r->lock);
}

static float sample_avg_mag(ring_buf_t *ring, int n)
{
    uint32_t total = 0;
    for (int k = 0; k < n; k++) {
        uint8_t iq[2];
        ring_read(ring, iq, 2);
        int iv = (int)iq[0] - 127;
        int qv = (int)iq[1] - 127;
        total += fast_mag(iv, qv);
    }
    return (float)total / (float)n;
}

static int nearest_gain_idx(const int *gtab, int n_gains, int target_tenths)
{
    int best = 0;
    for (int g = 1; g < n_gains; g++)
        if (abs(gtab[g] - target_tenths) < abs(gtab[best] - target_tenths))
            best = g;
    return best;
}

static void adjust_gains(SDR *sdrs, ring_buf_t *rings, int n_sdrs)
{
    int n_gains = rtlsdr_get_tuner_gains(sdrs[0].dev, NULL);
    if (n_gains <= 0) { LOG_ERR("[sync_test] Could not query gain table\n"); return; }
    int *gtab = malloc((size_t)n_gains * sizeof(int));
    if (!gtab) return;
    rtlsdr_get_tuner_gains(sdrs[0].dev, gtab);

    /* SDR0 is the reference — keep it at INITIAL_GAIN throughout.
     * SDR 1..N start from the table entry nearest to INITIAL_GAIN. */
    int gidx[n_sdrs];
    for (int s = 0; s < n_sdrs; s++) {
        gidx[s] = nearest_gain_idx(gtab, n_gains, INITIAL_GAIN);
        rtlsdr_set_tuner_gain_mode(sdrs[s].dev, 1);
        rtlsdr_set_tuner_gain(sdrs[s].dev, gtab[gidx[s]]);
    }
    nanosleep(&(struct timespec){0, 100000000}, NULL);
    for (int s = 0; s < n_sdrs; s++) ring_flush(&rings[s]);

    /* Measure SDR0 reference over 4 full tone cycles for a stable average. */
    float ref_mag = sample_avg_mag(&rings[0], GAIN_SAMPLE);
    LOG_INFO("[sync_test] SDR0 reference  gain=%.1f dB  avg_mag=%.2f\n",
             gtab[gidx[0]] / 10.0f, ref_mag);
    if (ref_mag < 0.5f) {
        LOG_ERR("[sync_test] Reference magnitude too low — is the BladeRF TX running?\n");
        free(gtab);
        return;
    }

    /* Converge SDR 1..N to match SDR0's magnitude using a direct dB-law jump
     * rather than one-step-at-a-time iteration. */
    int done[n_sdrs];
    done[0] = 1;
    for (int s = 1; s < n_sdrs; s++) done[s] = 0;

    for (int iter = 0; iter < MAX_GAIN_ITER; iter++) {
        int all_done = 1;
        for (int s = 1; s < n_sdrs; s++) {
            if (done[s]) continue;
            all_done = 0;

            float avg = sample_avg_mag(&rings[s], GAIN_SAMPLE);
            LOG_INFO("[sync_test] SDR%d  gain=%.1f dB  avg_mag=%.2f  ref=%.2f\n",
                     s, gtab[gidx[s]] / 10.0f, avg, ref_mag);

            if (fabsf(avg - ref_mag) <= MAG_TOL) {
                done[s] = 1;
                LOG_OK("[sync_test] SDR%d converged at %.1f dB (err=%.2f)\n",
                       s, gtab[gidx[s]] / 10.0f, avg - ref_mag);
                continue;
            }

            /* dB-law direct jump: ΔdB = 20·log10(target/current).
             * Gain table is in tenths of dB, so multiply by 10. */
            float delta_db      = 20.0f * log10f(ref_mag / avg);
            int   target_tenths = gtab[gidx[s]] + (int)(delta_db * 10.0f + 0.5f);
            int   new_idx       = nearest_gain_idx(gtab, n_gains, target_tenths);

            /* If the formula lands on the same entry, nudge one step to break out. */
            if (new_idx == gidx[s]) {
                if (avg < ref_mag && gidx[s] < n_gains - 1) new_idx++;
                else if (avg > ref_mag && gidx[s] > 0)      new_idx--;
            }

            gidx[s] = new_idx;
            rtlsdr_set_tuner_gain(sdrs[s].dev, gtab[gidx[s]]);
            nanosleep(&(struct timespec){0, 50000000}, NULL);
            ring_flush(&rings[s]);
        }
        if (all_done) break;
    }

    free(gtab);
}

/* ------------------------------------------------------------------ */
/* Retune                                                               */
/* ------------------------------------------------------------------ */

/* Tune SDRs 1..n_sdrs-1 away then back to confirm sync loss. SDR 0 is
 * left untouched so the reference stream stays uninterrupted. */
static void retune_sdrs(SDR *sdrs, ring_buf_t *rings, int n_sdrs)
{
#define RETUNE_FREQ  100000000UL  /* 100 MHz detour */
    LOG_INFO("[sync_test] Retuning SDR 1..%d → 100 MHz\n", n_sdrs - 1);
    for (int s = 1; s < n_sdrs; s++)
        rtlsdr_set_center_freq(sdrs[s].dev, RETUNE_FREQ);

    nanosleep(&(struct timespec){0, 200000000}, NULL); /* 200 ms away */

    LOG_INFO("[sync_test] Retuning SDR 1..%d → back to %lu Hz\n",
             n_sdrs - 1, (unsigned long)CENTER_FREQ);
    for (int s = 1; s < n_sdrs; s++)
        rtlsdr_set_center_freq(sdrs[s].dev, CENTER_FREQ);

    /* Settle and flush so dump reads samples at the restored frequency. */
    nanosleep(&(struct timespec){0, 100000000}, NULL);
    for (int s = 1; s < n_sdrs; s++) ring_flush(&rings[s]);
}

/* ------------------------------------------------------------------ */
/* Sample dump                                                          */
/* ------------------------------------------------------------------ */

static void dump_samples(ring_buf_t *rings, int n_sdrs, const char *suffix)
{
    for (int s = 0; s < n_sdrs; s++) {
        char fname[64];
        snprintf(fname, sizeof(fname), "sync_sdr%d_%s.bin", s, suffix);
        FILE *f = fopen(fname, "wb");
        if (!f) {
            LOG_ERR("[sync_test] Failed to open %s\n", fname);
            continue;
        }
        for (int k = 0; k < DUMP_SAMPLES; k++) {
            uint8_t iq[2];
            ring_read(&rings[s], iq, 2);
            fwrite(iq, 1, 2, f);
        }
        fclose(f);
        LOG_INFO("[sync_test] Dumped %d samples → %s\n", DUMP_SAMPLES, fname);
    }
}

/* ------------------------------------------------------------------ */
/* Placeholder                                                          */
/* ------------------------------------------------------------------ */

static void good(void)
{
    LOG_OK("[sync_test] good\n");
}

/* ------------------------------------------------------------------ */
/* Main                                                                 */
/* ------------------------------------------------------------------ */

int main(void)
{
    /* ---- RTL-SDR init ----------------------------------------------- */
    LOG_HEADER("\n----- Sync Test: RTL-SDR Init -----");

    int n_sdrs = sdr_count();
    LOG_INFO(TAG_RTL " Found %d device(s)\n", n_sdrs);
    if (n_sdrs < 2) {
        LOG_ERR(TAG_RTL " Need at least two devices\n");
        return 1;
    }

    SDR        sdrs[n_sdrs];
    ring_buf_t rings[n_sdrs];
    int        n_opened = 0;

    for (int i = 0; i < n_sdrs; i++) {
        char fname[32];
        snprintf(fname, sizeof(fname), "sync_sdr%d_raw.bin", i);
        if (sdr_open(&sdrs[i], i, CENTER_FREQ, SAMPLE_RATE, INITIAL_GAIN, fname) < 0)
            goto sdr_fail;
        if (ring_init(&rings[i]) < 0) {
            sdr_close(&sdrs[i]);
            goto sdr_fail;
        }
        sdrs[i].ring = &rings[i];
        n_opened++;
    }
    goto sdr_ok;

sdr_fail:
    LOG_ERR("[sync_test] Failed to open SDR devices\n");
    for (int i = 0; i < n_opened; i++) { ring_free(&rings[i]); sdr_close(&sdrs[i]); }
    return 1;

sdr_ok:

    /* ---- BladeRF init ----------------------------------------------- */
    LOG_HEADER("\n----- Sync Test: BladeRF Init -----");

    if (blade_count() < 1) {
        LOG_ERR(TAG_BLADE " No bladeRF found\n");
        for (int i = 0; i < n_sdrs; i++) { ring_free(&rings[i]); sdr_close(&sdrs[i]); }
        return 1;
    }

    BLADE blade;
    if (blade_open(&blade, 0, CENTER_FREQ, SAMPLE_RATE, BLADE_BW, BLADE_GAIN) < 0) {
        for (int i = 0; i < n_sdrs; i++) { ring_free(&rings[i]); sdr_close(&sdrs[i]); }
        return 1;
    }

    /* 10ms on / 10ms off; 10 kHz offset avoids RTL-SDR DC bias at baseband 0 */
    if (create_test_tone(&blade, 10000.0f, 1, 1) < 0) {
        blade_close(&blade);
        for (int i = 0; i < n_sdrs; i++) { ring_free(&rings[i]); sdr_close(&sdrs[i]); }
        return 1;
    }

    /* ---- Schedule simultaneous RX + TX start ------------------------- */
    LOG_HEADER("\n----- Sync Test: Starting -----");

    struct timespec trigger;
    clock_gettime(CLOCK_REALTIME, &trigger);
    trigger.tv_nsec += 200000000L;
    if (trigger.tv_nsec >= 1000000000L) {
        trigger.tv_sec++;
        trigger.tv_nsec -= 1000000000L;
    }

    LOG_INFO("[sync_test] Scheduling RX + TX in 200ms...\n");

    /* High-priority wrapper threads all wake at the same RTC instant. */
    pthread_t        t_sdr[n_sdrs];
    sdr_start_args_t sdr_args[n_sdrs];
    for (int i = 0; i < n_sdrs; i++) {
        sdr_args[i] = (sdr_start_args_t){ .sdr = &sdrs[i], .trigger = &trigger };
        pthread_create(&t_sdr[i], NULL, sdr_start_thread, &sdr_args[i]);
    }

    volatile int tx_stop = 0;
    tx_args_t    ta      = { .blade = &blade, .stop_flag = &tx_stop, .trigger = &trigger };
    pthread_t    t_tx;
    pthread_create(&t_tx, NULL, tx_thread, &ta);

    /* Join start-trigger threads (they exit once sdr_stream_start returns). */
    for (int i = 0; i < n_sdrs; i++)
        pthread_join(t_sdr[i], NULL);

    /* ---- Phase 1: xcorr lag calibration ----------------------------- */
    LOG_HEADER("\n----- Sync Test: Calibrating -----");

    lag_state_t lag;
    if (lag_state_init(&lag, n_sdrs) < 0) {
        tx_stop = 1;
        pthread_join(t_tx, NULL);
        blade_close(&blade);
        for (int i = 0; i < n_sdrs; i++) { sdr_stream_stop(&sdrs[i]); ring_free(&rings[i]); sdr_close(&sdrs[i]); }
        return 1;
    }

    ring_buf_t *ring_ptrs[n_sdrs];
    for (int i = 0; i < n_sdrs; i++) ring_ptrs[i] = &rings[i];

    volatile int  proc_stop = 0;
    proc_args_t   pa = {
        .n_sdrs          = n_sdrs,
        .rings           = ring_ptrs,
        .lag             = &lag,
        .stop_flag       = &proc_stop,
        .on_window_event = NULL,
        .min_diff_thresh = 10,  /* signal mag ~4-10, so transition diff ~4-8 */
    };
    pthread_t t_proc;
    pthread_create(&t_proc, NULL, proc_thread_wrapper, &pa);

    /* Wait for all pairs to be xcorr-calibrated (30s timeout). */
    for (int i = 0; i < CALIB_TIMEOUT && !lag_all_calibrated(&lag); i++)
        nanosleep(&(struct timespec){0, 50000000}, NULL);

    proc_stop = 1;
    pthread_join(t_proc, NULL);

    if (!lag_all_calibrated(&lag)) {
        LOG_ERR("[sync_test] Calibration timed out — dumping samples for analysis\n");
        dump_samples(rings, n_sdrs, "timeout");
        tx_stop = 1;
        pthread_join(t_tx, NULL);
        lag_state_free(&lag);
        blade_close(&blade);
        for (int i = 0; i < n_sdrs; i++) { sdr_stream_stop(&sdrs[i]); ring_free(&rings[i]); sdr_close(&sdrs[i]); }
        return 1;
    }

    LOG_OK("[sync_test] Ring heads aligned — %d pairs calibrated\n", n_sdrs - 1);

    /* ---- Dump 1: immediately post-alignment, pre-gain ----------------- */
    LOG_HEADER("\n----- Sync Test: Dumping Aligned Samples -----");
    dump_samples(rings, n_sdrs, "aligned");

    /* ---- Phase 2: gain adjustment ----------------------------------- */
    LOG_HEADER("\n----- Sync Test: Adjusting Gains -----");
    adjust_gains(sdrs, rings, n_sdrs);

    /* ---- Dump 2: post-gain adjustment --------------------------------- */
    LOG_HEADER("\n----- Sync Test: Dumping Post-Gain Samples -----");
    dump_samples(rings, n_sdrs, "gained");

    /* ---- Phase 3: re-sync after gain disruption ----------------------- */
    LOG_HEADER("\n----- Sync Test: Re-Calibrating -----");
    for (int p = 0; p < n_sdrs - 1; p++) lag.calibrated[p] = 0;
    volatile int proc_stop2 = 0;
    pa.stop_flag = &proc_stop2;
    pthread_t t_proc2;
    pthread_create(&t_proc2, NULL, proc_thread_wrapper, &pa);
    for (int i = 0; i < CALIB_TIMEOUT && !lag_all_calibrated(&lag); i++)
        nanosleep(&(struct timespec){0, 50000000}, NULL);
    proc_stop2 = 1;
    pthread_join(t_proc2, NULL);
    if (lag_all_calibrated(&lag))
        LOG_OK("[sync_test] Re-sync complete\n");
    else
        LOG_ERR("[sync_test] Re-sync timed out\n");

    /* ---- Dump 3: post-resync, pre-retune ------------------------------ */
    LOG_HEADER("\n----- Sync Test: Dumping Re-Synced Samples -----");
    dump_samples(rings, n_sdrs, "resynced");

    /* ---- Phase 4: retune SDR 1..N away and back ----------------------- */
    LOG_HEADER("\n----- Sync Test: Retuning -----");
    retune_sdrs(sdrs, rings, n_sdrs);

    /* ---- Dump 3: post-retune ----------------------------------------- */
    LOG_HEADER("\n----- Sync Test: Dumping Post-Retune Samples -----");
    dump_samples(rings, n_sdrs, "retune");

    /* ---- Placeholder ------------------------------------------------ */
    good();

    /* ---- Teardown --------------------------------------------------- */
    LOG_HEADER("\n----- Sync Test: Done -----");

    tx_stop = 1;
    pthread_join(t_tx, NULL);
    for (int i = 0; i < n_sdrs; i++)
        sdr_stream_stop(&sdrs[i]);

    lag_state_free(&lag);
    blade_close(&blade);
    for (int i = 0; i < n_sdrs; i++) {
        ring_free(&rings[i]);
        sdr_close(&sdrs[i]);
    }

    return 0;
}
