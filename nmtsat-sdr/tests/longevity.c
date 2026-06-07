#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <pthread.h>
#include <sched.h>
#include <time.h>
#include <signal.h>
#include <unistd.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <errno.h>
#include "log.h"
#include "sdr.h"
#include "blade.h"
#include "process.h"

/* ------------------------------------------------------------------ */
/* Config                                                               */
/* ------------------------------------------------------------------ */

#define CENTER_FREQ          1000000000UL
#define SAMPLE_RATE          2048000
#define BLADE_BW             1500000
#define BLADE_GAIN           64
#define INITIAL_GAIN         496

#define MAG_TOL              0.5f
#define GAIN_SAMPLE          (4 * 40960)
#define MAX_GAIN_ITER        8
#define DUMP_SAMPLES         9000
#define CALIB_TIMEOUT        200          /* iterations × 50ms = 10s max */

#define HEARTBEAT_INTERVAL_S 1
#define FORCE_DESYNC_SAMPLES 2048
#define SNAPSHOT_LOOK_BACK   4096   /* ~2 ms of recent samples for health check */

/* ------------------------------------------------------------------ */
/* Globals                                                              */
/* ------------------------------------------------------------------ */

static char g_run_dir[256];
static volatile sig_atomic_t g_sigusr1 = 0;
static volatile int g_last_desync_forced = 0;
static volatile int g_quit = 0;
static volatile int *g_monitor_stop_ptr = NULL;
static proc_args_t  *g_pa = NULL;
static uint32_t      g_snapshot_count = 0;

static struct {
    lag_state_t  *lag;
    ring_buf_t  **rings;
    int           n_sdrs;
    time_t        last_desync_time;
    time_t        run_start_time;
} g_ctx;

/* ------------------------------------------------------------------ */
/* Signal handler                                                       */
/* ------------------------------------------------------------------ */

static void sigusr1_handler(int sig)
{
    (void)sig;
    g_sigusr1 = 1;
}

/* ------------------------------------------------------------------ */
/* Shared helpers (mirror sync_test.c)                                  */
/* ------------------------------------------------------------------ */

static void set_fifo_max(void)
{
    struct sched_param sp = { .sched_priority = sched_get_priority_max(SCHED_FIFO) };
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);
}

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

static void *proc_thread_wrapper(void *arg)
{
    set_fifo_max();
    processing_thread(arg);
    return NULL;
}

/* ------------------------------------------------------------------ */
/* Gain helpers                                                         */
/* ------------------------------------------------------------------ */

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
    if (n_gains <= 0) { LOG_ERR("[longevity] Could not query gain table\n"); return; }
    int *gtab = malloc((size_t)n_gains * sizeof(int));
    if (!gtab) return;
    rtlsdr_get_tuner_gains(sdrs[0].dev, gtab);

    int gidx[n_sdrs];
    for (int s = 0; s < n_sdrs; s++) {
        gidx[s] = nearest_gain_idx(gtab, n_gains, INITIAL_GAIN);
        rtlsdr_set_tuner_gain_mode(sdrs[s].dev, 1);
        rtlsdr_set_tuner_gain(sdrs[s].dev, gtab[gidx[s]]);
    }
    nanosleep(&(struct timespec){0, 100000000}, NULL);
    for (int s = 0; s < n_sdrs; s++) ring_flush(&rings[s]);

    float ref_mag = sample_avg_mag(&rings[0], GAIN_SAMPLE);
    LOG_INFO("[longevity] SDR0 reference  gain=%.1f dB  avg_mag=%.2f\n",
             gtab[gidx[0]] / 10.0f, ref_mag);
    if (ref_mag < 0.5f) {
        LOG_ERR("[longevity] Reference magnitude too low — is BladeRF TX running?\n");
        free(gtab);
        return;
    }

    int done[n_sdrs];
    done[0] = 1;
    for (int s = 1; s < n_sdrs; s++) done[s] = 0;

    for (int iter = 0; iter < MAX_GAIN_ITER; iter++) {
        int all_done = 1;
        for (int s = 1; s < n_sdrs; s++) {
            if (done[s]) continue;
            all_done = 0;

            float avg = sample_avg_mag(&rings[s], GAIN_SAMPLE);
            LOG_INFO("[longevity] SDR%d  gain=%.1f dB  avg_mag=%.2f  ref=%.2f\n",
                     s, gtab[gidx[s]] / 10.0f, avg, ref_mag);

            if (fabsf(avg - ref_mag) <= MAG_TOL) {
                done[s] = 1;
                LOG_OK("[longevity] SDR%d converged at %.1f dB (err=%.2f)\n",
                       s, gtab[gidx[s]] / 10.0f, avg - ref_mag);
                continue;
            }

            float delta_db      = 20.0f * log10f(ref_mag / avg);
            int   target_tenths = gtab[gidx[s]] + (int)(delta_db * 10.0f + 0.5f);
            int   new_idx       = nearest_gain_idx(gtab, n_gains, target_tenths);

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
/* Sample dump                                                          */
/* ------------------------------------------------------------------ */

static void dump_samples(ring_buf_t *rings, int n_sdrs, const char *dir, const char *suffix)
{
    for (int s = 0; s < n_sdrs; s++) {
        char fname[512];
        snprintf(fname, sizeof(fname), "%s/sdr%d_%s.bin", dir, s, suffix);
        FILE *f = fopen(fname, "wb");
        if (!f) {
            LOG_ERR("[longevity] Failed to open %s\n", fname);
            continue;
        }
        for (int k = 0; k < DUMP_SAMPLES; k++) {
            uint8_t iq[2];
            ring_read(&rings[s], iq, 2);
            fwrite(iq, 1, 2, f);
        }
        fclose(f);
        LOG_INFO("[longevity] Dumped %d samples → %s\n", DUMP_SAMPLES, fname);
    }
}

/* ------------------------------------------------------------------ */
/* Window dump (forensic magnitude context)                             */
/* ------------------------------------------------------------------ */

static void window_dump(const window_t *w, const char *path)
{
    FILE *f = fopen(path, "wb");
    if (!f) {
        LOG_ERR("[longevity] window_dump: failed to open '%s'\n", path);
        return;
    }
    int n_pairs = w->n_sdrs - 1;
    for (int i = 0; i < WINDOW_SIZE; i++) {
        uint32_t idx = (w->pos + (uint32_t)i) % WINDOW_SIZE;
        for (int s = 0; s < w->n_sdrs; s++)
            fwrite(&w->mags[s][idx], 1, 1, f);
        for (int p = 0; p < n_pairs; p++)
            fwrite(&w->diffs[p][idx], sizeof(int16_t), 1, f);
    }
    fclose(f);
    LOG_INFO("[longevity] Window context → %s\n", path);
}

/* ------------------------------------------------------------------ */
/* Run directory setup                                                  */
/* ------------------------------------------------------------------ */

static int setup_run_dir(void)
{
    time_t     now     = time(NULL);
    struct tm *tm_info = localtime(&now);
    char       ts[32];
    strftime(ts, sizeof(ts), "%Y%m%d_%H%M%S", tm_info);

    if (mkdir("longevity_runs", 0755) < 0 && errno != EEXIST) {
        LOG_ERR("[longevity] Failed to create longevity_runs/\n");
        return -1;
    }
    snprintf(g_run_dir, sizeof(g_run_dir), "longevity_runs/run_%s", ts);
    if (mkdir(g_run_dir, 0755) < 0) {
        LOG_ERR("[longevity] Failed to create %s\n", g_run_dir);
        return -1;
    }

    char path[512];
    snprintf(path, sizeof(path), "%s/summary.csv", g_run_dir);
    FILE *f = fopen(path, "w");
    if (!f) { LOG_ERR("[longevity] Failed to create summary.csv\n"); return -1; }
    fprintf(f, "index,timestamp_iso,uptime_s,inter_arrival_s,forced\n");
    fclose(f);

    LOG_OK("[longevity] Run directory: %s\n", g_run_dir);
    return 0;
}

/* ------------------------------------------------------------------ */
/* Desync event handler                                                 */
/* ------------------------------------------------------------------ */

static void on_desync(const window_t *w, const char *suggested_filename)
{
    (void)suggested_filename;

    uint32_t idx    = g_ctx.lag->desync_count;
    time_t   now    = time(NULL);
    int      forced = g_last_desync_forced;
    g_last_desync_forced = 0;

    char ts[32];
    struct tm *tm_info = localtime(&now);
    strftime(ts, sizeof(ts), "%Y-%m-%dT%H:%M:%S", tm_info);
    long   uptime = (long)(now - g_ctx.run_start_time);
    double inter  = (g_ctx.last_desync_time > 0)
                    ? difftime(now, g_ctx.last_desync_time) : -1.0;

    /* Create per-event subdirectory */
    char subdir[512];
    snprintf(subdir, sizeof(subdir), "%s/desync_%03u", g_run_dir, idx);
    mkdir(subdir, 0755);

    /* Magnitude window context */
    char path[sizeof(subdir) + 32];
    snprintf(path, sizeof(path), "%s/window.bin", subdir);
    window_dump(w, path);

    /* Event log */
    snprintf(path, sizeof(path), "%s/event.log", subdir);
    FILE *f = fopen(path, "w");
    if (f) {
        fprintf(f, "index=%u\ntimestamp=%s\nuptime_s=%ld\ninter_arrival_s=%.1f\nforced=%d\n",
                idx, ts, uptime, inter, forced);
        for (int p = 0; p < g_ctx.n_sdrs - 1; p++)
            fprintf(f, "lag_pair%d=%d\n", p, g_ctx.lag->lag_samples[p]);
        fclose(f);
    }

    /* Append row to summary */
    snprintf(path, sizeof(path), "%s/summary.csv", g_run_dir);
    f = fopen(path, "a");
    if (f) {
        fprintf(f, "%u,%s,%ld,%.1f,%d\n", idx, ts, uptime, inter, forced);
        fclose(f);
    }

    g_ctx.last_desync_time = now;

    LOG_ERR("[longevity] Desync #%u at %s (uptime %lds, inter-arrival %.1fs)%s\n",
            idx, ts, uptime, inter, forced ? " [FORCED]" : "");
}

/* ------------------------------------------------------------------ */
/* Heartbeat thread                                                     */
/* ------------------------------------------------------------------ */

typedef struct {
    volatile int *stop_flag;
    time_t        start_time;
} heartbeat_args_t;

static void *heartbeat_thread(void *arg)
{
    heartbeat_args_t *a = arg;
    char path[512];
    snprintf(path, sizeof(path), "%s/heartbeat.log", g_run_dir);

    while (!(*a->stop_flag)) {
        time_t now    = time(NULL);
        long   uptime = (long)(now - a->start_time);

        FILE *f = fopen(path, "w");
        if (f) {
            char ts[32];
            struct tm *tm_info = localtime(&now);
            strftime(ts, sizeof(ts), "%Y-%m-%dT%H:%M:%S", tm_info);
            fprintf(f, "time=%s\nuptime_s=%ld\ndesync_count=%u\n",
                    ts, uptime, g_ctx.lag->desync_count);
            fclose(f);
        }
        sleep(HEARTBEAT_INTERVAL_S);
    }
    return NULL;
}

/* ------------------------------------------------------------------ */
/* Snapshot                                                             */
/* ------------------------------------------------------------------ */

static void take_snapshot(void)
{
    if (!g_pa) {
        LOG_INFO("[longevity] Not in monitoring phase\n");
        return;
    }

    const window_t *w = g_pa->active_window;
    if (!w || w->pos < WARMUP_SAMPLES) {
        LOG_INFO("[longevity] Snapshot unavailable (calibrating)\n");
        return;
    }

    int      n_sdrs      = w->n_sdrs;
    int      n_pairs     = n_sdrs - 1;
    uint32_t pos         = w->pos;
    int32_t  floor_thresh = g_pa->min_diff_thresh ? g_pa->min_diff_thresh : MIN_DIFF_THRESH;

    time_t now    = time(NULL);
    long   uptime = (long)(now - g_ctx.run_start_time);
    char   ts[32];
    strftime(ts, sizeof(ts), "%Y-%m-%dT%H:%M:%S", localtime(&now));

    /* Per-SDR: average magnitude over last SNAPSHOT_LOOK_BACK samples */
    float avg_mag[n_sdrs];
    for (int s = 0; s < n_sdrs; s++) {
        uint32_t total = 0;
        for (int k = 0; k < SNAPSHOT_LOOK_BACK; k++)
            total += w->mags[s][(pos - 1 - (uint32_t)k) & (WINDOW_SIZE - 1)];
        avg_mag[s] = (float)total / SNAPSHOT_LOOK_BACK;
    }

    /* Per-pair: current baseline, RMS diff, and headroom to threshold */
    float baseline[n_pairs], diff_rms[n_pairs], headroom[n_pairs];
    for (int p = 0; p < n_pairs; p++) {
        baseline[p] = (float)w->diff_avg[p] / DIFF_AVG_LEN;

        int64_t sum_sq = 0;
        for (int k = 0; k < SNAPSHOT_LOOK_BACK; k++) {
            int32_t d = w->diffs[p][(pos - 1 - (uint32_t)k) & (WINDOW_SIZE - 1)];
            sum_sq += (int64_t)d * d;
        }
        diff_rms[p] = sqrtf((float)sum_sq / SNAPSHOT_LOOK_BACK);

        float thresh = fmaxf(fabsf(baseline[p]) * 8.0f, (float)floor_thresh);
        headroom[p]  = 1.0f - diff_rms[p] / thresh;
        if (headroom[p] < 0.0f) headroom[p] = 0.0f;
    }

    /* Overall health: degrade on active streak or headroom below 25% */
    int status = 0;  /* 0=healthy, 1=near threshold, 2=streak active */
    for (int p = 0; p < n_pairs; p++) {
        if (w->desync_streak[p] > 0 && status < 2) status = 2;
        else if (headroom[p] < 0.25f && status < 1) status = 1;
    }

    g_snapshot_count++;
    LOG_INFO("\n[snapshot #%u]  %s  uptime %lds  desync_count=%u\n",
             g_snapshot_count, ts, uptime, g_ctx.lag->desync_count);
    for (int s = 0; s < n_sdrs; s++)
        LOG_INFO("  sdr%-2d  avg_mag=%.2f\n", s, avg_mag[s]);
    for (int p = 0; p < n_pairs; p++)
        LOG_INFO("  pair%d  baseline=%.2f  rms=%.2f  headroom=%.0f%%  streak=%d\n",
                 p, baseline[p], diff_rms[p], headroom[p] * 100.0f,
                 w->desync_streak[p]);

    if (status == 0)
        LOG_OK("  status=HEALTHY\n\n");
    else if (status == 1)
        fprintf(stderr, BOLD_RED "  status=NEAR THRESHOLD\n\n" RESET);
    else
        fprintf(stderr, BOLD_RED "  status=STREAK ACTIVE\n\n" RESET);

    /* Save to file */
    char path[512];
    snprintf(path, sizeof(path), "%s/snapshot_%03u.txt", g_run_dir, g_snapshot_count);
    FILE *f = fopen(path, "w");
    if (f) {
        fprintf(f, "timestamp=%s\nuptime_s=%ld\ndesync_count=%u\n",
                ts, uptime, g_ctx.lag->desync_count);
        for (int s = 0; s < n_sdrs; s++)
            fprintf(f, "avg_mag_sdr%d=%.2f\n", s, avg_mag[s]);
        for (int p = 0; p < n_pairs; p++) {
            float thresh = fmaxf(fabsf(baseline[p]) * 8.0f, (float)floor_thresh);
            fprintf(f, "baseline_pair%d=%.2f\ndiff_rms_pair%d=%.2f\n"
                       "threshold_pair%d=%.2f\nheadroom_pair%d=%.2f\nstreak_pair%d=%d\n",
                    p, baseline[p], p, diff_rms[p], p, thresh,
                    p, headroom[p], p, w->desync_streak[p]);
        }
        fprintf(f, "healthy=%d\n", status == 0 ? 1 : 0);
        fclose(f);
    }

    /* Binary sample dump — SNAPSHOT_LOOK_BACK rows of (n_sdrs uint8 mags +
     * n_pairs int16 diffs) in chronological order, same layout as window.bin.
     * Used by view_desync.py to plot alignment of the live streams. */
    snprintf(path, sizeof(path), "%s/snapshot_%03u.bin", g_run_dir, g_snapshot_count);
    FILE *bf = fopen(path, "wb");
    if (bf) {
        for (int k = SNAPSHOT_LOOK_BACK - 1; k >= 0; k--) {
            uint32_t idx = (pos - 1 - (uint32_t)k) & (WINDOW_SIZE - 1);
            for (int s = 0; s < n_sdrs; s++)
                fwrite(&w->mags[s][idx], 1, 1, bf);
            for (int p = 0; p < n_pairs; p++)
                fwrite(&w->diffs[p][idx], sizeof(int16_t), 1, bf);
        }
        fclose(bf);
    }
}

/* ------------------------------------------------------------------ */
/* Debug / force-desync                                                 */
/* ------------------------------------------------------------------ */

static void inject_desync(void)
{
    g_last_desync_forced = 1;
    LOG_INFO("[longevity] Injecting forced desync on ring0 (%d samples)\n",
             FORCE_DESYNC_SAMPLES);
    pthread_mutex_lock(&g_ctx.rings[0]->lock);
    g_ctx.rings[0]->read_pos += FORCE_DESYNC_SAMPLES * 2;
    pthread_cond_signal(&g_ctx.rings[0]->not_empty);
    pthread_mutex_unlock(&g_ctx.rings[0]->lock);
}

static void *debug_thread(void *arg)
{
    (void)arg;
    char line[64];
    int stdin_open = 1;

    while (!g_quit) {
        if (g_sigusr1) {
            g_sigusr1 = 0;
            inject_desync();
        }

        if (!stdin_open) {
            nanosleep(&(struct timespec){0, 100000000}, NULL);
            continue;
        }

        struct timeval tv = { .tv_sec = 0, .tv_usec = 100000 };
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(STDIN_FILENO, &rfds);

        if (select(STDIN_FILENO + 1, &rfds, NULL, NULL, &tv) > 0) {
            if (fgets(line, sizeof(line), stdin)) {
                line[strcspn(line, "\n")] = '\0';
                if (strcmp(line, "desync") == 0) {
                    inject_desync();
                } else if (strcmp(line, "snapshot") == 0 || strcmp(line, "s") == 0) {
                    take_snapshot();
                } else if (strcmp(line, "q") == 0 || strcmp(line, "quit") == 0) {
                    g_quit = 1;
                    if (g_monitor_stop_ptr) *g_monitor_stop_ptr = 1;
                } else {
                    LOG_INFO("[longevity] Commands: desync | snapshot | quit\n");
                }
            } else {
                stdin_open = 0;
            }
        }
    }
    return NULL;
}

/* ------------------------------------------------------------------ */
/* Main                                                                 */
/* ------------------------------------------------------------------ */

int main(void)
{
    /* ---- Run directory --------------------------------------------- */
    if (setup_run_dir() < 0) return 1;

    /* ---- RTL-SDR init ----------------------------------------------- */
    LOG_HEADER("\n----- Longevity: RTL-SDR Init -----");

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
        /* Output file unused in async streaming mode */
        if (sdr_open(&sdrs[i], i, CENTER_FREQ, SAMPLE_RATE, INITIAL_GAIN, "/dev/null") < 0)
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
    LOG_ERR("[longevity] Failed to open SDR devices\n");
    for (int i = 0; i < n_opened; i++) { ring_free(&rings[i]); sdr_close(&sdrs[i]); }
    return 1;

sdr_ok:

    /* ---- BladeRF init ----------------------------------------------- */
    LOG_HEADER("\n----- Longevity: BladeRF Init -----");

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

    if (create_test_tone(&blade, 10000.0f, 1, 1) < 0) {
        blade_close(&blade);
        for (int i = 0; i < n_sdrs; i++) { ring_free(&rings[i]); sdr_close(&sdrs[i]); }
        return 1;
    }

    /* ---- Schedule simultaneous RX + TX start ------------------------- */
    LOG_HEADER("\n----- Longevity: Starting -----");

    struct timespec trigger;
    clock_gettime(CLOCK_REALTIME, &trigger);
    trigger.tv_nsec += 200000000L;
    if (trigger.tv_nsec >= 1000000000L) {
        trigger.tv_sec++;
        trigger.tv_nsec -= 1000000000L;
    }

    LOG_INFO("[longevity] Scheduling RX + TX in 200ms...\n");

    ring_buf_t      *ring_ptrs[n_sdrs];
    pthread_t        t_sdr[n_sdrs];
    sdr_start_args_t sdr_args[n_sdrs];
    for (int i = 0; i < n_sdrs; i++) {
        ring_ptrs[i] = &rings[i];
        sdr_args[i]  = (sdr_start_args_t){ .sdr = &sdrs[i], .trigger = &trigger };
        pthread_create(&t_sdr[i], NULL, sdr_start_thread, &sdr_args[i]);
    }

    volatile int tx_stop = 0;
    tx_args_t    ta      = { .blade = &blade, .stop_flag = &tx_stop, .trigger = &trigger };
    pthread_t    t_tx;
    pthread_create(&t_tx, NULL, tx_thread, &ta);

    for (int i = 0; i < n_sdrs; i++)
        pthread_join(t_sdr[i], NULL);

    /* ---- Phase 1: xcorr lag calibration ----------------------------- */
    LOG_HEADER("\n----- Longevity: Calibrating (pass 1) -----");

    lag_state_t lag = {0};
    if (lag_state_init(&lag, n_sdrs) < 0) {
        tx_stop = 1; pthread_join(t_tx, NULL);
        blade_close(&blade);
        for (int i = 0; i < n_sdrs; i++) { sdr_stream_stop(&sdrs[i]); ring_free(&rings[i]); sdr_close(&sdrs[i]); }
        return 1;
    }

    proc_args_t pa = {
        .n_sdrs           = n_sdrs,
        .rings            = ring_ptrs,
        .lag              = &lag,
        .stop_flag        = NULL,         /* set per-phase below */
        .on_window_event  = NULL,
        .min_diff_thresh  = 10,
        .read_timeout_ms  = 500,
    };

    volatile int proc_stop = 0;
    pa.stop_flag = &proc_stop;
    pthread_t t_proc;
    pthread_create(&t_proc, NULL, proc_thread_wrapper, &pa);

    for (int i = 0; i < CALIB_TIMEOUT && !lag_all_calibrated(&lag); i++)
        nanosleep(&(struct timespec){0, 50000000}, NULL);

    proc_stop = 1;
    pthread_join(t_proc, NULL);

    if (!lag_all_calibrated(&lag)) {
        LOG_ERR("[longevity] Pass 1 calibration timed out\n");
        dump_samples(rings, n_sdrs, g_run_dir, "timeout");
        tx_stop = 1; pthread_join(t_tx, NULL);
        lag_state_free(&lag);
        blade_close(&blade);
        for (int i = 0; i < n_sdrs; i++) { sdr_stream_stop(&sdrs[i]); ring_free(&rings[i]); sdr_close(&sdrs[i]); }
        return 1;
    }

    LOG_OK("[longevity] Pass 1 complete — %d pair(s) calibrated\n", n_sdrs - 1);
    dump_samples(rings, n_sdrs, g_run_dir, "aligned");

    /* ---- Phase 2: gain adjustment ----------------------------------- */
    LOG_HEADER("\n----- Longevity: Adjusting Gains -----");
    adjust_gains(sdrs, rings, n_sdrs);
    dump_samples(rings, n_sdrs, g_run_dir, "gained");

    /* ---- Phase 3: re-sync after gain disruption --------------------- */
    LOG_HEADER("\n----- Longevity: Calibrating (pass 2) -----");

    for (int p = 0; p < n_sdrs - 1; p++) lag.calibrated[p] = 0;

    volatile int proc_stop2 = 0;
    pa.stop_flag = &proc_stop2;
    pthread_t t_proc2;
    pthread_create(&t_proc2, NULL, proc_thread_wrapper, &pa);

    for (int i = 0; i < CALIB_TIMEOUT && !lag_all_calibrated(&lag); i++)
        nanosleep(&(struct timespec){0, 50000000}, NULL);

    proc_stop2 = 1;
    pthread_join(t_proc2, NULL);

    if (!lag_all_calibrated(&lag)) {
        LOG_ERR("[longevity] Pass 2 calibration timed out\n");
        dump_samples(rings, n_sdrs, g_run_dir, "timeout2");
        tx_stop = 1; pthread_join(t_tx, NULL);
        lag_state_free(&lag);
        blade_close(&blade);
        for (int i = 0; i < n_sdrs; i++) { sdr_stream_stop(&sdrs[i]); ring_free(&rings[i]); sdr_close(&sdrs[i]); }
        return 1;
    }

    LOG_OK("[longevity] Pass 2 complete — streams aligned and gains matched\n");
    dump_samples(rings, n_sdrs, g_run_dir, "resynced");

    /* ---- Monitoring ------------------------------------------------- */
    LOG_HEADER("\n----- Longevity: Monitoring -----");

    g_ctx.lag              = &lag;
    g_ctx.rings            = ring_ptrs;
    g_ctx.n_sdrs           = n_sdrs;
    g_ctx.last_desync_time = 0;
    g_ctx.run_start_time   = time(NULL);

    g_pa = &pa;
    signal(SIGUSR1, sigusr1_handler);
    LOG_INFO("[longevity] PID %d  —  commands: desync | snapshot (s) | quit\n",
             getpid());

    volatile int hb_stop = 0;
    heartbeat_args_t hb_args = { .stop_flag = &hb_stop, .start_time = g_ctx.run_start_time };
    pthread_t t_heartbeat;
    pthread_create(&t_heartbeat, NULL, heartbeat_thread, &hb_args);

    pthread_t t_debug;
    pthread_create(&t_debug, NULL, debug_thread, NULL);

    pa.on_window_event = on_desync;

    while (!g_quit) {
        volatile int monitor_stop = 0;
        g_monitor_stop_ptr = &monitor_stop;
        pa.stop_flag       = &monitor_stop;

        pthread_t t_monitor;
        pthread_create(&t_monitor, NULL, proc_thread_wrapper, &pa);
        pthread_join(t_monitor, NULL);

        g_monitor_stop_ptr = NULL;

        if (g_quit || pa.exit_reason == PROC_EXIT_STOPPED)
            break;

        if (pa.exit_reason == PROC_EXIT_DROPOUT) {
            time_t now    = time(NULL);
            long   uptime = (long)(now - g_ctx.run_start_time);
            char   ts[32];
            struct tm *tm_info = localtime(&now);
            strftime(ts, sizeof(ts), "%Y-%m-%dT%H:%M:%S", tm_info);

            char path[512];
            snprintf(path, sizeof(path), "%s/dropout.log", g_run_dir);
            FILE *f = fopen(path, "w");
            if (f) {
                fprintf(f, "timestamp=%s\nuptime_s=%ld\nsdr=%d\ndesync_count=%u\n",
                        ts, uptime, pa.dropout_sdr, lag.desync_count);
                fclose(f);
            }
            LOG_ERR("[longevity] SDR%d dropout at %s (uptime %lds) — exiting\n",
                    pa.dropout_sdr, ts, uptime);
            break;
        }

        /* PROC_EXIT_DESYNC — on_desync already logged; re-sync and continue */
        LOG_HEADER("\n----- Longevity: Re-Syncing -----");

        /* Pass 1: xcorr re-calibration */
        for (int p = 0; p < n_sdrs - 1; p++) lag.calibrated[p] = 0;
        for (int s = 0; s < n_sdrs; s++) ring_flush(&rings[s]);

        volatile int rs1_stop = 0;
        pa.stop_flag       = &rs1_stop;
        pa.on_window_event = NULL;
        pthread_t t_rs1;
        pthread_create(&t_rs1, NULL, proc_thread_wrapper, &pa);
        for (int i = 0; i < CALIB_TIMEOUT && !lag_all_calibrated(&lag); i++)
            nanosleep(&(struct timespec){0, 50000000}, NULL);
        rs1_stop = 1;
        pthread_join(t_rs1, NULL);

        if (!lag_all_calibrated(&lag)) {
            LOG_ERR("[longevity] Re-sync pass 1 timed out — exiting\n");
            break;
        }

        /* Gain re-match */
        adjust_gains(sdrs, rings, n_sdrs);

        /* Pass 2: xcorr re-calibration after gain disruption */
        for (int p = 0; p < n_sdrs - 1; p++) lag.calibrated[p] = 0;

        volatile int rs2_stop = 0;
        pa.stop_flag = &rs2_stop;
        pthread_t t_rs2;
        pthread_create(&t_rs2, NULL, proc_thread_wrapper, &pa);
        for (int i = 0; i < CALIB_TIMEOUT && !lag_all_calibrated(&lag); i++)
            nanosleep(&(struct timespec){0, 50000000}, NULL);
        rs2_stop = 1;
        pthread_join(t_rs2, NULL);

        if (!lag_all_calibrated(&lag)) {
            LOG_ERR("[longevity] Re-sync pass 2 timed out — exiting\n");
            break;
        }

        LOG_OK("[longevity] Re-sync complete — resuming monitoring\n");
        pa.on_window_event = on_desync;
    }

    /* ---- Teardown --------------------------------------------------- */
    LOG_HEADER("\n----- Longevity: Done -----");

    g_pa    = NULL;
    g_quit  = 1;
    hb_stop = 1;
    pthread_join(t_debug, NULL);
    pthread_join(t_heartbeat, NULL);

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
