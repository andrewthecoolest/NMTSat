#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <pthread.h>
#include <sched.h>
#include <time.h>
#include "log.h"
#include "sdr.h"
#include "blade.h"
#include "process.h"

/* ------------------------------------------------------------------ */
/* Forensics Dump                                                       */
/* ------------------------------------------------------------------ */

static void window_dump(const window_t *w, const char *filename)
{
    FILE *f = fopen(filename, "wb");
    if (!f) {
        LOG_ERR("[process] window_dump: failed to open '%s'\n", filename);
        return;
    }
    /* Write WINDOW_SIZE entries of: mag0, mag1, diff(int16) = 4 bytes each */
    for (int i = 0; i < WINDOW_SIZE; i++) {
        uint32_t idx = (w->pos + (uint32_t)i) % WINDOW_SIZE;
        fwrite(&w->mag0[idx], 1, 1, f);
        fwrite(&w->mag1[idx], 1, 1, f);
        fwrite(&w->diff[idx], sizeof(int16_t), 1, f);
    }
    fclose(f);
    LOG_INFO("[process] Window dumped to '%s'\n", filename);
}

/* Shared RTC trigger — all threads wake at this exact moment */
static struct timespec g_trigger;

static void set_realtime(int cpu, int priority_offset)
{
    struct sched_param sp = {
        .sched_priority = sched_get_priority_max(SCHED_FIFO) - priority_offset
    };
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);

    cpu_set_t cpus;
    CPU_ZERO(&cpus);
    CPU_SET(cpu, &cpus);
    pthread_setaffinity_np(pthread_self(), sizeof(cpus), &cpus);
}

/* ---------- Timer thread ---------- */
typedef struct { volatile int *stop_flag; } timer_args_t;

static void *timer_thread(void *arg)
{
    timer_args_t *a = (timer_args_t *)arg;
    int seconds = 0;
    while (!(*a->stop_flag)) {
        sleep(1);
        if (*a->stop_flag) break;
        seconds++;
        LOG_INFO("[timer] %ds elapsed\n", seconds);
    }
    return NULL;
}

/* ---------- TX thread ---------- */
typedef struct { BLADE *blade; volatile int *stop_flag; } tx_args_t;

static void *tx_thread(void *arg)
{
    tx_args_t *a = (tx_args_t *)arg;
    set_realtime(3, 0);
    clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &g_trigger, NULL);
    LOG_INFO(TAG_BLADE " TX loop started\n");
    blade_transmit_loop(a->blade, a->stop_flag);
    return NULL;
}

/* ---------- Drop simulation thread ---------- */
typedef struct {
    ring_buf_t   *ring;      /* ring to stall (sdr0) */
    lag_state_t  *lag;
    volatile int *stop_flag;
} drop_args_t;

static void *drop_thread(void *arg)
{
    drop_args_t *a = (drop_args_t *)arg;

    /* Wait for alignment to complete */
    while (!a->lag->calibrated && !(*a->stop_flag))
        nanosleep(&(struct timespec){0, 5000000}, NULL);  /* poll every 5ms */

    if (*a->stop_flag) return NULL;

    /* 5 seconds of coherence before injecting the drop */
    for (int i = 0; i < 5 && !(*a->stop_flag); i++)
        sleep(1);

    if (*a->stop_flag) return NULL;

    /* Simulate a 2048-sample drop on sdr0: advance the ring read head forward,
     * as if the USB layer silently discarded one transfer worth of samples. */
    pthread_mutex_lock(&a->ring->lock);
    a->ring->read_pos += 2048 * 2;  /* 2 bytes per I/Q pair */
    pthread_cond_signal(&a->ring->not_empty);
    pthread_mutex_unlock(&a->ring->lock);

    LOG_INFO("[drop_sim] Injected 2048-sample drop on sdr0\n");

    return NULL;
}

/* ---------- Processing thread ---------- */
static void *proc_thread(void *arg)
{
    set_realtime(0, 1);  /* CPU 0, one step below SDR producers */
    clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &g_trigger, NULL);
    LOG_INFO("[process] Processing thread started\n");
    processing_thread(arg);
    return NULL;
}

/* ---------- test_function ---------- */
static int test_function(SDR *sdr0, SDR *sdr1, BLADE *blade)
{
    if (!blade->tone_buf) {
        LOG_ERR(TAG_BLADE " test_function: no tone loaded\n");
        return -1;
    }

    /* Allocate ring buffers */
    ring_buf_t ring0, ring1;
    if (ring_init(&ring0) < 0 || ring_init(&ring1) < 0) return -1;

    sdr0->ring = &ring0;
    sdr1->ring = &ring1;

    lag_state_t lag      = { .calibrated = 0, .lag_samples = 0, .desync_count = 0 };
    volatile int stop_flag = 0;

    /* Set RTC trigger 200ms from now */
    clock_gettime(CLOCK_REALTIME, &g_trigger);
    g_trigger.tv_nsec += 200000000L;
    if (g_trigger.tv_nsec >= 1000000000L) {
        g_trigger.tv_sec++;
        g_trigger.tv_nsec -= 1000000000L;
    }

    LOG_INFO("[process] Scheduling all threads to start in 200ms...\n");

    /* SDR producers — sdr_stream_start spawns internally, pinned in stream_thread */
    sdr_stream_start(sdr0);
    sdr_stream_start(sdr1);

    /* Timer thread */
    timer_args_t timer_args = { .stop_flag = &stop_flag };
    pthread_t t_timer;
    pthread_create(&t_timer, NULL, timer_thread, &timer_args);

    /* TX thread */
    tx_args_t tx_args = { .blade = blade, .stop_flag = &stop_flag };
    pthread_t t_tx, t_proc, t_drop;
    pthread_create(&t_tx, NULL, tx_thread, &tx_args);

    /* Processing thread */
    proc_args_t proc_args = {
        .ring0           = &ring0,
        .ring1           = &ring1,
        .lag             = &lag,
        .stop_flag       = &stop_flag,
        .on_window_event = window_dump
    };
    pthread_create(&t_proc, NULL, proc_thread, &proc_args);

    /* Drop simulation thread — fires once after 5s of coherence */
    drop_args_t drop_args = { .ring = &ring0, .lag = &lag, .stop_flag = &stop_flag };
    pthread_create(&t_drop, NULL, drop_thread, &drop_args);

    /* Wait for processing to finish (stop_flag set on post-calibration desync) */
    pthread_join(t_proc, NULL);

    /* Stop everything */
    stop_flag = 1;
    sdr_stream_stop(sdr0);
    sdr_stream_stop(sdr1);
    pthread_join(t_drop, NULL);
    pthread_join(t_tx, NULL);
    pthread_join(t_timer, NULL);

    ring_free(&ring0);
    ring_free(&ring1);

    LOG_INFO("[process] Total desyncs: %u | Lag: %d samples\n",
             lag.desync_count, lag.lag_samples);

    return 0;
}

/* ---------- main ---------- */
int main()
{
    LOG_HEADER("\n----- RTL-SDR Init -----");

    int n = sdr_count();
    LOG_INFO(TAG_RTL " Found %d device(s)\n", n);
    if (n < 2) {
        LOG_ERR(TAG_RTL " Error: need two\n");
        return 1;
    }

    SDR sdr0, sdr1;

    if (sdr_open(&sdr0, 0, 914800000, 2048000, "sdr0.bin") < 0) return 1;
    if (sdr_open(&sdr1, 1, 914800000, 2048000, "sdr1.bin") < 0) {
        sdr_close(&sdr0);
        return 1;
    }

    LOG_HEADER("\n----- BladeRF Init -----");

    int nb = blade_count();
    LOG_INFO(TAG_BLADE " Found %d device(s)\n", nb);
    if (nb < 1) {
        LOG_ERR(TAG_BLADE " Error: no bladeRF found\n");
        sdr_close(&sdr0);
        sdr_close(&sdr1);
        return 1;
    }

    BLADE blade;
    if (blade_open(&blade, 0, 914800000, 2048000, 1500000, 30) < 0) {
        sdr_close(&sdr0);
        sdr_close(&sdr1);
        return 1;
    }

    /* 1ms on / 1ms off blip at 10 kHz baseband offset */
    if (create_test_tone(&blade, 10000.0f, 1, 1) < 0) {
        blade_close(&blade);
        sdr_close(&sdr0);
        sdr_close(&sdr1);
        return 1;
    }

    LOG_HEADER("\n----- Running Test -----");
    test_function(&sdr0, &sdr1, &blade);

    blade_close(&blade);
    sdr_close(&sdr0);
    sdr_close(&sdr1);

    LOG_HEADER("\n----- Done -----");
    return 0;
}
