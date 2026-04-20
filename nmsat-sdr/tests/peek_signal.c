#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <pthread.h>
#include <time.h>
#include "log.h"
#include "sdr.h"
#include "blade.h"
#include "process.h"

/* ~1 second of raw I/Q at 2.048 MSPS (2 bytes per sample) */
#define PEEK_BYTES (4 * 1024 * 1024)

/* ------------------------------------------------------------------ */

typedef struct {
    SDR              *sdr;
    struct timespec  *trigger;
} peek_sdr_args_t;

static void *peek_sdr_thread(void *arg)
{
    peek_sdr_args_t *a = (peek_sdr_args_t *)arg;
    clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, a->trigger, NULL);
    sdr_gather_samples(a->sdr, PEEK_BYTES);
    return NULL;
}

typedef struct {
    BLADE            *blade;
    volatile int     *stop_flag;
    struct timespec  *trigger;
} peek_tx_args_t;

static void *peek_tx_thread(void *arg)
{
    peek_tx_args_t *a = (peek_tx_args_t *)arg;
    clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, a->trigger, NULL);
    blade_transmit_loop(a->blade, a->stop_flag);
    return NULL;
}

/* ------------------------------------------------------------------ */

int peek_signal(void)
{
    LOG_HEADER("\n----- Peek: RTL-SDR Init -----");

    int n = sdr_count();
    LOG_INFO(TAG_RTL " Found %d device(s)\n", n);
    if (n < 2) {
        LOG_ERR(TAG_RTL " Need at least two devices\n");
        return 1;
    }

    SDR sdr0, sdr1;
    if (sdr_open(&sdr0, 0, 914800000, 2048000, 100, "peek_sdr0.bin") < 0) return 1;
    if (sdr_open(&sdr1, 1, 914800000, 2048000, 100, "peek_sdr1.bin") < 0) {
        sdr_close(&sdr0);
        return 1;
    }

    LOG_HEADER("\n----- Peek: BladeRF Init -----");

    int nb = blade_count();
    LOG_INFO(TAG_BLADE " Found %d device(s)\n", nb);
    if (nb < 1) {
        LOG_ERR(TAG_BLADE " No bladeRF found\n");
        sdr_close(&sdr0);
        sdr_close(&sdr1);
        return 1;
    }

    BLADE blade;
    if (blade_open(&blade, 0, 914800000, 2048000, 1500000, 0) < 0) {
        sdr_close(&sdr0);
        sdr_close(&sdr1);
        return 1;
    }

    if (create_test_tone(&blade, 10000.0f, 10, 100) < 0) {
        blade_close(&blade);
        sdr_close(&sdr0);
        sdr_close(&sdr1);
        return 1;
    }

    LOG_HEADER("\n----- Peek: Capturing -----");

    /* Schedule all threads to start simultaneously 200ms from now */
    struct timespec trigger;
    clock_gettime(CLOCK_REALTIME, &trigger);
    trigger.tv_nsec += 200000000L;
    if (trigger.tv_nsec >= 1000000000L) {
        trigger.tv_sec++;
        trigger.tv_nsec -= 1000000000L;
    }

    LOG_INFO("[peek] Starting TX + capture in 200ms...\n");

    volatile int stop_tx = 0;

    peek_tx_args_t  tx_args  = { .blade = &blade, .stop_flag = &stop_tx, .trigger = &trigger };
    peek_sdr_args_t sdr0_args = { .sdr = &sdr0, .trigger = &trigger };
    peek_sdr_args_t sdr1_args = { .sdr = &sdr1, .trigger = &trigger };

    pthread_t t_tx, t_sdr0, t_sdr1;
    pthread_create(&t_tx,   NULL, peek_tx_thread,  &tx_args);
    pthread_create(&t_sdr0, NULL, peek_sdr_thread, &sdr0_args);
    pthread_create(&t_sdr1, NULL, peek_sdr_thread, &sdr1_args);

    /* Wait for both captures to complete, then stop TX */
    pthread_join(t_sdr0, NULL);
    pthread_join(t_sdr1, NULL);
    stop_tx = 1;
    pthread_join(t_tx, NULL);

    sdr_flush_to_file(&sdr0);
    sdr_flush_to_file(&sdr1);

    LOG_INFO("[peek] Wrote peek_sdr0.bin (%d bytes)\n", PEEK_BYTES);
    LOG_INFO("[peek] Wrote peek_sdr1.bin (%d bytes)\n", PEEK_BYTES);

    blade_close(&blade);
    sdr_close(&sdr0);
    sdr_close(&sdr1);

    LOG_HEADER("\n----- Peek: Done -----");
    return 0;
}

int main(void) { return peek_signal(); }
