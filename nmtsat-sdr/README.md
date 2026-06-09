# nmtsat-sdr

Real-time RF coherence monitor. RTL-SDR receivers capture the same signal
simultaneously while a BladeRF transmits a known test tone. A processing thread
detects when receivers fall out of sync.

## Requirements

- 2+ RTL-SDR devices
- 1 BladeRF
- Libraries: `librtlsdr`, `libbladeRF`, `libusb-1.0`, `pthreads`, `libm`
- Python 3 + `numpy` + `matplotlib`

## Build & run

```bash
# Build everything
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)

# Interactive desync event viewer
python3 tests/view_desync.py
```

---

## Library — `nmtsat_core`

### `src/sdr.h` — RTL-SDR device management

```c
int  sdr_count(void);

int  sdr_open(SDR *sdr, int index, uint32_t center_freq,
              uint32_t sample_rate, int gain, const char *output_filename);
void sdr_close(SDR *sdr);

int  sdr_stream_start(SDR *sdr);
void sdr_stream_stop(SDR *sdr);

int  sdr_gather_samples(SDR *sdr, uint32_t n);
void sdr_flush_to_file(SDR *sdr);
```

`sdr_open` opens a device by index, sets frequency/rate/gain, allocates a 50 MB
I/Q buffer, and opens an output file. `gain` is in tenths of dB (e.g. `496` =
49.6 dB); pass `0` for AGC.

`sdr_stream_start` frees the 50 MB buffer, then spawns a `SCHED_FIFO` USB read
thread that feeds all incoming samples into `sdr->ring`. Set `sdr->ring` before
calling. Raw bytes are 8-bit unsigned interleaved I/Q; signed value is
`byte − 127`.

`sdr_stream_stop` cancels the async read and joins the thread.

`sdr_gather_samples` / `sdr_flush_to_file` are for synchronous (non-streaming)
capture: read `n` bytes into `sdr->buf`, then write to `sdr->file`. Not used in
the streaming path.

---

### `src/blade.h` — BladeRF TX control

```c
int  blade_count(void);

int  blade_open(BLADE *blade, int index, uint64_t center_freq,
                uint32_t sample_rate, uint32_t bandwidth, int gain);
void blade_close(BLADE *blade);

int  create_test_tone(BLADE *blade, float tone_hz,
                      uint32_t on_ms, uint32_t off_ms);
int  blade_transmit_loop(BLADE *blade, volatile int *stop_flag);
int  blade_transmit(BLADE *blade, int16_t *buf, uint32_t n);
```

`blade_open` configures the TX channel in SC16_Q11 sync mode (±2047 full
scale).

`create_test_tone` generates a sine-wave burst at `tone_hz` offset from center,
with `on_ms` on / `off_ms` off. Buffer is stored in `blade->tone_buf` and freed
by `blade_close`.

`blade_transmit_loop` transmits the tone buffer continuously until `*stop_flag`
is set. Designed to run in its own thread.

---

### `src/process.h` — Signal processing

#### Ring buffer

```c
int  ring_init(ring_buf_t *r);
void ring_free(ring_buf_t *r);
void ring_write(ring_buf_t *r, const uint8_t *src, size_t len);
void ring_read (ring_buf_t *r, uint8_t *dst, size_t len);
int  ring_read_timed(ring_buf_t *r, uint8_t *dst, size_t len,
                     uint32_t timeout_ms);
```

4 MB power-of-2 FIFO. `ring_write` overwrites oldest data if full. `ring_read`
blocks until data is available. `ring_read_timed` returns `-1` on timeout — used
to detect a dead SDR.

#### Sliding window

```c
int  window_init(window_t *w, int n_sdrs);
void window_free(window_t *w);
void window_reset(window_t *w);
int  window_push(window_t *w, const uint8_t *mags);
```

Maintains a 65536-sample magnitude history per SDR and a 32-sample rolling
average of the pairwise difference (sdr0 − sdr1, sdr0 − sdr2, …).

`window_push` accepts one magnitude per SDR and returns a bitmask: bit `p` set
means pair `p` triggered a desync. Detection fires when
`|diff − baseline| > max(8 × |baseline|, MIN_DIFF_THRESH)` for two consecutive
samples. Inactive until `WARMUP_SAMPLES` (65536) have been pushed.

`fast_mag(i, q)` — inline Alpha-Max Beta-Min approximation
(`max + min/4 + min/8`, ~5% max error, branchless).

#### Processing thread

```c
void *processing_thread(void *arg);  /* arg is proc_args_t * */
```

Reads one I/Q pair per SDR per iteration, computes magnitudes, and drives the
sliding window.

**Phase 1 — lag calibration:** after warmup, cross-correlates each SDR's
magnitude ring against SDR0 over ±2048 samples (±1 ms at 2.048 MSPS),
mean-subtracting to remove DC bias. Advances each ring's read pointer to align
all streams to the slowest receiver.

**Phase 2 — desync detection:** active once all pairs are calibrated. Collects
`WINDOW_SIZE/2` post-event samples (so the trigger lands at the midpoint of the
dump), then calls `on_window_event`, sets `exit_reason`, and exits via
`*stop_flag = 1`.

`exit_reason` values written before return:

| Value | Meaning |
|-------|---------|
| `PROC_EXIT_STOPPED` | `*stop_flag` was set externally |
| `PROC_EXIT_DESYNC`  | magnitude desync detected |
| `PROC_EXIT_DROPOUT` | `ring_read_timed` timed out — SDR stopped |

---

## Executables

### `nmtsat_sync` — one-shot characterization test

Runs a four-phase pipeline then exits:

| Phase | Description | Dump |
|-------|-------------|------|
| 1 | xcorr alignment | `sync_sdr{N}_aligned.bin` |
| 2 | gain matching (dB-law jump) | `sync_sdr{N}_gained.bin` |
| 3 | re-sync after gain disruption | `sync_sdr{N}_resynced.bin` |
| 4 | retune SDR 1..N to 100 MHz and back | `sync_sdr{N}_retune.bin` |

`tests/visualize_sync.py` plots all four stages as `sync_plot.png`.

### `nmtsat_longevity` — long-running coherence monitor

Runs the same three-phase startup (align → gain → re-align), then monitors
indefinitely. On each desync the process:

1. Saves `window.bin` and `event.log` under
   `longevity_runs/run_YYYYMMDD_HHMMSS/desync_NNN/`
2. Re-runs the three-phase sync pipeline
3. Resumes monitoring

A `heartbeat.log` is rewritten every second. All events are appended to
`summary.csv`. A `dropout.log` is written and the process exits if any SDR
stops delivering samples.

**Debug controls:**

| Input | Effect |
|-------|--------|
| `snapshot` / `s` + Enter | Capture live sample window and save `snapshot_NNN.bin` + `snapshot_NNN.txt` |
| `desync` + Enter | Force a desync (advances ring0 by 2048 samples) |
| `quit` / `q` + Enter | Graceful shutdown |
| `kill -USR1 <pid>` | Same as `desync` |

`tests/view_desync.py` is an interactive viewer that traverses `longevity_runs/`
and offers two views:

- **Alignment snapshot** — loads a `snapshot_NNN.bin` (4096 samples of live
  magnitude + diff data) and plots the SDR streams overlaid with the rolling
  baseline and detection band. Use this to verify xcorr alignment is still
  holding between desyncs.
- **Desync detail** — loads `desync_NNN/window.bin` (65536-sample forensic
  window) and plots the same layout with the event trigger marked at the
  midpoint.
