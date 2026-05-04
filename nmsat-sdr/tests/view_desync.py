#!/usr/bin/env python3
"""
Interactive viewer for longevity runs.

Two views are available:

  Health snapshot  — plots a snapshot of live sample data (magnitude overlay
                     + per-pair diff with detection band) so you can verify
                     that xcorr alignment is still holding.

  Desync detail    — plots the forensic window around a captured desync event
                     (magnitude overlay + per-pair diff with detection band).

Both views share the same plot layout; the snapshot view has no event marker.
"""

import os
import sys
import numpy as np
import matplotlib.pyplot as plt

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_ROOT       = os.path.join(_SCRIPT_DIR, "..")
RUNS_DIR    = os.path.join(_ROOT, "longevity_runs")

WINDOW_SIZE      = 65536
SNAPSHOT_SAMPLES = 4096        # must match SNAPSHOT_LOOK_BACK in longevity.c
SAMPLE_RATE      = 2_048_000
EVENT_CENTER     = WINDOW_SIZE // 2
DIFF_AVG_LEN     = 32
MIN_THRESH       = 10


# ── File helpers ─────────────────────────────────────────────────────────────

def read_kv(path):
    """Read a key=value file into a dict."""
    out = {}
    try:
        with open(path) as f:
            for line in f:
                line = line.strip()
                if "=" in line:
                    k, v = line.split("=", 1)
                    out[k.strip()] = v.strip()
    except FileNotFoundError:
        pass
    return out


def read_summary(run_dir):
    path = os.path.join(run_dir, "summary.csv")
    rows = []
    try:
        with open(path) as f:
            header = f.readline().strip().split(",")
            for line in f:
                vals = line.strip().split(",")
                if len(vals) == len(header):
                    rows.append(dict(zip(header, vals)))
    except FileNotFoundError:
        pass
    return rows


def load_samples(path, n_rows):
    """
    Parse a binary sample file with a known row count.

    Layout per row: [mag_0 .. mag_{N-1}] (N × uint8)
                    [diff_0 .. diff_{P-1}] (P × int16, little-endian)
    where N = n_sdrs, P = n_sdrs - 1.

    Returns (n_sdrs, mags, diffs):
        mags  shape (n_rows, n_sdrs)  float32
        diffs shape (n_rows, n_pairs) float32
    """
    raw   = np.frombuffer(open(path, "rb").read(), dtype=np.uint8)
    total = len(raw)
    if total % n_rows != 0:
        raise ValueError(f"{os.path.basename(path)}: size {total} not divisible by {n_rows}")
    row_size = total // n_rows
    if (row_size + 2) % 3 != 0:
        raise ValueError(f"Cannot infer n_sdrs from row_size={row_size}")
    n_sdrs  = (row_size + 2) // 3
    n_pairs = n_sdrs - 1

    rows  = raw.reshape(n_rows, row_size)
    mags  = rows[:, :n_sdrs].astype(np.float32)

    if n_pairs > 0:
        diff_bytes = rows[:, n_sdrs:].flatten().copy()
        diffs = diff_bytes.view(np.int16).reshape(n_rows, n_pairs).astype(np.float32)
    else:
        diffs = np.zeros((n_rows, 0), dtype=np.float32)

    return n_sdrs, mags, diffs


# ── Causal rolling mean ───────────────────────────────────────────────────────

def rolling_mean(x, n):
    return np.convolve(x, np.ones(n) / float(n), mode="full")[:len(x)]


# ── Interactive selection ─────────────────────────────────────────────────────

def pick(prompt, options, labels=None):
    """
    Print numbered options and return selection.
    Accepts a single number, a comma-separated list, or 'a' for all.
    Returns a list of chosen options.
    """
    labels = labels or [str(o) for o in options]
    print(f"\n{prompt}")
    for i, lbl in enumerate(labels):
        print(f"  [{i}] {lbl}")
    print("  [a] all")

    while True:
        raw = input("Select: ").strip().lower()
        if raw in ("a", "all"):
            return list(options)
        parts = [p.strip() for p in raw.split(",")]
        if all(p.isdigit() for p in parts):
            indices = [int(p) for p in parts]
            if all(0 <= idx < len(options) for idx in indices):
                return [options[idx] for idx in indices]
        print(f"  Enter 0–{len(options)-1}, a comma-separated list, or 'a'")


# ── Shared plot core ──────────────────────────────────────────────────────────

def _plot_window(axes, n_sdrs, mags, diffs, t_ms, title, event_ms=None):
    """
    Draw magnitude overlay + per-pair diff rows into pre-created axes.
    Pass event_ms to draw a vertical marker (desync events only).
    """
    n_pairs = n_sdrs - 1
    COLORS  = plt.rcParams["axes.prop_cycle"].by_key()["color"]

    # Row 0: magnitude overlay
    ax = axes[0]
    for s in range(n_sdrs):
        ax.plot(t_ms, mags[:, s], label=f"sdr{s}",
                color=COLORS[s % len(COLORS)], linewidth=0.7, alpha=0.85)
    if event_ms is not None:
        ax.axvline(event_ms, color="white", linewidth=1.2, linestyle="--",
                   alpha=0.7, label=f"event (~{event_ms:.1f} ms)")
    ax.set_ylabel("Magnitude")
    ax.set_title(f"{title} — magnitude overlay")
    ax.legend(loc="upper right", fontsize=8)

    # Rows 1..n_pairs: diff + baseline + detection band
    for p in range(n_pairs):
        ax   = axes[1 + p]
        diff = diffs[:, p]

        baseline  = rolling_mean(diff, DIFF_AVG_LEN)
        threshold = np.maximum(np.abs(baseline) * 8.0, MIN_THRESH)

        ax.fill_between(t_ms, baseline - threshold, baseline + threshold,
                        alpha=0.18, color="orange", label="detection band")
        ax.plot(t_ms, baseline, color="orange", linewidth=0.9, alpha=0.7,
                label="baseline (32-samp avg)")
        ax.plot(t_ms, diff, color=COLORS[(p + 1) % len(COLORS)], linewidth=0.7,
                label=f"sdr0 − sdr{p + 1}")
        ax.axhline(0, color="gray", linewidth=0.5, linestyle="--")
        if event_ms is not None:
            ax.axvline(event_ms, color="white", linewidth=1.2,
                       linestyle="--", alpha=0.7)

        ax.set_ylabel("Δ magnitude")
        ax.set_title(
            f"Diff  sdr0 − sdr{p + 1}"
            f"  (detection band = baseline ± 8×|baseline|, floor {MIN_THRESH})"
        )
        ax.legend(loc="upper right", fontsize=8)

    axes[-1].set_xlabel("Time (ms)")


# ── Health snapshot view ──────────────────────────────────────────────────────

def plot_snapshot(run_dir, snap_name):
    """Plot a live-stream sample snapshot for alignment verification."""
    bin_path  = os.path.join(run_dir, f"{snap_name}.bin")
    meta_path = os.path.join(run_dir, f"{snap_name}.txt")

    if not os.path.exists(bin_path):
        print(f"  [skip] no .bin for {snap_name}")
        return

    meta    = read_kv(meta_path)
    n_sdrs, mags, diffs = load_samples(bin_path, SNAPSHOT_SAMPLES)
    n_pairs = n_sdrs - 1

    t_ms = np.arange(SNAPSHOT_SAMPLES) / SAMPLE_RATE * 1000.0

    ts        = meta.get("timestamp",    "?")
    uptime    = meta.get("uptime_s",     "?")
    n_desyncs = meta.get("desync_count", "?")
    healthy   = meta.get("healthy",      "?")

    status = "HEALTHY" if healthy == "1" else "DEGRADED" if healthy == "0" else "?"
    title  = f"{snap_name}   {ts}   uptime {uptime}s   desyncs: {n_desyncs}   [{status}]"

    lag_info = "   ".join(
        f"headroom pair{p}={float(meta[f'headroom_pair{p}']):.0%}"
        for p in range(n_pairs)
        if f"headroom_pair{p}" in meta
    )

    n_rows = 1 + max(n_pairs, 1)
    fig, axes = plt.subplots(n_rows, 1, figsize=(16, 4 * n_rows), sharex=True)
    if n_rows == 1:
        axes = [axes]

    fig.suptitle(title + (f"\n{lag_info}" if lag_info else ""), fontsize=10)

    _plot_window(axes, n_sdrs, mags, diffs, t_ms,
                 title="Alignment check", event_ms=None)

    plt.tight_layout()
    out = os.path.join(run_dir, f"{snap_name}_plot.png")
    plt.savefig(out, dpi=150, bbox_inches="tight")
    print(f"  Saved {out}")
    plt.show()
    plt.close()


# ── Desync event detail ───────────────────────────────────────────────────────

def plot_desync(run_dir, desync_name):
    """Plot the forensic window around a captured desync event."""
    desync_dir  = os.path.join(run_dir, desync_name)
    window_path = os.path.join(desync_dir, "window.bin")
    event_path  = os.path.join(desync_dir, "event.log")

    if not os.path.exists(window_path):
        print(f"  [skip] no window.bin in {desync_dir}")
        return

    meta    = read_kv(event_path)
    n_sdrs, mags, diffs = load_samples(window_path, WINDOW_SIZE)
    n_pairs = n_sdrs - 1

    t_ms     = np.arange(WINDOW_SIZE) / SAMPLE_RATE * 1000.0
    event_ms = t_ms[EVENT_CENTER]

    ts     = meta.get("timestamp",       "?")
    uptime = meta.get("uptime_s",        "?")
    inter  = meta.get("inter_arrival_s", "?")
    forced = meta.get("forced", "0") == "1"

    title = f"{desync_name}   {ts}   uptime {uptime}s"
    if inter not in ("-1.0", "-1", "?"):
        title += f"   inter-arrival {float(inter):.1f}s"
    if forced:
        title += "   [FORCED]"

    lag_info = "   ".join(
        f"lag pair{p}={meta[f'lag_pair{p}']}"
        for p in range(n_pairs)
        if f"lag_pair{p}" in meta
    )

    n_rows = 1 + max(n_pairs, 1)
    fig, axes = plt.subplots(n_rows, 1, figsize=(16, 4 * n_rows), sharex=True)
    if n_rows == 1:
        axes = [axes]

    fig.suptitle(title + (f"\n{lag_info}" if lag_info else ""), fontsize=10)

    _plot_window(axes, n_sdrs, mags, diffs, t_ms,
                 title=desync_name, event_ms=event_ms)

    plt.tight_layout()
    out = os.path.join(desync_dir, "plot.png")
    plt.savefig(out, dpi=150, bbox_inches="tight")
    print(f"  Saved {out}")
    plt.show()
    plt.close()


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    if not os.path.isdir(RUNS_DIR):
        print(f"No longevity_runs/ directory found (looked in {RUNS_DIR})")
        sys.exit(1)

    # ── Pick run ──
    run_names = sorted(
        d for d in os.listdir(RUNS_DIR)
        if d.startswith("run_") and os.path.isdir(os.path.join(RUNS_DIR, d))
    )
    if not run_names:
        print("No runs found in longevity_runs/")
        sys.exit(1)

    run_labels = []
    for r in run_names:
        rdir    = os.path.join(RUNS_DIR, r)
        summary = read_summary(rdir)
        hb      = read_kv(os.path.join(rdir, "heartbeat.log"))
        dropout = os.path.exists(os.path.join(rdir, "dropout.log"))
        n_snaps = sum(
            1 for f in os.listdir(rdir)
            if f.startswith("snapshot_") and f.endswith(".bin")
        )
        uptime  = hb.get("uptime_s", "?")
        parts   = [f"{len(summary)} desync(s)", f"uptime {uptime}s",
                   f"{n_snaps} snapshot(s)"]
        if dropout:
            parts.append("[DROPOUT]")
        run_labels.append(f"{r}  —  {', '.join(parts)}")

    chosen_runs = pick("Select a run:", run_names, run_labels)
    run_name    = chosen_runs[0]
    run_dir     = os.path.join(RUNS_DIR, run_name)

    # ── Dropout notice ──
    dropout_path = os.path.join(run_dir, "dropout.log")
    if os.path.exists(dropout_path):
        dm = read_kv(dropout_path)
        print(f"\n  [!] This run ended with a dropout: SDR{dm.get('sdr','?')} "
              f"at {dm.get('timestamp','?')} (uptime {dm.get('uptime_s','?')}s)")

    # ── Discover available data ──
    desync_names = sorted(
        d for d in os.listdir(run_dir)
        if d.startswith("desync_") and os.path.isdir(os.path.join(run_dir, d))
    )
    snap_names = sorted(
        f[:-4] for f in os.listdir(run_dir)
        if f.startswith("snapshot_") and f.endswith(".bin")
    )

    has_desyncs   = bool(desync_names)
    has_snapshots = bool(snap_names)

    if not has_desyncs and not has_snapshots:
        print("No desync events or health snapshots recorded in this run.")
        sys.exit(0)

    VIEW_SNAPSHOTS = "snapshots"
    VIEW_DESYNCS   = "desyncs"

    if has_desyncs and has_snapshots:
        view_options = [VIEW_SNAPSHOTS, VIEW_DESYNCS]
        view_labels  = [
            f"Alignment snapshots  ({len(snap_names)} snapshot(s))",
            f"Desync detail        ({len(desync_names)} event(s))",
        ]
        chosen_views = pick("What would you like to view?", view_options, view_labels)
    elif has_snapshots:
        chosen_views = [VIEW_SNAPSHOTS]
    else:
        chosen_views = [VIEW_DESYNCS]

    # ── Health snapshots ──
    if VIEW_SNAPSHOTS in chosen_views:
        snap_labels = []
        for sn in snap_names:
            meta    = read_kv(os.path.join(run_dir, f"{sn}.txt"))
            ts      = meta.get("timestamp", "?")
            uptime  = meta.get("uptime_s",  "?")
            healthy = "HEALTHY" if meta.get("healthy") == "1" else "DEGRADED"
            snap_labels.append(f"{sn}  {ts}  uptime {uptime}s  [{healthy}]")

        targets = pick("Select snapshot(s):", snap_names, snap_labels)
        for t in targets:
            print(f"\nPlotting {t} ...")
            plot_snapshot(run_dir, t)

    # ── Desync detail ──
    if VIEW_DESYNCS in chosen_views:
        if not has_desyncs:
            print("No desync events recorded in this run.")
        else:
            desync_labels = []
            for d in desync_names:
                meta   = read_kv(os.path.join(run_dir, d, "event.log"))
                ts     = meta.get("timestamp",       "?")
                inter  = meta.get("inter_arrival_s", "?")
                forced = "  [FORCED]" if meta.get("forced", "0") == "1" else ""
                if inter not in ("-1.0", "-1", "?"):
                    inter_str = f"  inter {float(inter):.1f}s"
                else:
                    inter_str = "  (first event)"
                desync_labels.append(f"{d}  {ts}{inter_str}{forced}")

            targets = pick("Select desync event(s):", desync_names, desync_labels)
            for t in targets:
                print(f"\nPlotting {t} ...")
                plot_desync(run_dir, t)


if __name__ == "__main__":
    main()
