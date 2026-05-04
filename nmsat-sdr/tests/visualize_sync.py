#!/usr/bin/env python3
"""
Visualize sync_test dumps: aligned (pre-gain) and gained (post-gain) side by side.
"""

import os
import re
import glob
import numpy as np
import matplotlib.pyplot as plt

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_ROOT       = os.path.join(_SCRIPT_DIR, "..")

SAMPLE_RATE = 2_048_000


def load_iq(path):
    raw = np.frombuffer(open(path, "rb").read(), dtype=np.uint8).astype(np.float32)
    return (raw[0::2] - 127.0) + 1j * (raw[1::2] - 127.0)


def fast_mag(iq):
    ai = np.abs(iq.real)
    aq = np.abs(iq.imag)
    mx = np.maximum(ai, aq)
    mn = np.minimum(ai, aq)
    return mx + (mn / 4) + (mn / 8)


def load_stage(suffix):
    pattern = os.path.join(_ROOT, f"sync_sdr*_{suffix}.bin")
    files = sorted(glob.glob(pattern),
                   key=lambda p: int(re.search(r'sync_sdr(\d+)_', p).group(1)))
    if not files:
        return None, []
    sigs = []
    for f in files:
        iq = load_iq(f)
        if len(iq) > 0:
            sigs.append(iq)
    return files, sigs


def draw_stage(axes, sigs, title, colors):
    n_sdrs  = len(sigs)
    n_pairs = n_sdrs - 1
    n       = min(len(s) for s in sigs)
    sigs    = [s[:n] for s in sigs]
    mags    = [fast_mag(s) for s in sigs]
    t_ms    = np.arange(n) / SAMPLE_RATE * 1000.0

    # row 0: magnitude overlay
    ax = axes[0]
    for i, m in enumerate(mags):
        ax.plot(t_ms, m, label=f"sdr{i}", color=colors[i % len(colors)],
                alpha=0.85, linewidth=0.7)
    ax.axhline(10.0, color="white", linewidth=0.8, linestyle="--", alpha=0.4, label="target=10")
    ax.set_ylabel("Magnitude")
    ax.set_title(f"{title} — magnitude overlay")
    ax.legend(loc="upper right", fontsize=8)

    # rows 1..n_pairs: pairwise diffs
    for p in range(n_pairs):
        ax = axes[1 + p]
        diff = mags[0] - mags[p + 1]
        ax.plot(t_ms, diff, color=colors[(p + 1) % len(colors)], linewidth=0.7)
        ax.axhline(0, color="gray", linewidth=0.5, linestyle="--")
        ax.set_ylabel("Δ mag")
        ax.set_title(f"{title} — diff  sdr0 − sdr{p + 1}")

    # last row: FFT spectrum
    ax = axes[1 + n_pairs]
    freqs_khz = np.fft.fftshift(np.fft.fftfreq(n, d=1.0 / SAMPLE_RATE)) / 1e3
    for i, s in enumerate(sigs):
        psd = 20.0 * np.log10(np.fft.fftshift(np.abs(np.fft.fft(s))) / n + 1e-12)
        ax.plot(freqs_khz, psd, label=f"sdr{i}", color=colors[i % len(colors)],
                alpha=0.85, linewidth=0.6)
    ax.set_xlabel("Frequency (kHz, baseband)")
    ax.set_ylabel("Power (dB)")
    ax.set_title(f"{title} — FFT spectrum")
    ax.legend(loc="upper right", fontsize=8)

    for ax in axes:
        ax.set_xlabel("Time (ms)" if ax != axes[-1] else "Frequency (kHz, baseband)")


# ── Load all stages ──────────────────────────────────────────────────────────

STAGE_DEFS = [
    ("aligned",  "aligned (pre-gain)"),
    ("gained",   "gained (post-gain)"),
    ("resynced", "resynced (post-resync)"),
    ("retune",   "retune (post-retune)"),
]

stages = []
for suffix, label in STAGE_DEFS:
    files, sigs = load_stage(suffix)
    if sigs:
        stages.append((label, sigs))
        print(f"  {suffix}: {[os.path.basename(f) for f in files]}")

if not stages:
    print("No sync_sdr*_{aligned,gained,retune}.bin files found")
    raise SystemExit(1)

n_sdrs  = len(stages[0][1])
n_pairs = n_sdrs - 1
n_rows  = 1 + n_pairs + 1   # mag + diffs + fft
n_cols  = len(stages)

COLORS = plt.rcParams["axes.prop_cycle"].by_key()["color"]

fig, all_axes = plt.subplots(n_rows, n_cols,
                             figsize=(10 * n_cols, 4 * n_rows),
                             squeeze=False)
fig.suptitle(f"sync_test  —  {n_sdrs} SDR(s)  @  {SAMPLE_RATE/1e6:.3f} MSPS",
             fontsize=13)

for col, (title, sigs) in enumerate(stages):
    draw_stage(all_axes[:, col], sigs, title, COLORS)

plt.tight_layout()
out = os.path.join(_ROOT, "sync_plot.png")
plt.savefig(out, dpi=150, bbox_inches="tight")
print(f"Saved {out}")
