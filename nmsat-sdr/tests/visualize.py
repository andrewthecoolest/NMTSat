#!/usr/bin/env python3
"""
Visualize raw I/Q captures from peek_signal.
Usage: python3 visualize.py [peek_sdr0.bin] [peek_sdr1.bin]
"""

import sys
import numpy as np
import matplotlib.pyplot as plt

SDR0_FILE = sys.argv[1] if len(sys.argv) > 1 else "peek_sdr0.bin"
SDR1_FILE = sys.argv[2] if len(sys.argv) > 2 else "peek_sdr1.bin"

SAMPLE_RATE = 2_048_000  # Hz


def load_iq(path):
    raw = np.frombuffer(open(path, "rb").read(), dtype=np.uint8).astype(np.float32)
    i = raw[0::2] - 127.0
    q = raw[1::2] - 127.0
    return i + 1j * q


def fast_mag(iq):
    """Alpha-max beta-min — mirrors fast_mag() in process.h (~5% max error)."""
    ai = np.abs(iq.real)
    aq = np.abs(iq.imag)
    mx = np.maximum(ai, aq)
    mn = np.minimum(ai, aq)
    return mx + (mn / 4) + (mn / 8)


print(f"Loading {SDR0_FILE} ...")
s0 = load_iq(SDR0_FILE)
print(f"Loading {SDR1_FILE} ...")
s1 = load_iq(SDR1_FILE)

# Trim to the shorter file
n = min(len(s0), len(s1))
s0, s1 = s0[:n], s1[:n]

mag0 = fast_mag(s0)
mag1 = fast_mag(s1)
diff = mag0.astype(np.float32) - mag1.astype(np.float32)

t = np.arange(n) / SAMPLE_RATE * 1000  # ms

# ── Plot ──────────────────────────────────────────────────────────────────────
fig, axes = plt.subplots(3, 1, figsize=(16, 9), sharex=True)
fig.suptitle("peek_signal capture", fontsize=13)

# 1. Magnitude overlay
ax = axes[0]
ax.plot(t, mag0, label="sdr0", alpha=0.75, linewidth=0.6)
ax.plot(t, mag1, label="sdr1", alpha=0.75, linewidth=0.6)
ax.set_ylabel("Magnitude")
ax.set_title("Magnitude — both SDRs")
ax.legend(loc="upper right")

# 2. Diff (what window_push tracks)
ax = axes[1]
ax.plot(t, diff, color="coral", linewidth=0.6)
ax.axhline(0, color="gray", linewidth=0.5, linestyle="--")
ax.set_ylabel("mag0 − mag1")
ax.set_title("Diff (window_push input)")

# 3. Spectrogram of sdr0 (quick sanity check that the tone is there)
ax = axes[2]
ax.specgram(s0.real, Fs=SAMPLE_RATE, NFFT=1024, noverlap=512,
            cmap="inferno", scale="dB")
ax.set_ylabel("Freq (Hz)")
ax.set_xlabel("Time (s)")
ax.set_title("Spectrogram — sdr0 real component")

plt.tight_layout()
plt.show()
