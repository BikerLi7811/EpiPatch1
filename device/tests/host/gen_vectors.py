"""Synthetic ECG / Bio-Z / PPG recordings with known truth for the host tests,
plus scipy reference outputs for the generated biquad filters.

usage: python gen_vectors.py <out_dir>
The segment timings and rates here are mirrored in host_tests.cpp.
"""
import sys
from pathlib import Path

import numpy as np
from scipy import signal

rng = np.random.default_rng(7)


def ecg(fs=125.0):
    # (start s, end s, bpm)
    segs = [(0, 60, 60), (60, 120, 100), (120, 180, 150), (180, 240, 72)]
    dur = segs[-1][1]
    t = np.arange(int(dur * fs)) / fs
    beats = []
    tb = 0.5
    for s0, s1, bpm in segs:
        while tb < s1:
            beats.append(tb)
            rr = 60.0 / bpm * (1 + 0.03 * rng.standard_normal())
            tb += rr
    beats = np.array(beats)
    x = np.zeros_like(t)
    waves = [  # (offset s, amplitude uV, width s)
        (-0.20, 150, 0.025), (-0.03, -120, 0.010), (0.0, 1200, 0.012),
        (0.03, -250, 0.010), (0.22, 300, 0.040)]
    for b in beats:
        rr = 60.0 / [bpm for s0, s1, bpm in segs if s0 <= b < s1][0]
        for off, amp, w in waves:
            o = off * np.sqrt(rr) if off > 0.1 else off
            i0, i1 = int((b + o - 5 * w) * fs), int((b + o + 5 * w) * fs) + 1
            i0, i1 = max(i0, 0), min(i1, len(t))
            x[i0:i1] += amp * np.exp(-0.5 * ((t[i0:i1] - b - o) / w) ** 2)
    x += 300 * np.sin(2 * np.pi * 0.25 * t)          # baseline wander (breathing)
    x += 40 * np.sin(2 * np.pi * 50 * t)             # mains
    x += 15 * rng.standard_normal(len(t))            # white noise
    for c in (30, 95, 150):                          # muscle-noise bursts
        i = slice(int(c * fs), int((c + 2) * fs))
        x[i] += 60 * rng.standard_normal(x[i].shape[0])
    x += 20000                                       # electrode DC offset
    return x.astype(np.float32), beats


def bioz(fs=125.0):
    # (start, end, breaths/min); 0 = apnea
    segs = [(0, 90, 14), (90, 180, 32), (180, 240, 0), (240, 330, 12)]
    t = np.arange(int(segs[-1][1] * fs)) / fs
    phase = np.zeros_like(t)
    amp = np.zeros_like(t)
    ph = 0.0
    for i, ti in enumerate(t):
        bpm = [b for s0, s1, b in segs if s0 <= ti < s1][0]
        ph += 2 * np.pi * bpm / 60.0 / fs
        phase[i] = ph
        amp[i] = 1.0 if bpm else 0.0
    amp = np.convolve(amp, np.ones(int(2 * fs)) / (2 * fs), mode="same")  # smooth on/off
    x = 1000 * amp * (np.sin(phase) + 0.25 * np.sin(2 * phase + 0.6))
    x += 120 * np.sin(2 * np.pi * 1.2 * t)           # cardiogenic impedance
    x += 400 * np.sin(2 * np.pi * 0.01 * t)          # slow drift
    x += 25 * rng.standard_normal(len(t))
    x += 3.0e6                                       # body impedance DC
    return x.astype(np.float32)


def pulse(p):
    return np.exp(-0.5 * ((p - 0.2) / 0.1) ** 2) + 0.3 * np.exp(-0.5 * ((p - 0.55) / 0.08) ** 2)


_grid = np.linspace(0, 1, 10001)
PULSE_P2P = pulse(_grid).max() - pulse(_grid).min()


def ppg(fs=25.0):
    # (start, end, bpm, PI %)
    segs = [(0, 120, 75, 2.0), (120, 240, 110, 0.6)]
    t = np.arange(int(segs[-1][1] * fs)) / fs
    dc_ir, dc_red, ratio = 120000.0, 90000.0, 0.6     # R = 0.6 -> SpO2 ~ 95 %
    ph = 0.0
    ir = np.zeros_like(t)
    red = np.zeros_like(t)
    for i, ti in enumerate(t):
        _, _, bpm, pi = [s for s in segs if s[0] <= ti < s[1]][0]
        ph += bpm / 60.0 / fs
        p = ph % 1.0
        # blood-volume pulse: fast systolic rise, slow fall, dicrotic bump; peak-to-trough 1
        v = pulse(p) / PULSE_P2P
        ac_ir = pi / 100.0 * dc_ir
        ac_red = ratio * pi / 100.0 * dc_red
        ir[i] = dc_ir - ac_ir * v        # more blood -> less light
        red[i] = dc_red - ac_red * v
    ir += 30 * rng.standard_normal(len(t)) + 800 * np.sin(2 * np.pi * 0.05 * t)
    red += 30 * rng.standard_normal(len(t)) + 600 * np.sin(2 * np.pi * 0.05 * t)
    return red.astype(np.uint32), ir.astype(np.uint32)


def main():
    out = Path(sys.argv[1])
    out.mkdir(parents=True, exist_ok=True)
    e, beats = ecg()
    e.tofile(out / "ecg_uv.f32")
    beats.astype(np.float32).tofile(out / "ecg_beats_s.f32")
    bioz().tofile(out / "bioz.f32")
    red, ir = ppg()
    red.tofile(out / "ppg_red.u32")
    ir.tofile(out / "ppg_ir.u32")

    ref = np.load(Path(__file__).resolve().parent / "filters_ref.npz")
    x = rng.standard_normal(2000).astype(np.float32)
    x.tofile(out / "filt_in.f32")
    for name in ref.files:
        signal.sosfilt(ref[name], x.astype(np.float64)).astype(np.float32).tofile(out / f"filt_{name}.f32")
    print("beats", len(beats), "->", out)


if __name__ == "__main__":
    main()
