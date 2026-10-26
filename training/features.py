"""Single source of truth for the on-device feature pipeline.

Everything here is mirrored 1:1 in firmware/src/wheeze_dsp.c. If you change a
constant, re-run export.py so the generated C tables stay in sync.

Pipeline (device):  PDM mic -> 16 kHz PCM -> FIR decimate x4 -> 4 kHz
                    -> 256-pt Hann frames, hop 64 (16 ms)
                    -> |RFFT|^2 -> 32 HTK-mel bands (100..2000 Hz) -> ln()
                    -> 40-frame window (0.688 s) -> subtract window mean
                    -> int8 -> CNN
"""
import numpy as np
from scipy import signal

FS = 4000               # analysis rate; wheezes live in ~100-1000 Hz
# PCM rates the firmware can request from PDM20, best (lowest power) first.
# 8 kHz lets the mic run at a ~0.5-1 MHz PDM clock (low-power mode on most
# MEMS mics); 16 kHz is the fallback the Zephyr nRF PDM driver always supports.
MIC_RATES = (8000, 16000)
FS_MIC = 16000          # default / reference rate for tests
DECIM = FS_MIC // FS
N_FFT = 256             # 64 ms frame, 15.6 Hz bins
HOP = 64                # 16 ms hop
N_MELS = 32
F_MIN = 100.0           # drops heart sounds / DC / handling rumble
F_MAX = 2000.0
N_FRAMES = 40           # 40 * 16 ms + 48 ms = 0.688 s window (paper: 0.6 s)
LOG_EPS = 1e-10
DECIM_TAPS = 64         # must be a multiple of DECIM for arm_fir_decimate_f32
WIN_SAMPLES = HOP * (N_FRAMES - 1) + N_FFT


def hann_periodic(n=N_FFT):
    return (0.5 - 0.5 * np.cos(2 * np.pi * np.arange(n) / n)).astype(np.float32)


def _hz_to_mel(f):
    return 2595.0 * np.log10(1.0 + f / 700.0)


def _mel_to_hz(m):
    return 700.0 * (10.0 ** (m / 2595.0) - 1.0)


def mel_filterbank():
    """(N_MELS, N_FFT//2+1) triangular HTK filters, peak 1.0."""
    n_bins = N_FFT // 2 + 1
    freqs = np.arange(n_bins) * FS / N_FFT
    mpts = np.linspace(_hz_to_mel(F_MIN), _hz_to_mel(F_MAX), N_MELS + 2)
    hz = _mel_to_hz(mpts)
    fb = np.zeros((N_MELS, n_bins), np.float32)
    for m in range(N_MELS):
        lo, c, hi = hz[m], hz[m + 1], hz[m + 2]
        up = (freqs - lo) / (c - lo)
        down = (hi - freqs) / (hi - c)
        fb[m] = np.maximum(0.0, np.minimum(up, down))
        if fb[m].sum() == 0:  # guarantee every band touches >= 1 bin
            fb[m, int(round(c * N_FFT / FS))] = 1.0
    return fb


def decim_fir(fs_mic=FS_MIC):
    """Anti-alias low-pass for fs_mic -> 4 kHz (cutoff 1.8 kHz). Linear phase, so
    CMSIS's time-reversed coefficient order is the same array."""
    taps = DECIM_TAPS * fs_mic // FS_MIC  # same filter duration at every rate
    return signal.firwin(taps, 1800.0, fs=fs_mic).astype(np.float32)


def tonality(frames):
    """Per-frame peak prominence in the log-mel spectrum: how far the strongest
    band sticks out above its neighbours two bands away. Wheezes are tonal
    (narrow-band) and score high; breath noise is broadband and scores low.
    frames: (..., N_MELS) log-mel. Mirrored in FeatureExtractor::compute_frame."""
    f = np.asarray(frames, np.float32)
    prom = f[..., 2:-2] - 0.5 * (f[..., :-4] + f[..., 4:])
    return prom.max(axis=-1)


_WIN = hann_periodic()
_FB = mel_filterbank()


def logmel_frames(x):
    """x: float32 mono @ FS. Returns (n_frames, N_MELS) natural-log mel energies."""
    x = np.asarray(x, np.float32)
    if len(x) < N_FFT:
        return np.zeros((0, N_MELS), np.float32)
    n = 1 + (len(x) - N_FFT) // HOP
    idx = np.arange(N_FFT)[None, :] + HOP * np.arange(n)[:, None]
    frames = x[idx] * _WIN[None, :]
    spec = np.fft.rfft(frames, axis=1)
    power = (spec.real ** 2 + spec.imag ** 2).astype(np.float32)
    mel = power @ _FB.T
    return np.log(mel + LOG_EPS).astype(np.float32)


def normalize_window(win):
    """win: (..., N_FRAMES, N_MELS) log-mel. Gain-invariant: subtract the window mean."""
    return win - win.mean(axis=(-2, -1), keepdims=True)
