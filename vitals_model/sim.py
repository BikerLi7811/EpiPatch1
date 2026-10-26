"""Synthetic 1 Hz vitals for training and testing the anaphylaxis decision.

No public dataset records anaphylaxis with ECG, Bio-Z, PPG, a chest mic, two
IMUs and a histamine sensor, so this simulates what the firmware's detectors
would output each second (the same fields as dev::Vitals), for one simulated
wearer per 40-minute recording. Physiology ranges follow the anaphylaxis,
asthma, syncope and exercise literature, but THIS IS NOT PATIENT DATA: the
model learns the scenarios as written here, and must be re-trained on real
recordings before it means anything clinically.

Each recording has one scenario. The event starts at t0 (exposure, or the
start of the confounder) and, for anaphylaxis, organ involvement starts at t1.

usage: python sim.py <out.npz> [n_recordings] [seed] [shift]
  shift: a harder world for robustness testing -- weaker wheeze detector with
  3x the false events, 1.5x sensor noise, smaller histamine rises
"""
import sys

import numpy as np

T = 2400  # seconds per recording

# columns of the per-second vitals array (what the firmware's sensors report)
COLS = ["hr_valid", "hr", "rr_valid", "rr", "apnea", "ppg_valid", "pi", "spo2",
        "wheezing", "activity", "collapsed", "fall", "hist_ok", "hist_nm"]
C = {n: i for i, n in enumerate(COLS)}

# scenario -> (share of recordings, is anaphylaxis)
SCENARIOS = {
    "anaphylaxis_resp": (0.13, True),    # wheeze/stridor, fast breathing
    "anaphylaxis_cv": (0.11, True),      # tachycardia, perfusion drop
    "anaphylaxis_severe": (0.09, True),  # both, then collapse
    "anaphylaxis_low_hist": (0.05, True),  # sensor barely responds
    "normal_day": (0.14, False),
    "sleep": (0.07, False),
    "exercise": (0.10, False),
    "asthma": (0.08, False),
    "allergy_mild": (0.07, False),       # histamine up, skin only
    "vasovagal": (0.05, False),          # faint: HR/PI drop, collapse
    "fall": (0.04, False),               # trip, no reaction
    "panic": (0.04, False),              # tachycardia + hyperventilation
    "hist_artifact": (0.03, False),      # sensor drift (sweat, pressure)
}

# wheeze detector, per second (from the ICBHI results: ~0.64 sensitivity, and
# false events that rise with heavy breathing and speech)
WHEEZE_SENS = 0.55
WHEEZE_FP_REST = 1.5 / 3600
WHEEZE_FP_ACTIVE = 8.0 / 3600
NOISE = 1.0       # measurement-noise multiplier
HIST_GAIN = 1.0   # histamine-rise multiplier


def ramp(t, start, dur):
    """0 before start, rising smoothly to 1 over dur seconds."""
    x = np.clip((t - start) / max(dur, 1.0), 0.0, 1.0)
    return x * x * (3 - 2 * x)


def smooth_noise(rng, n, sd, tau):
    """AR(1) noise with time constant tau [s]."""
    a = np.exp(-1.0 / tau)
    e = rng.normal(0, sd * np.sqrt(1 - a * a), n)
    out = np.empty(n)
    acc = rng.normal(0, sd)
    for i in range(n):
        acc = a * acc + e[i]
        out[i] = acc
    return out


def walking_bouts(rng, t, n_bouts, avoid=None):
    """Activity [g] and HR/RR offsets for a few short walks."""
    act = np.zeros(T)
    for _ in range(n_bouts):
        s = rng.integers(60, T - 300)
        if avoid is not None and avoid[0] - 400 < s < avoid[1]:
            continue
        d = rng.integers(60, 300)
        act[s:s + d] = rng.uniform(0.10, 0.30)
    return act


def simulate(rng, scen):
    t = np.arange(T, dtype=float)
    hr0 = rng.uniform(55, 85)
    rr0 = rng.uniform(12, 18)
    pi0 = float(np.exp(rng.normal(np.log(2.5), 0.45)))
    hist0 = rng.uniform(5, 20)
    spo20 = rng.uniform(96, 99)

    t0 = float(rng.integers(900, 1500))
    t1 = np.nan
    hr = hr0 + smooth_noise(rng, T, 3.0, 120)
    rr = rr0 + smooth_noise(rng, T, 1.0, 120)
    pi = pi0 * np.exp(smooth_noise(rng, T, 0.12, 200))
    spo2 = spo20 + smooth_noise(rng, T, 0.4, 100)
    hist = hist0 * np.exp(smooth_noise(rng, T, 0.05, 600))
    act = 0.015 + 0.01 * rng.random(T)
    wheeze_p = np.zeros(T)      # probability the detector fires this second
    apnea = np.zeros(T, bool)
    lying = np.zeros(T, bool)
    collapse_t, fall_t = [], []

    if scen != "sleep":
        act += walking_bouts(rng, t, rng.integers(0, 4), avoid=(t0, T))
    moving = act > 0.08
    hr += 25 * moving
    rr += 6 * moving

    if scen.startswith("anaphylaxis"):
        # histamine first, then organs (minutes later)
        rise = 1.0 if scen != "anaphylaxis_low_hist" else 0.0
        peak = rise * (1 + HIST_GAIN * rng.uniform(1.0, 5.0)) + (1 - rise) * rng.uniform(1.05, 1.3)
        hist *= 1 + (peak - 1) * ramp(t, t0, rng.uniform(120, 600))
        t1 = t0 + rng.uniform(120, 900)
        dur = rng.uniform(120, 600)   # how fast it gets worse
        g = ramp(t, t1, dur)
        resp = scen in ("anaphylaxis_resp", "anaphylaxis_severe") or (
            scen == "anaphylaxis_low_hist" and rng.random() < 0.6)
        cv = scen in ("anaphylaxis_cv", "anaphylaxis_severe") or (
            scen == "anaphylaxis_low_hist" and not resp) or rng.random() < 0.3
        if resp:
            wheeze_p = np.maximum(wheeze_p, WHEEZE_SENS * np.clip(g * 1.3, 0, 1) *
                                  rng.uniform(0.5, 1.0))
            rr += rng.uniform(8, 20) * g
            spo2 -= rng.uniform(2, 8) * g
        else:
            rr += rng.uniform(1, 6) * g
        if cv:
            hr += rng.uniform(25, 60) * g
            pi *= 1 - rng.uniform(0.45, 0.85) * g
            spo2 -= rng.uniform(0, 3) * g
        else:
            hr += rng.uniform(5, 20) * g
            pi *= 1 - rng.uniform(0.0, 0.3) * g
        if scen == "anaphylaxis_severe" or (scen == "anaphylaxis_low_hist" and rng.random() < 0.5):
            tc = t1 + dur + rng.uniform(60, 600)
            if tc < T - 60:
                collapse_t.append(tc)
                if rng.random() < 0.5:
                    fall_t.append(tc)
    elif scen == "sleep":
        lying[:] = True
        act[:] = 0.005 + 0.005 * rng.random(T)
        hr -= rng.uniform(5, 12)
        rr -= rng.uniform(1, 3)
        if rng.random() < 0.5:  # obstructive sleep apnea: pauses + desaturation
            for s in rng.integers(300, T - 60, rng.integers(2, 10)):
                d = rng.integers(20, 45)
                apnea[s:s + d] = True
                spo2[s:s + d + 20] -= rng.uniform(2, 6)
                hr[s + d:s + d + 30] += rng.uniform(10, 25)
        wheeze_p += 2.0 / 3600 * (rng.random() < 0.3)  # snoring
    elif scen == "exercise":
        d = rng.uniform(480, 900)
        on = (t >= t0) & (t < t0 + d)
        a = rng.uniform(0.3, 0.9)
        act[on] = a + 0.1 * rng.random(on.sum())
        # HR/RR rise during and decay after (recovery while resting)
        g = ramp(t, t0, 90) * (t < t0 + d) + (t >= t0 + d) * np.exp(-(t - t0 - d) / rng.uniform(90, 300))
        hr += rng.uniform(60, 100) * g
        rr += rng.uniform(12, 25) * g
        pi *= 1 + rng.uniform(0.2, 1.0) * g
        hist *= 1 + 0.15 * g  # sweat on the sensor
        wheeze_p[on] = WHEEZE_FP_ACTIVE * 4
        if rng.random() < 0.15:  # exercise-induced bronchospasm afterwards
            wheeze_p[(t >= t0 + d) & (t < t0 + d + 600)] = WHEEZE_SENS * 0.5
    elif scen == "asthma":
        g = ramp(t, t0, rng.uniform(120, 900))
        wheeze_p = np.maximum(wheeze_p, WHEEZE_SENS * rng.uniform(0.5, 1.0) * g)
        rr += rng.uniform(8, 18) * g
        hr += rng.uniform(10, 30) * g
        spo2 -= rng.uniform(1, 5) * g
    elif scen == "allergy_mild":
        hist *= 1 + HIST_GAIN * rng.uniform(1.0, 4.0) * ramp(t, t0, rng.uniform(120, 600))
        hr += rng.uniform(3, 12) * ramp(t, t0 + 300, 300)
    elif scen == "vasovagal":
        # HR and perfusion drop, faint, then recovery lying down
        g = ramp(t, t0, 30) * np.exp(-np.clip(t - t0 - 90, 0, None) / 300)
        hr -= rng.uniform(15, 35) * g
        pi *= 1 - rng.uniform(0.4, 0.7) * g
        collapse_t.append(t0 + 60)
        if rng.random() < 0.4:
            fall_t.append(t0 + 60)
    elif scen == "fall":
        fall_t.append(t0)
        collapse_t.append(t0)
        hr += 15 * ramp(t, t0, 10) * np.exp(-np.clip(t - t0, 0, None) / 120)
        recover = t0 + rng.uniform(20, 300)
    elif scen == "panic":
        g = ramp(t, t0, 60) * np.exp(-np.clip(t - t0 - 600, 0, None) / 300)
        hr += rng.uniform(30, 60) * g
        rr += rng.uniform(10, 20) * g
        act[(t > t0) & (t < t0 + 600)] += 0.03
    elif scen == "hist_artifact":
        hist *= 1 + rng.uniform(1.0, 3.0) * ramp(t, t0, rng.uniform(60, 600))
        if rng.random() < 0.5:  # it's sweat during a hard session
            on = (t >= t0) & (t < t0 + 600)
            act[on] = 0.4
            hr += 60 * on
            rr += 15 * on

    # body motion detector output: collapse holds while lying
    collapsed = np.zeros(T, bool)
    fall = np.zeros(T, bool)
    for tc in collapse_t:
        if rng.random() < 0.9:  # detector sensitivity
            s = int(tc) + 15     # motionless 15 s before it latches
            collapsed[s:] = True
            lying[int(tc):] = True
            act[int(tc):] = np.minimum(act[int(tc):], 0.02)
    for tf in fall_t:
        if rng.random() < 0.85:
            s = int(tf) + 10
            e = s + int(rng.uniform(30, 900)) if scen == "fall" else T
            fall[s:e] = True
    if scen == "fall":  # gets up again
        collapsed[int(recover):] = False
    # false collapses (flopping onto a sofa) in a few recordings
    if rng.random() < 0.04:
        s = int(rng.integers(300, T - 200))
        collapsed[s + 15:s + int(rng.uniform(60, 400))] = True

    # wheeze detector: true episodes + false events, more when moving
    fp = np.where(act > 0.08, WHEEZE_FP_ACTIVE, WHEEZE_FP_REST)
    wheezing = rng.random(T) < np.clip(wheeze_p + fp, 0, 1)

    # sensor validity: leads-off spells, PPG lost while moving
    hr_valid = np.ones(T, bool)
    for s in rng.integers(0, T, rng.poisson(1.0)):
        hr_valid[s:s + rng.integers(10, 180)] = False
    ppg_valid = act < 0.25
    for s in rng.integers(0, T, rng.poisson(1.0)):
        ppg_valid[s:s + rng.integers(10, 120)] = False
    hist_ok = np.ones(T, bool)
    if rng.random() < 0.03:
        hist_ok[int(rng.integers(0, T)):] = False  # sensor failure

    # measurement noise of each detector
    hr_m = hr + NOISE * (rng.normal(0, 1.5, T) + rng.normal(0, 6, T) * (act > 0.08))
    rr_m = np.clip(rr + NOISE * rng.normal(0, 1.0, T), 2, 60)
    pi_m = np.clip(pi * np.exp(NOISE * rng.normal(0, 0.08, T)), 0.05, 20)
    spo2_m = np.clip(spo2 + NOISE * rng.normal(0, 0.8, T), 70, 100)
    hist_m = hist * np.exp(rng.normal(0, 0.04, T))
    rr_valid = hr_valid & ~apnea  # Bio-Z shares the electrodes with the ECG
    rr_m[apnea] = 0

    x = np.zeros((T, len(COLS)), np.float32)
    x[:, C["hr_valid"]] = hr_valid
    x[:, C["hr"]] = np.clip(hr_m, 25, 230)
    x[:, C["rr_valid"]] = rr_valid
    x[:, C["rr"]] = rr_m
    x[:, C["apnea"]] = apnea & hr_valid
    x[:, C["ppg_valid"]] = ppg_valid
    x[:, C["pi"]] = pi_m
    x[:, C["spo2"]] = spo2_m
    x[:, C["wheezing"]] = wheezing
    x[:, C["activity"]] = np.clip(act + 0.004 * rng.standard_normal(T), 0, 4)
    x[:, C["collapsed"]] = collapsed
    x[:, C["fall"]] = fall
    x[:, C["hist_ok"]] = hist_ok
    x[:, C["hist_nm"]] = hist_m
    return x, t0, t1


def main():
    global WHEEZE_SENS, WHEEZE_FP_REST, WHEEZE_FP_ACTIVE, NOISE, HIST_GAIN
    if len(sys.argv) > 4 and sys.argv[4] == "shift":
        WHEEZE_SENS, WHEEZE_FP_REST, WHEEZE_FP_ACTIVE = 0.40, 4.5 / 3600, 24.0 / 3600
        NOISE, HIST_GAIN = 1.5, 0.6
    out = sys.argv[1]
    n = int(sys.argv[2]) if len(sys.argv) > 2 else 2000
    rng = np.random.default_rng(int(sys.argv[3]) if len(sys.argv) > 3 else 7)
    names = list(SCENARIOS)
    p = np.array([SCENARIOS[s][0] for s in names])
    scen = rng.choice(len(names), n, p=p / p.sum())
    X = np.zeros((n, T, len(COLS)), np.float32)
    t0 = np.zeros(n)
    t1 = np.full(n, np.nan)
    for i in range(n):
        X[i], t0[i], t1[i] = simulate(rng, names[scen[i]])
    np.savez_compressed(out, X=X, t0=t0, t1=t1, scen=scen, names=np.array(names),
                        pos=np.array([SCENARIOS[s][1] for s in names]), cols=np.array(COLS))
    print(f"{n} recordings x {T} s -> {out}")


if __name__ == "__main__":
    main()
