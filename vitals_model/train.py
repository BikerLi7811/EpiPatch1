"""Trains anaphylaxis classifiers on the simulated vitals and compares them
with the firmware's rule engine (replay.exe output).

Features are causal: each second only sees the past, as on the device.
Split by recording (each recording is a different simulated wearer):
60 % train, 20 % validation (threshold), 20 % test (all numbers reported).

usage: python train.py <data.npz> <rules.bin> <results.json> [<shift.npz> <shift_rules.bin>]
  The optional shifted set (sim.py ... shift) is a robustness test only:
  the models and thresholds are never fitted on it.
"""
import json
import sys
import time

import numpy as np
from sklearn.ensemble import HistGradientBoostingClassifier
from sklearn.inspection import permutation_importance
from sklearn.linear_model import LogisticRegression
from sklearn.metrics import average_precision_score, roc_auc_score, roc_curve
from sklearn.pipeline import make_pipeline
from sklearn.preprocessing import StandardScaler
from sklearn.tree import DecisionTreeClassifier

from sim import C, T

ALARM_HOLD_S = 10  # model must stay above threshold this long (debounce)

FEATURES = [
    ("hr", "Heart rate [/min], 30 s average"),
    ("hr_d", "Heart rate above resting baseline"),
    ("hr_trend", "Heart-rate change over 5 min"),
    ("rr", "Breathing rate [/min], 30 s average"),
    ("rr_d", "Breathing rate above resting baseline"),
    ("apnea60", "Apnea seconds in the last minute"),
    ("pi_ratio", "Perfusion index / baseline"),
    ("pi_trend", "Perfusion-ratio change over 5 min"),
    ("spo2", "SpO2 estimate, 30 s average"),
    ("wz60", "Wheeze detections, last 1 min"),
    ("wz300", "Wheeze detections, last 5 min"),
    ("act60", "Activity [g], 1 min average"),
    ("still_min", "Minutes since last movement (capped 15)"),
    ("collapsed", "Collapse flag"),
    ("down300", "Collapse or fall in the last 5 min"),
    ("hist_ratio", "Histamine / baseline"),
    ("hist_slope", "Histamine-ratio change over 5 min"),
    ("hr_ok", "ECG valid"),
    ("ppg_ok", "PPG valid"),
    ("hist_ok", "Histamine sensor valid"),
]
F = {n: i for i, (n, _) in enumerate(FEATURES)}


def features(X):
    """X [n, T, cols] -> [n, T, features], computed causally, vectorised over recordings."""
    n = X.shape[0]
    out = np.zeros((n, T, len(FEATURES)), np.float32)
    a30 = 1 / 30
    hr = X[:, 0, C["hr"]].copy(); rr = X[:, 0, C["rr"]].copy()
    pi = X[:, 0, C["pi"]].copy(); sp = X[:, 0, C["spo2"]].copy()
    hr_b = hr.copy(); rr_b = rr.copy(); pi_b = pi.copy()
    rest_s = np.zeros(n)
    hist_b = X[:, 0, C["hist_nm"]].copy(); hist = hist_b.copy()
    wz = X[:, :, C["wheezing"]].cumsum(1)
    ap = X[:, :, C["apnea"]].cumsum(1)
    act = X[:, :, C["activity"]].cumsum(1)
    down = np.maximum(X[:, :, C["collapsed"]], X[:, :, C["fall"]])
    last_down = np.full(n, -1e9)
    last_move = np.zeros(n)
    hist_hist = np.ones((n, T), np.float32)
    hr_hist = np.zeros((n, T), np.float32); pi_hist = np.ones((n, T), np.float32)

    def win(cs, t, w):
        return cs[:, t] - (cs[:, t - w] if t >= w else 0)

    for t in range(T):
        x = X[:, t]
        hv = x[:, C["hr_valid"]] > 0.5; rv = x[:, C["rr_valid"]] > 0.5
        pv = x[:, C["ppg_valid"]] > 0.5; ho = x[:, C["hist_ok"]] > 0.5
        hr = np.where(hv, hr + a30 * (x[:, C["hr"]] - hr), hr)
        rr = np.where(rv, rr + a30 * (x[:, C["rr"]] - rr), rr)
        pi = np.where(pv, pi + a30 * (x[:, C["pi"]] - pi), pi)
        sp = np.where(pv, sp + a30 * (x[:, C["spo2"]] - sp), sp)
        if t % 10 == 0:  # histamine sampled every 10 s, like the firmware
            hist = np.where(ho, x[:, C["hist_nm"]], hist)
            warm = t < 300
            calm = ho & (hist < 1.3 * hist_b)
            ah = np.where(warm, 0.2, 10 / 1800)
            hist_b = np.where(calm, hist_b + ah * (hist - hist_b), hist_b)
        # resting baselines: still, and not far from the current baseline
        a_now = x[:, C["activity"]]
        resting = a_now < 0.05
        rest_s = np.where(resting, rest_s + 1, rest_s)
        tau = np.where(rest_s < 120, 30.0, 600.0)
        ok = resting & ((rest_s < 120) | (np.abs(hr - hr_b) < 20))
        hr_b = np.where(ok & hv, hr_b + (hr - hr_b) / tau, hr_b)
        rr_b = np.where(ok & rv, rr_b + (rr - rr_b) / tau, rr_b)
        okp = resting & ((rest_s < 120) | (pi > 0.6 * pi_b))
        pi_b = np.where(okp & pv, pi_b + (pi - pi_b) / tau, pi_b)
        last_move = np.where(a_now > 0.15, t, last_move)
        last_down = np.where(down[:, t] > 0.5, t, last_down)

        hratio = np.where(ho, hist / np.maximum(hist_b, 1e-3), 1.0)
        pratio = np.where(pv, pi / np.maximum(pi_b, 1e-3), 1.0)
        hist_hist[:, t] = hratio; hr_hist[:, t] = hr; pi_hist[:, t] = pratio
        b = max(t - 300, 0)
        o = out[:, t]
        o[:, F["hr"]] = hr
        o[:, F["hr_d"]] = hr - hr_b
        o[:, F["hr_trend"]] = hr - hr_hist[:, b]
        o[:, F["rr"]] = rr
        o[:, F["rr_d"]] = rr - rr_b
        o[:, F["apnea60"]] = win(ap, t, 60)
        o[:, F["pi_ratio"]] = pratio
        o[:, F["pi_trend"]] = pratio - pi_hist[:, b]
        o[:, F["spo2"]] = sp
        o[:, F["wz60"]] = win(wz, t, 60)
        o[:, F["wz300"]] = win(wz, t, 300)
        o[:, F["act60"]] = win(act, t, 60) / min(t + 1, 60)
        o[:, F["still_min"]] = np.minimum(t - last_move, 900) / 60
        o[:, F["collapsed"]] = x[:, C["collapsed"]]
        o[:, F["down300"]] = (t - last_down) <= 300
        o[:, F["hist_ratio"]] = hratio
        o[:, F["hist_slope"]] = hratio - hist_hist[:, b]
        o[:, F["hr_ok"]] = hv
        o[:, F["ppg_ok"]] = pv
        o[:, F["hist_ok"]] = ho
    return out


def labels(t0, t1, n):
    """1 from organ involvement on; -1 (ignored) between exposure and involvement."""
    y = np.zeros((n, T), np.int8)
    for i in range(n):
        if not np.isnan(t1[i]):
            y[i, int(t0[i]):int(t1[i])] = -1
            y[i, int(t1[i]):] = 1
    return y


def alarms(p, thr):
    """Debounced alarm: p >= thr for ALARM_HOLD_S consecutive seconds."""
    above = p >= thr
    run = np.zeros(p.shape[0], int)
    out = np.zeros_like(above)
    for t in range(p.shape[1]):
        run = np.where(above[:, t], run + 1, 0)
        out[:, t] = run >= ALARM_HOLD_S
    return out


def events(alarm, t0, t1, is_pos):
    """Per recording: false alarm (any alarm before t0, or at all in a negative),
    detected (positive with an alarm after t0), and time to detect from t1."""
    first_after = np.full(len(t0), np.nan)
    fa = np.zeros(len(t0), bool)
    for i in range(len(t0)):
        a0 = int(t0[i])
        if is_pos[i]:
            fa[i] = alarm[i, :a0].any()
            idx = np.flatnonzero(alarm[i, a0:])
            if idx.size:
                first_after[i] = a0 + idx[0] - t1[i]
        else:
            fa[i] = alarm[i].any()
    return fa, first_after


def summary(fa, ttd, is_pos):
    neg = ~is_pos
    det = ~np.isnan(ttd) & is_pos
    return {
        "sensitivity": float(det[is_pos].mean()),
        "false_alarm_rate": float(fa[neg].mean()),
        "ttd_median_s": float(np.nanmedian(ttd[det])) if det.any() else None,
        "ttd_p90_s": float(np.nanpercentile(ttd[det], 90)) if det.any() else None,
        "early_pct": float((ttd[det] < 0).mean()) if det.any() else None,
    }


def sweep(p, t0, t1, is_pos, thrs):
    rows = []
    for thr in thrs:
        fa, ttd = events(alarms(p, thr), t0, t1, is_pos)
        rows.append({"thr": float(thr), **summary(fa, ttd, is_pos)})
    return rows


def main():
    data, rules_path, out_path = sys.argv[1:4]
    d = np.load(data)
    X, t0, t1, scen = d["X"], d["t0"], d["t1"], d["scen"]
    names = [str(s) for s in d["names"]]
    pos_s = d["pos"]
    n = len(scen)
    is_pos = pos_s[scen]
    rules = np.fromfile(rules_path, np.uint8).reshape(n, T)

    t_start = time.time()
    Fx = features(X)
    y = labels(t0, t1, n)
    print(f"features {Fx.shape} in {time.time() - t_start:.0f} s")

    idx = np.arange(n)
    tr, va, te = idx[: int(0.6 * n)], idx[int(0.6 * n): int(0.8 * n)], idx[int(0.8 * n):]

    def rows(sel, step):
        f = Fx[sel][:, ::step].reshape(-1, Fx.shape[2])
        l = y[sel][:, ::step].reshape(-1)
        k = l >= 0
        return f[k], l[k]

    Xtr, ytr = rows(tr, 5)
    Xte_s, yte_s = rows(te, 5)
    print(f"train rows {len(ytr)} ({ytr.mean():.1%} positive)")

    models = {
        "Logistic regression": make_pipeline(StandardScaler(), LogisticRegression(C=1.0, max_iter=2000,
                                                                                  class_weight="balanced")),
        "Decision tree (depth 6)": DecisionTreeClassifier(max_depth=6, min_samples_leaf=200,
                                                          class_weight="balanced", random_state=0),
        "Gradient-boosted trees": HistGradientBoostingClassifier(max_depth=4, max_iter=150,
                                                                 learning_rate=0.1, random_state=0),
    }
    res = {"features": FEATURES, "scenarios": names, "pos": pos_s.tolist(), "models": {},
           "split": {"train": len(tr), "val": len(va), "test": len(te)}, "T": T}

    # dataset table
    comp = []
    for k, nm in enumerate(names):
        m = scen == k
        comp.append({"scenario": nm, "anaphylaxis": bool(pos_s[k]), "n": int(m.sum()),
                     "train": int(m[tr].sum()), "val": int(m[va].sum()), "test": int(m[te].sum())})
    res["dataset"] = comp

    # rules engine on val / test
    rule_alarm = (rules & 1).astype(bool)
    def rule_eval(sel):
        fa, ttd = events(rule_alarm[sel], t0[sel], t1[sel], is_pos[sel])
        return fa, ttd
    fa_rv, ttd_rv = rule_eval(va)
    rules_val = summary(fa_rv, ttd_rv, is_pos[va])
    fa_rt, ttd_rt = rule_eval(te)
    res["rules"] = {"val": rules_val, "test": summary(fa_rt, ttd_rt, is_pos[te]),
                    "ttd": [float(v) for v in ttd_rt[~np.isnan(ttd_rt)]]}
    yt_sec = y[te].reshape(-1)
    k = yt_sec >= 0
    rs = rule_alarm[te].reshape(-1)[k]
    yk = yt_sec[k]
    res["rules"]["per_second"] = {"tpr": float(rs[yk == 1].mean()), "fpr": float(rs[yk == 0].mean())}
    print("rules test", res["rules"]["test"])

    thrs = np.concatenate([np.linspace(0.05, 0.95, 37), [0.97, 0.98, 0.99, 0.995]])
    per_scen = {"Rules engine": scen_table(fa_rt, ttd_rt, scen[te], names, pos_s)}
    examples_p = {}
    for name, mdl in models.items():
        t_fit = time.time()
        mdl.fit(Xtr, ytr)
        print(f"{name}: fit {time.time() - t_fit:.0f} s")
        def prob(sel):
            f = Fx[sel].reshape(-1, Fx.shape[2])
            return mdl.predict_proba(f)[:, 1].reshape(len(sel), T)
        pv, pt = prob(va), prob(te)
        sw_val = sweep(pv, t0[va], t1[va], is_pos[va], thrs)
        # operating point: most sensitive threshold whose validation false-alarm
        # rate is no worse than the rule engine's
        ok = [r for r in sw_val if r["false_alarm_rate"] <= rules_val["false_alarm_rate"]]
        best = max(ok, key=lambda r: (r["sensitivity"], r["thr"])) if ok else sw_val[-1]
        thr = best["thr"]
        fa, ttd = events(alarms(pt, thr), t0[te], t1[te], is_pos[te])
        test = summary(fa, ttd, is_pos[te])
        sw_test = sweep(pt, t0[te], t1[te], is_pos[te], thrs)
        ps = pt.reshape(-1)[k]
        fpr, tpr, _ = roc_curve(yk, ps)
        sel = np.unique(np.linspace(0, len(fpr) - 1, 200).astype(int))
        entry = {
            "thr": thr, "test": test, "val_at_thr": best,
            "auc": float(roc_auc_score(yk, ps)), "ap": float(average_precision_score(yk, ps)),
            "roc": {"fpr": fpr[sel].round(5).tolist(), "tpr": tpr[sel].round(5).tolist()},
            "sweep_test": sw_test,
            "ttd": [float(v) for v in ttd[~np.isnan(ttd)]],
            "size": model_size(mdl),
        }
        if name.startswith("Gradient"):
            pi_ = permutation_importance(mdl, Xte_s[::4], yte_s[::4], scoring="roc_auc",
                                         n_repeats=3, random_state=0)
            entry["importance"] = pi_.importances_mean.round(4).tolist()
        if name.startswith("Logistic"):
            entry["coef"] = mdl[-1].coef_[0].round(4).tolist()
        res["models"][name] = entry
        per_scen[name] = scen_table(fa, ttd, scen[te], names, pos_s)
        examples_p[name] = pt
        print(name, f"AUC {entry['auc']:.3f}", test)
    res["per_scenario"] = per_scen

    if len(sys.argv) > 5:
        ds = np.load(sys.argv[4])
        Xs, s0, s1, ss = ds["X"], ds["t0"], ds["t1"], ds["scen"]
        sp = pos_s[ss]
        rs_ = (np.fromfile(sys.argv[5], np.uint8).reshape(len(ss), T) & 1).astype(bool)
        fa_s, ttd_s = events(rs_, s0, s1, sp)
        shift = {"n": len(ss), "Rules engine": summary(fa_s, ttd_s, sp)}
        Fs = features(Xs).reshape(-1, Fx.shape[2])
        for name, mdl in models.items():
            p_s = mdl.predict_proba(Fs)[:, 1].reshape(len(ss), T)
            fa_m, ttd_m = events(alarms(p_s, res["models"][name]["thr"]), s0, s1, sp)
            shift[name] = summary(fa_m, ttd_m, sp)
        res["shift"] = shift
        print("shift", json.dumps(shift, indent=1))

    # example timelines from the test set, 10 s resolution
    gb = examples_p["Gradient-boosted trees"]
    ex = []
    want = ["anaphylaxis_cv", "anaphylaxis_low_hist", "allergy_mild", "vasovagal", "exercise"]
    for w in want:
        k_s = names.index(w)
        cand = [j for j, i in enumerate(te) if scen[i] == k_s]
        j = cand[0]
        for c in cand:  # prefer a case where rules and model disagree
            i = te[c]
            ra = rule_alarm[i, int(t0[i]):].any() or rule_alarm[i, :int(t0[i])].any()
            ma = alarms(gb[c:c + 1], res["models"]["Gradient-boosted trees"]["thr"])[0].any()
            if ra != ma:
                j = c
                break
        i = te[j]
        s = slice(0, T, 10)
        ex.append({
            "scenario": w, "t0": float(t0[i]), "t1": None if np.isnan(t1[i]) else float(t1[i]),
            "hr": Fx[i, s, F["hr"]].round(1).tolist(),
            "rr": Fx[i, s, F["rr"]].round(1).tolist(),
            "pi_ratio": Fx[i, s, F["pi_ratio"]].round(3).tolist(),
            "hist_ratio": Fx[i, s, F["hist_ratio"]].round(3).tolist(),
            "wz60": Fx[i, s, F["wz60"]].tolist(),
            "act": X[i, s, C["activity"]].round(3).tolist(),
            "down": np.maximum(X[i, s, C["collapsed"]], X[i, s, C["fall"]]).tolist(),
            "p": gb[j, s].round(3).tolist(),
            "rule": rule_alarm[i, s].astype(int).tolist(),
        })
    res["examples"] = ex
    with open(out_path, "w") as f:
        json.dump(res, f)
    print("->", out_path)


def scen_table(fa, ttd, scen, names, pos_s):
    out = []
    for k, nm in enumerate(names):
        m = scen == k
        if not m.any():
            continue
        if pos_s[k]:
            det = ~np.isnan(ttd[m])
            out.append({"scenario": nm, "n": int(m.sum()), "detected": float(det.mean()),
                        "ttd_median_s": float(np.median(ttd[m][det])) if det.any() else None,
                        "false_alarm": float(fa[m].mean())})
        else:
            out.append({"scenario": nm, "n": int(m.sum()), "false_alarm": float(fa[m].mean())})
    return out


def model_size(m):
    """Parameters and approximate flash for an on-device port."""
    if hasattr(m, "steps"):
        p = m[-1].coef_.size + 1 + 2 * m[-1].coef_.size  # weights, bias, scaler
        return {"params": int(p), "flash_bytes": int(4 * p), "ops": "20 MAC + sigmoid"}
    if isinstance(m, DecisionTreeClassifier):
        nodes = m.tree_.node_count
        return {"params": int(nodes), "flash_bytes": int(8 * nodes), "ops": f"<= {m.get_depth()} compares"}
    nodes = sum(p.nodes.shape[0] for it in m._predictors for p in it)
    return {"params": int(nodes), "flash_bytes": int(8 * nodes),
            "ops": f"{m.n_iter_} trees x <= 4 compares"}


if __name__ == "__main__":
    main()
