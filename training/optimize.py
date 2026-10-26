"""Energy/accuracy sweep: find the cheapest on-device configuration that keeps accuracy.

usage: python optimize.py <icbhi_frames.npz> <out.json> <run_dir> [<run_dir> ...]

Knobs swept (all of them change CPU work on the nRF54L15):
  * model width (each run_dir is one trained width)
  * inference stride: run the CNN every 4/8/12/16 frames (64..256 ms)
  * tonality gate: skip the CNN unless >= K frames in the window have peak
    prominence >= T (wheezes are tonal). Gated windows count as p = 0.

Selection uses VALIDATION only: cheapest config whose val breath-cycle AUC is
within 0.01 of the best and whose val sensitivity (at its own >= 90 %-specificity
threshold) is within 0.03 of the best. The test split is only reported.
"""
import json
import os
import sys

import numpy as np
import torch
from sklearn.metrics import roc_auc_score

import features as F
from train import WheezeNet, cycle_scores, make_windows, predict, sens_spec

T, M = F.N_FRAMES, F.N_MELS


def macs(w):
    return (T * M * 9 * w + (T // 2) * (M // 2) * 9 * w * 2 * w
            + (T // 4) * (M // 4) * 9 * (2 * w) ** 2 + 2 * w)


def window_gate_stat(ton, starts, thr):
    """# frames in each window whose tonality >= thr."""
    hit = (ton >= thr).astype(np.int32)
    c = np.concatenate([[0], np.cumsum(hit)])
    return c[starts + T] - c[starts]


def evaluate(p, ckeys, truth, thr=None):
    sc, tr = cycle_scores(p, ckeys, truth)
    auc = roc_auc_score(tr, sc)
    if thr is None:
        cand = np.unique(sc)
        spec = np.array([sens_spec(sc, tr, t)[1] for t in cand])
        thr = float(cand[np.argmax(spec >= 0.90)])
    se, sp = sens_spec(sc, tr, thr)
    return auc, float(se), float(sp), thr


def main(npz, out, *runs):
    d = dict(np.load(npz, allow_pickle=True))
    feats = d["feats"]
    ton = F.tonality(feats)
    truth = {}
    for r, cw in enumerate(d["cycle_wheeze"]):
        for c, w in enumerate(cw):
            truth[r * 1000 + c] = int(w)
    sp = np.load(os.path.join(runs[0], "split.npz"))
    s_va, _, c_va = make_windows(d, sp["rec_va"], stride=4)
    s_te, _, c_te = make_windows(d, sp["rec_te"], stride=4)
    # window start relative to its recording (strides subsample these)
    local_va = s_va - d["rec_offsets"][d["rec"][s_va]]
    local_te = s_te - d["rec_offsets"][d["rec"][s_te]]

    # Gate thresholds from the tonality distribution of all training frames
    tr_frames = np.concatenate([np.arange(d["rec_offsets"][r], d["rec_offsets"][r + 1])
                                for r in sp["rec_tr"]])
    t_grid = [None] + [float(np.percentile(ton[tr_frames], q)) for q in (50, 70, 80, 90, 95)]
    k_grid = (1, 3, 6)

    rows = []
    for run in runs:
        width = json.load(open(os.path.join(run, "metrics.json")))["width"]
        net = WheezeNet(width)
        net.load_state_dict(torch.load(os.path.join(run, "wheezenet.pt"), weights_only=True))
        p_va_full = predict(net, feats, s_va)
        p_te_full = predict(net, feats, s_te)
        for stride in (4, 8, 12, 16):
            k_va = local_va % stride == 0
            k_te = local_te % stride == 0
            for tg in t_grid:
                for kg in (k_grid if tg is not None else (0,)):
                    pv, pt = p_va_full[k_va].copy(), p_te_full[k_te].copy()
                    pass_v = pass_t = None
                    if tg is not None:
                        pass_v = window_gate_stat(ton, s_va[k_va], tg) >= kg
                        pass_t = window_gate_stat(ton, s_te[k_te], tg) >= kg
                        pv[~pass_v] = 0.0
                        pt[~pass_t] = 0.0
                    auc_v, se_v, sp_v, thr = evaluate(pv, c_va[k_va], truth)
                    auc_t, se_t, sp_t, _ = evaluate(pt, c_te[k_te], truth, thr)
                    duty = 1.0 if pass_t is None else float(pass_t.mean())
                    inf_per_s = F.FS / (F.HOP * stride)
                    rows.append(dict(
                        width=width, stride_frames=stride, stride_ms=stride * F.HOP * 1000 // F.FS,
                        gate_thr=tg, gate_k=kg, cnn_pass_rate=duty,
                        mmac_per_s=macs(width) * inf_per_s * duty / 1e6,
                        threshold=thr, val_auc=auc_v, val_sens=se_v, val_spec=sp_v,
                        test_auc=auc_t, test_sens=se_t, test_spec=sp_t, run=run))
            print(f"width {width} stride {stride} done", flush=True)

    best_auc = max(r["val_auc"] for r in rows)
    best_se = max(r["val_sens"] for r in rows)
    ok = [r for r in rows if r["val_auc"] >= best_auc - 0.01 and r["val_sens"] >= best_se - 0.03]
    pick = min(ok, key=lambda r: r["mmac_per_s"])
    base = max((r for r in rows if r["stride_frames"] == 4 and r["gate_thr"] is None),
               key=lambda r: (r["width"], r["val_auc"]))  # original firmware config
    json.dump({"pick": pick, "baseline": base, "all": rows}, open(out, "w"), indent=1)

    def show(tag, r):
        print(f"{tag}: width {r['width']}, every {r['stride_ms']} ms, gate "
              f"{'off' if r['gate_thr'] is None else f'T={r['gate_thr']:.2f} K={r['gate_k']}'}"
              f" | CNN runs {r['cnn_pass_rate']:.0%} | {r['mmac_per_s']:.1f} MMAC/s | "
              f"test cycle AUC {r['test_auc']:.3f} sens {r['test_sens']:.2f} spec {r['test_spec']:.2f}")
    show("baseline", base)
    show("picked  ", pick)
    print(f"CNN compute reduced {base['mmac_per_s'] / pick['mmac_per_s']:.1f}x")


if __name__ == "__main__":
    main(*sys.argv[1:])
