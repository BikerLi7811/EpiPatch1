"""Train the wheeze CNN on frames produced by prepare_icbhi.py.

usage: python train.py <icbhi_frames.npz> <out_dir> [--epochs 30] [--width 16]

Splits: official ICBHI test split is held out; 15% of the train patients are
used as validation for threshold selection. Nothing from a test patient is
ever seen during training.
"""
import argparse
import json
import os

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as Fn
from sklearn.metrics import roc_auc_score

import features as F

T, M = F.N_FRAMES, F.N_MELS


class WheezeNet(nn.Module):
    """NHWC on device: H = time (40), W = mel (32), C = 1."""

    def __init__(self, w=16):
        super().__init__()
        self.c1, self.b1 = nn.Conv2d(1, w, 3, padding=1, bias=False), nn.BatchNorm2d(w)
        self.c2, self.b2 = nn.Conv2d(w, 2 * w, 3, padding=1, bias=False), nn.BatchNorm2d(2 * w)
        self.c3, self.b3 = nn.Conv2d(2 * w, 2 * w, 3, padding=1, bias=False), nn.BatchNorm2d(2 * w)
        self.drop = nn.Dropout(0.3)
        self.fc = nn.Linear(2 * w, 1)

    def forward(self, x):
        x = Fn.max_pool2d(Fn.relu(self.b1(self.c1(x))), 2)
        x = Fn.max_pool2d(Fn.relu(self.b2(self.c2(x))), 2)
        x = Fn.relu(self.b3(self.c3(x)))
        x = x.mean(dim=(2, 3))
        return self.fc(self.drop(x)).squeeze(1)


def make_windows(d, rec_ids, stride):
    """Returns (starts, labels, cycle_key) for windows fully inside each recording."""
    starts, labels, ckeys = [], [], []
    lab, cyc, offs = d["labels"], d["cycle"], d["rec_offsets"]
    for r in rec_ids:
        a, b = offs[r], offs[r + 1]
        s = np.arange(a, b - T + 1, stride)
        if len(s) == 0:
            continue
        idx = s[:, None] + np.arange(T)[None, :]
        l = lab[idx]
        f1, f0 = (l == 1).mean(1), (l == 0).mean(1)
        y = np.full(len(s), -1, np.int8)
        y[(f0 >= 0.8) & (f1 == 0)] = 0
        y[f1 >= 0.5] = 1
        c = cyc[s + T // 2]
        starts.append(s)
        labels.append(y)
        ckeys.append(np.where(c >= 0, r * 1000 + c, -1))
    return np.concatenate(starts), np.concatenate(labels), np.concatenate(ckeys)


def gather(feats, starts):
    return feats[starts[:, None] + np.arange(T)[None, :]]  # (B, T, M)


def augment(x, rng):
    """x: (B,T,M) raw log-mel. Simulates a different mic/coupling + ambient noise."""
    B = len(x)
    mel = np.arange(M)[None, :] / M
    # 1) random smooth EQ (mic frequency response / skin coupling), up to ~±6 dB
    eq = np.zeros((B, M), np.float32)
    for k in range(1, 4):
        eq += rng.uniform(-0.5, 0.5, (B, 1)) * np.cos(np.pi * k * mel + rng.uniform(0, np.pi, (B, 1)))
    eq += rng.uniform(-1.0, 1.0, (B, 1)) * (mel - 0.5)  # spectral tilt
    x = x + eq[:, None, :]
    # 2) additive stationary noise at random SNR (power domain)
    m = rng.random(B) < 0.5
    if m.any():
        p = np.exp(x[m])
        sig = p.mean(axis=(1, 2), keepdims=True)
        snr_db = rng.uniform(0, 30, (m.sum(), 1, 1))
        tilt = np.exp(rng.uniform(-3, 1, (m.sum(), 1, 1)) * mel[None])  # pink-ish..white
        noise = tilt / tilt.mean(axis=2, keepdims=True) * sig * 10 ** (-snr_db / 10)
        noise = noise * np.exp(rng.normal(0, 0.3, p.shape))  # non-flat over time
        x[m] = np.log(p + noise + F.LOG_EPS)
    # 3) small pitch shift by one mel band (wheeze frequency varies by age)
    for sh in (-1, 1):
        m = rng.random(B) < 0.15
        x[m] = np.roll(x[m], sh, axis=2)
        x[m, :, 0 if sh > 0 else -1] = x[m, :, 1 if sh > 0 else -2]
    # 4) time / freq masking
    for i in np.nonzero(rng.random(B) < 0.3)[0]:
        t0, w = rng.integers(0, T), rng.integers(1, 7)
        x[i, t0:t0 + w] = x[i].mean()
    for i in np.nonzero(rng.random(B) < 0.3)[0]:
        f0, w = rng.integers(0, M), rng.integers(1, 4)
        x[i, :, f0:f0 + w] = x[i].mean()
    return x


def to_tensor(x):
    return torch.from_numpy(F.normalize_window(x).astype(np.float32)).unsqueeze(1)


@torch.no_grad()
def predict(model, feats, starts, bs=2048):
    model.eval()
    out = []
    for i in range(0, len(starts), bs):
        out.append(torch.sigmoid(model(to_tensor(gather(feats, starts[i:i + bs])))).numpy())
    return np.concatenate(out)


def cycle_scores(p, ckeys, cycle_truth):
    """Breath-cycle decision = max window probability centred in that cycle."""
    v = ckeys >= 0
    keys, inv = np.unique(ckeys[v], return_inverse=True)
    score = np.full(len(keys), -np.inf)
    np.maximum.at(score, inv, p[v])
    truth = np.array([cycle_truth[k] for k in keys])
    return score, truth


def sens_spec(score, truth, thr):
    pred = score >= thr
    return pred[truth == 1].mean(), (~pred[truth == 0]).mean()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("npz")
    ap.add_argument("out")
    ap.add_argument("--epochs", type=int, default=30)
    ap.add_argument("--width", type=int, default=16)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--threads", type=int, default=min(16, os.cpu_count()))
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    torch.manual_seed(a.seed)
    torch.set_num_threads(a.threads)
    rng = np.random.default_rng(a.seed)

    d = dict(np.load(a.npz, allow_pickle=True))
    feats = d["feats"]
    split, patient = d["rec_split"], d["rec_patient"]
    cycle_truth = {}
    for r, cw in enumerate(d["cycle_wheeze"]):
        for c, w in enumerate(cw):
            cycle_truth[r * 1000 + c] = int(w)

    train_pat = np.unique(patient[split == "train"])
    val_pat = rng.choice(train_pat, int(round(0.15 * len(train_pat))), replace=False)
    is_val = np.isin(patient, val_pat)
    rec_tr = np.nonzero((split == "train") & ~is_val)[0]
    rec_va = np.nonzero((split == "train") & is_val)[0]
    rec_te = np.nonzero(split == "test")[0]

    s_tr, y_tr, _ = make_windows(d, rec_tr, stride=4)
    keep = y_tr >= 0
    s_tr, y_tr = s_tr[keep], y_tr[keep].astype(np.float32)
    s_va, y_va, c_va = make_windows(d, rec_va, stride=4)
    s_te, y_te, c_te = make_windows(d, rec_te, stride=4)
    print(f"train windows {len(s_tr)} (wheeze {y_tr.mean():.1%}), val recs {len(rec_va)}, test recs {len(rec_te)}")

    model = WheezeNet(a.width)
    n_par = sum(p.numel() for p in model.parameters())
    print(f"params {n_par}")
    opt = torch.optim.AdamW(model.parameters(), lr=2e-3, weight_decay=1e-3)
    bs = 256
    steps = a.epochs * (len(s_tr) // bs)
    sched = torch.optim.lr_scheduler.OneCycleLR(opt, 2e-3, total_steps=steps)
    pos_w = torch.tensor((1 - y_tr.mean()) / y_tr.mean())
    best, best_state = -1, None
    for ep in range(a.epochs):
        model.train()
        perm = rng.permutation(len(s_tr))
        tot = 0.0
        for i in range(len(s_tr) // bs):
            b = perm[i * bs:(i + 1) * bs]
            jitter = rng.integers(-2, 3, len(b))
            st = np.clip(s_tr[b] + jitter, 0, len(feats) - T)
            x = to_tensor(augment(gather(feats, st).copy(), rng))
            loss = Fn.binary_cross_entropy_with_logits(model(x), torch.from_numpy(y_tr[b]), pos_weight=pos_w)
            opt.zero_grad()
            loss.backward()
            opt.step()
            sched.step()
            tot += loss.item()
        p_va = predict(model, feats, s_va)
        m = y_va >= 0
        auc_w = roc_auc_score(y_va[m], p_va[m])
        sc, tr = cycle_scores(p_va, c_va, cycle_truth)
        auc_c = roc_auc_score(tr, sc)
        print(f"ep {ep:2d} loss {tot / (len(s_tr) // bs):.4f} val win-AUC {auc_w:.3f} cycle-AUC {auc_c:.3f}", flush=True)
        if auc_c > best:
            best, best_state = auc_c, {k: v.clone() for k, v in model.state_dict().items()}

    model.load_state_dict(best_state)
    # threshold on validation: operating point with >= 90% cycle specificity
    p_va = predict(model, feats, s_va)
    sc, tr = cycle_scores(p_va, c_va, cycle_truth)
    cand = np.unique(sc)
    spec = np.array([sens_spec(sc, tr, t)[1] for t in cand])
    thr = float(cand[np.argmax(spec >= 0.90)])

    p_te = predict(model, feats, s_te)
    m = y_te >= 0
    sc_te, tr_te = cycle_scores(p_te, c_te, cycle_truth)
    se, sp = sens_spec(sc_te, tr_te, thr)
    res = {
        "test_window_auc": float(roc_auc_score(y_te[m], p_te[m])),
        "test_cycle_auc": float(roc_auc_score(tr_te, sc_te)),
        "threshold": thr,
        "test_cycle_sensitivity": float(se),
        "test_cycle_specificity": float(sp),
        "test_cycles": int(len(tr_te)),
        "test_wheeze_cycles": int(tr_te.sum()),
        "width": a.width,
        "params": n_par,
    }
    # per-device breakdown (how robust is it across stethoscopes/mics?)
    dev = d["rec_device"]
    for dv in np.unique(dev[rec_te]):
        mk = np.isin(c_te // 1000, np.nonzero(dev == dv)[0])
        sc_d, tr_d = cycle_scores(p_te[mk], c_te[mk], cycle_truth)
        if 0 < tr_d.sum() < len(tr_d):
            res[f"test_cycle_auc_{dv}"] = float(roc_auc_score(tr_d, sc_d))
    print(json.dumps(res, indent=2))
    torch.save(model.state_dict(), os.path.join(a.out, "wheezenet.pt"))
    json.dump(res, open(os.path.join(a.out, "metrics.json"), "w"), indent=2)
    np.savez(os.path.join(a.out, "split.npz"), rec_tr=rec_tr, rec_va=rec_va, rec_te=rec_te)


if __name__ == "__main__":
    main()
