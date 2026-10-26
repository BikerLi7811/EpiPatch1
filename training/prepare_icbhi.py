"""Turn the ICBHI 2017 database into log-mel frames + per-frame wheeze labels.

usage: python prepare_icbhi.py <ICBHI_final_database dir> <ICBHI_challenge_train_test.txt> <out.npz>

ICBHI annotations are per respiratory cycle: "start end crackle wheeze".
A frame gets label 1 if it lies in a cycle with wheeze=1, 0 if in a cycle with
wheeze=0 (crackles count as "not wheezing"), -1 if between annotated cycles.
"""
import os
import sys
from math import gcd
from multiprocessing import Pool

import numpy as np
from scipy.io import wavfile
from scipy.signal import resample_poly

import features as F


def load_wav(path):
    sr, x = wavfile.read(path)
    if x.ndim > 1:
        x = x[:, 0]
    if np.issubdtype(x.dtype, np.integer):
        x = x.astype(np.float32) / float(np.iinfo(x.dtype).max)
    x = x.astype(np.float32)
    if sr != F.FS:
        g = gcd(sr, F.FS)
        x = resample_poly(x, F.FS // g, sr // g).astype(np.float32)
    return x


def process(args):
    root, name = args
    x = load_wav(os.path.join(root, name + ".wav"))
    feats = F.logmel_frames(x)
    n = len(feats)
    t = (np.arange(n) * F.HOP + F.N_FFT / 2) / F.FS
    lab = np.full(n, -1, np.int8)
    cyc = np.full(n, -1, np.int32)
    cycles = np.loadtxt(os.path.join(root, name + ".txt"), ndmin=2)
    for ci, (s, e, _crackle, wheeze) in enumerate(cycles):
        m = (t >= s) & (t < e)
        lab[m] = int(wheeze)
        cyc[m] = ci
    return name, feats, lab, cyc, cycles[:, 3].astype(np.int8)


def main(root, split_file, out):
    split = dict(l.split() for l in open(split_file) if l.strip())
    # The split file and the wav names disagree on the device suffix for a few
    # recordings (e.g. 226_1b1_Pl_sc_LittC2SE vs ..._Meditron), so fall back to
    # matching patient_recording_location_mode.
    by_stem = {k.rsplit("_", 1)[0]: v for k, v in split.items()}
    names = sorted(f[:-4] for f in os.listdir(root) if f.endswith(".wav"))
    with Pool() as p:
        res = p.map(process, [(root, n) for n in names], chunksize=4)
    feats, labs, cycs, rec, cyc_lab = [], [], [], [], []
    meta = []
    for i, (name, f, l, c, cl) in enumerate(res):
        feats.append(f)
        labs.append(l)
        cycs.append(c)
        rec.append(np.full(len(f), i, np.int32))
        cyc_lab.append(cl)
        sp = split.get(name) or by_stem[name.rsplit("_", 1)[0]]
        meta.append((name, sp, name.split("_")[-1], int(name.split("_")[0])))
    offs = np.cumsum([0] + [len(f) for f in feats])
    np.savez_compressed(
        out,
        feats=np.concatenate(feats),
        labels=np.concatenate(labs),
        cycle=np.concatenate(cycs),
        rec=np.concatenate(rec),
        rec_offsets=offs,
        rec_name=np.array([m[0] for m in meta]),
        rec_split=np.array([m[1] for m in meta]),
        rec_device=np.array([m[2] for m in meta]),
        rec_patient=np.array([m[3] for m in meta]),
        cycle_wheeze=np.array(cyc_lab, dtype=object),
    )
    lab = np.concatenate(labs)
    print(f"{len(names)} recordings, {len(lab)} frames "
          f"({len(lab) * F.HOP / F.FS / 3600:.2f} h), wheeze frames {np.mean(lab == 1):.1%}, "
          f"normal {np.mean(lab == 0):.1%}")


if __name__ == "__main__":
    main(*sys.argv[1:4])
