"""Fold BN, int8-quantize (CMSIS-NN / TFLite scheme), and emit C sources.

usage: python export.py <icbhi_frames.npz> <run_dir|optimize.json> <firmware/src/generated> <host_test/vectors>

Given optimize.json, exports its picked model together with the picked inference
stride, tonality gate and the threshold re-tuned for that configuration.

Weights: symmetric per-output-channel int8. Activations: asymmetric int8,
ranges from calibration on training windows (incl. augmented ones).
Emits C++17/20 sources (namespaced constexpr constants + const tables) and
test vectors so host_test can check the C++ pipeline against Python.
"""
import json
import math
import os
import sys

import numpy as np
import torch

import features as F
from train import WheezeNet, augment, gather, make_windows

T, M = F.N_FRAMES, F.N_MELS


def fold(conv, bn):
    w = conv.weight.detach().numpy()
    g = (bn.weight / torch.sqrt(bn.running_var + bn.eps)).detach().numpy()
    b = (bn.bias - bn.running_mean * bn.weight / torch.sqrt(bn.running_var + bn.eps)).detach().numpy()
    return w * g[:, None, None, None], b


def qmult(real):
    """TFLite QuantizeMultiplier: real = q * 2^(shift-31), q in [2^30, 2^31)."""
    if real == 0:
        return 0, 0
    m, e = math.frexp(real)
    q = int(round(m * (1 << 31)))
    if q == 1 << 31:
        q //= 2
        e += 1
    return q, e


def act_q(lo, hi):
    lo, hi = min(lo, 0.0), max(hi, 0.0)
    s = (hi - lo) / 255.0
    zp = int(round(-128 - lo / s))
    return float(s), int(np.clip(zp, -128, 127))


def carr(name, arr, ctype, per_line=16):
    flat = np.asarray(arr).ravel()
    body = ",\n".join("    " + ", ".join(str(int(v)) if "int" in ctype else f"{v:.9e}f"
                                         for v in flat[i:i + per_line])
                      for i in range(0, len(flat), per_line))
    return f"alignas(4) const {ctype} {name}[{len(flat)}] = {{\n{body}\n}};\n"


def pcm_vectors(fs_mic, vec_dir, rng, lfilter):
    t = np.arange(fs_mic * 2 // 256 * 256) / fs_mic  # whole firmware DMA chunks
    pcm = (0.3 * np.sin(2 * np.pi * 450 * t) + 0.2 * np.sin(2 * np.pi * 0.39 * fs_mic * t)
           + 0.05 * rng.standard_normal(len(t)))
    pcm = np.clip(np.round(pcm * 32767), -32768, 32767).astype(np.int16)
    tag = f"pcm_{fs_mic // 1000}k"
    pcm.tofile(os.path.join(vec_dir, tag + ".s16"))
    ref = lfilter(F.decim_fir(fs_mic).astype(np.float64), 1.0, pcm / 32768.0)[::fs_mic // F.FS]
    F.logmel_frames(ref.astype(np.float32)).tofile(os.path.join(vec_dir, tag + "_logmel.f32"))
    F.tonality(F.logmel_frames(ref.astype(np.float32))).tofile(os.path.join(vec_dir, tag + "_tonality.f32"))


def main(npz, run, gen_dir, vec_dir):
    os.makedirs(gen_dir, exist_ok=True)
    os.makedirs(vec_dir, exist_ok=True)
    if run.endswith(".json"):
        pick = json.load(open(run))["pick"]
        run = pick["run"]
    else:  # no optimisation: original every-64-ms, ungated behaviour
        pick = {"stride_frames": 4, "gate_thr": None, "gate_k": 0,
                "threshold": json.load(open(os.path.join(run, "metrics.json")))["threshold"]}
    metrics = json.load(open(os.path.join(run, "metrics.json")))
    width = metrics["width"]
    net = WheezeNet(width)
    net.load_state_dict(torch.load(os.path.join(run, "wheezenet.pt"), weights_only=True))
    net.eval()
    layers = [fold(net.c1, net.b1), fold(net.c2, net.b2), fold(net.c3, net.b3)]
    fc_w = net.fc.weight.detach().numpy()
    fc_b = net.fc.bias.detach().numpy()

    # ---- calibration activations (float, folded) ----
    d = dict(np.load(npz, allow_pickle=True))
    feats = d["feats"]
    sp = np.load(os.path.join(run, "split.npz"))
    rng = np.random.default_rng(1)
    s_tr, y_tr, _ = make_windows(d, sp["rec_tr"], stride=4)
    s_tr = s_tr[y_tr >= 0]
    cal_idx = rng.choice(len(s_tr), 6000, replace=False)
    raw = gather(feats, s_tr[cal_idx]).copy()
    raw[3000:] = augment(raw[3000:], rng)
    x = torch.from_numpy(F.normalize_window(raw).astype(np.float32)).unsqueeze(1)

    def fwd(x):
        acts = [x]
        h = x
        for i, (w, b) in enumerate(layers):
            h = torch.relu(torch.nn.functional.conv2d(h, torch.from_numpy(w), torch.from_numpy(b), padding=1))
            acts.append(h)
            if i < 2:
                h = torch.nn.functional.max_pool2d(h, 2)
        g = h.mean(dim=(2, 3))
        acts.append(g)
        out = g @ torch.from_numpy(fc_w).T + torch.from_numpy(fc_b)
        acts.append(out)
        return acts

    with torch.no_grad():
        acts = fwd(x)
    q_in = act_q(np.percentile(acts[0], 0.01), np.percentile(acts[0], 99.99))
    q_c = [act_q(0.0, float(np.percentile(a, 99.99))) for a in acts[1:4]]
    q_g = q_c[2]  # avgpool keeps its input quantization (CMSIS/TFLite requirement)
    q_o = act_q(float(acts[5].min()) - 1.0, float(acts[5].max()) + 1.0)

    # ---- quantize conv layers ----
    src = []
    in_q = q_in
    for li, ((w, b), oq) in enumerate(zip(layers, q_c), start=1):
        ws = np.maximum(np.abs(w).reshape(len(w), -1).max(1), 1e-8) / 127.0
        wq = np.clip(np.round(w / ws[:, None, None, None]), -127, 127).astype(np.int8)
        bq = np.round(b / (in_q[0] * ws)).astype(np.int64)
        mults = [qmult(in_q[0] * s / oq[0]) for s in ws]
        src.append(carr(f"c{li}_w", wq.transpose(0, 2, 3, 1), "int8_t"))  # OIHW -> OHWI
        src.append(carr(f"c{li}_b", bq, "int32_t"))
        src.append(carr(f"c{li}_mult", [m for m, _ in mults], "int32_t"))
        src.append(carr(f"c{li}_shift", [s for _, s in mults], "int32_t"))
        in_q = oq
    fs = max(np.abs(fc_w).max(), 1e-8) / 127.0
    fwq = np.clip(np.round(fc_w / fs), -127, 127).astype(np.int8)
    fbq = np.round(fc_b / (q_g[0] * fs)).astype(np.int64)
    fm, fsh = qmult(q_g[0] * fs / q_o[0])
    src.append(carr("fc_w", fwq, "int8_t"))
    src.append(carr("fc_b", fbq, "int32_t"))

    hdr = f"""// Generated by training/export.py - do not edit.
#pragma once
#include <cstdint>

namespace wheeze::model {{

inline constexpr int kW1 = {width};
inline constexpr int kW2 = {2 * width};
inline constexpr int kW3 = {2 * width};

inline constexpr float   kInScale  = {q_in[0]:.9e}f;
inline constexpr int32_t kInZp     = {q_in[1]};
inline constexpr int32_t kC1Zp     = {q_c[0][1]};
inline constexpr int32_t kC2Zp     = {q_c[1][1]};
inline constexpr int32_t kC3Zp     = {q_c[2][1]};
inline constexpr int32_t kFcMult   = {fm};
inline constexpr int32_t kFcShift  = {fsh};
inline constexpr float   kOutScale = {q_o[0]:.9e}f;
inline constexpr int32_t kOutZp    = {q_o[1]};

// Breath-cycle operating point chosen on validation (>= 90% specificity)
// for exactly this stride + gate configuration.
inline constexpr float kThreshold = {pick['threshold']:.6f}f;

// Energy knobs picked by training/optimize.py
inline constexpr uint32_t kInferStride   = {pick['stride_frames']};  // run the CNN every N frames (16 ms each)
inline constexpr bool     kGateEnabled   = {'true' if pick['gate_thr'] is not None else 'false'};
inline constexpr float    kGateTonality  = {(pick['gate_thr'] or 0.0):.6f}f;  // see features.tonality()
inline constexpr uint32_t kGateMinFrames = {pick['gate_k']};  // tonal frames needed in the window

extern const int8_t  c1_w[], c2_w[], c3_w[], fc_w[];
extern const int32_t c1_b[], c2_b[], c3_b[], fc_b[];
extern const int32_t c1_mult[], c2_mult[], c3_mult[];
extern const int32_t c1_shift[], c2_shift[], c3_shift[];

}} // namespace wheeze::model
"""
    open(os.path.join(gen_dir, "model_data.hpp"), "w").write(hdr)
    open(os.path.join(gen_dir, "model_data.cpp"), "w").write(
        "// Generated by training/export.py - do not edit.\n#include \"model_data.hpp\"\n\n"
        "namespace wheeze::model {\n\n" + "\n".join(src) + "\n} // namespace wheeze::model\n")

    # ---- DSP tables ----
    fb = F.mel_filterbank()
    starts, lens, wts = [], [], []
    for row in fb:
        nz = np.nonzero(row)[0]
        starts.append(nz[0])
        lens.append(nz[-1] - nz[0] + 1)
        wts.extend(row[nz[0]:nz[-1] + 1])
    offs = np.concatenate([[0], np.cumsum(lens)[:-1]])
    dsp_h = f"""// Generated by training/export.py - do not edit.
#pragma once
#include <cstddef>
#include <cstdint>

namespace wheeze::dsp {{

struct MicRate {{
    uint32_t fs;      // PCM rate requested from PDM20
    uint16_t decim;   // fs / kFs
    uint16_t taps;    // FIR length (multiple of decim)
    const float *fir;
}};
inline constexpr std::size_t kNumMicRates = {len(F.MIC_RATES)};
inline constexpr uint16_t    kMaxDecimTaps = {max(len(F.decim_fir(r)) for r in F.MIC_RATES)};
extern const MicRate mic_rates[kNumMicRates];  // lowest-power first
inline constexpr uint32_t    kFs        = {F.FS};
inline constexpr std::size_t kNfft      = {F.N_FFT};
inline constexpr std::size_t kHop       = {F.HOP};
inline constexpr std::size_t kNumMels   = {F.N_MELS};
inline constexpr std::size_t kNumFrames = {F.N_FRAMES};
inline constexpr float       kLogEps    = {F.LOG_EPS:e}f;
inline constexpr std::size_t kMelWeights = {len(wts)};

extern const float    hann[kNfft];
extern const uint16_t mel_start[kNumMels];
extern const uint16_t mel_len[kNumMels];
extern const uint16_t mel_off[kNumMels];
extern const float    mel_w[kMelWeights];

}} // namespace wheeze::dsp
"""
    dsp_c = ("// Generated by training/export.py - do not edit.\n#include \"dsp_tables.hpp\"\n\n"
             "namespace wheeze::dsp {\n\n"
             + carr("hann", F.hann_periodic(), "float", 8)
             + "".join(carr(f"decim_fir_{r // 1000}k", F.decim_fir(r), "float", 8) for r in F.MIC_RATES)
             + "\nconst MicRate mic_rates[kNumMicRates] = {\n"
             + "".join(f"    {{{r}, {r // F.FS}, {len(F.decim_fir(r))}, decim_fir_{r // 1000}k}},\n"
                       for r in F.MIC_RATES)
             + "};\n"
             + carr("mel_start", starts, "uint16_t")
             + carr("mel_len", lens, "uint16_t")
             + carr("mel_off", offs, "uint16_t")
             + carr("mel_w", wts, "float", 8)
             + "\n} // namespace wheeze::dsp\n")
    open(os.path.join(gen_dir, "dsp_tables.hpp"), "w").write(dsp_h)
    open(os.path.join(gen_dir, "dsp_tables.cpp"), "w").write(dsp_c)

    # ---- test vectors for host_test ----
    # 1) 20 s of a test recording at 4 kHz -> C must reproduce logmel frames.
    from scipy.signal import lfilter
    from prepare_icbhi import load_wav
    rec_te = sp["rec_te"]
    names = d["rec_name"]
    root = os.environ.get("ICBHI_DIR")
    if root:
        wav = load_wav(os.path.join(root, names[rec_te[0]] + ".wav"))[:F.FS * 20]
        wav.astype(np.float32).tofile(os.path.join(vec_dir, "audio_4k.f32"))
        F.logmel_frames(wav).tofile(os.path.join(vec_dir, "audio_4k_logmel.f32"))
    # 1b) synthetic PCM at every mic rate -> C decimator+features vs lfilter(h)[::M]
    #     (CMSIS keeps phase 0).
    for fs_mic in F.MIC_RATES:
        pcm_vectors(fs_mic, vec_dir, rng, lfilter)
    # 2) every test window (stride 4) as normalized float -> C quantizes + runs CNN.
    s_te, y_te, c_te = make_windows(d, rec_te, stride=4)
    win = F.normalize_window(gather(feats, s_te)).astype(np.float32)
    win.tofile(os.path.join(vec_dir, "test_windows.f32"))
    with torch.no_grad():
        p = torch.sigmoid(net(torch.from_numpy(win).unsqueeze(1))).numpy()
    np.savez(os.path.join(vec_dir, "test_meta.npz"), y=y_te, ckey=c_te, p_float=p)
    print(json.dumps({"in": q_in, "c": q_c, "out": q_o, "fc_mult": [fm, fsh], "n_test_windows": len(s_te)}, indent=1))


if __name__ == "__main__":
    main(*sys.argv[1:5])
