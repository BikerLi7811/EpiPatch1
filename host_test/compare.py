"""Compare C (int8 CMSIS-NN) probabilities with the float PyTorch model on the test split."""
import sys

import numpy as np
from sklearn.metrics import roc_auc_score

vec = sys.argv[1]
m = np.load(f"{vec}/test_meta.npz")
pc = np.fromfile(f"{vec}/c_probs.f32", np.float32)
pf, y = m["p_float"], m["y"]
assert len(pc) == len(pf), (len(pc), len(pf))
k = y >= 0
print(f"windows {len(pc)}: mean |p_int8 - p_float| = {np.abs(pc - pf).mean():.4f}, "
      f"max {np.abs(pc - pf).max():.4f}, corr {np.corrcoef(pc, pf)[0, 1]:.4f}")
print(f"window AUC  float {roc_auc_score(y[k], pf[k]):.4f}   int8-C {roc_auc_score(y[k], pc[k]):.4f}")
