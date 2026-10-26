"""Writes the report's CSV datasets from results.json.
usage: python export_report.py <results.json> <out dir>"""
import csv
import json
import os
import sys

r = json.load(open(sys.argv[1]))
out = sys.argv[2]
os.makedirs(out, exist_ok=True)
RULES = "Rules engine (firmware)"
SHORT = {"Logistic regression": "logistic", "Decision tree (depth 6)": "tree",
         "Gradient-boosted trees": "gbt", RULES: "rules"}
LABEL = {
    "anaphylaxis_resp": "Anaphylaxis, respiratory", "anaphylaxis_cv": "Anaphylaxis, cardiovascular",
    "anaphylaxis_severe": "Anaphylaxis, severe with collapse",
    "anaphylaxis_low_hist": "Anaphylaxis, weak histamine signal",
    "normal_day": "Normal day", "sleep": "Sleep (half with apnea)", "exercise": "Exercise and recovery",
    "asthma": "Asthma attack", "allergy_mild": "Mild allergy, skin only", "vasovagal": "Fainting (vasovagal)",
    "fall": "Accidental fall", "panic": "Panic attack", "hist_artifact": "Histamine sensor artifact",
}


def write(name, cols, rows):
    with open(os.path.join(out, name + ".csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(cols)
        w.writerows(rows)


def rnd(v, n=4):
    return "" if v is None else round(v, n)


rows = []
ru, sh = r["rules"]["test"], r["shift"]["Rules engine"]
rows.append([RULES, rnd(ru["sensitivity"]), rnd(ru["false_alarm_rate"]), rnd(ru["ttd_median_s"], 0),
             rnd(ru["ttd_p90_s"], 0), "", "", "", "", "", "Kconfig thresholds",
             rnd(sh["sensitivity"]), rnd(sh["false_alarm_rate"]), rnd(sh["ttd_median_s"], 0)])
for m, e in r["models"].items():
    t, s = e["test"], r["shift"][m]
    rows.append([m, rnd(t["sensitivity"]), rnd(t["false_alarm_rate"]), rnd(t["ttd_median_s"], 0),
                 rnd(t["ttd_p90_s"], 0), rnd(e["auc"]), rnd(e["ap"]), rnd(e["thr"], 3), e["size"]["params"],
                 e["size"]["flash_bytes"], e["size"]["ops"],
                 rnd(s["sensitivity"]), rnd(s["false_alarm_rate"]), rnd(s["ttd_median_s"], 0)])
write("models", ["model", "sensitivity", "false_alarm_rate", "ttd_median_s", "ttd_p90_s", "auc", "avg_precision",
                 "threshold", "params", "flash_bytes", "compute", "shift_sensitivity", "shift_false_alarm_rate",
                 "shift_ttd_median_s"], rows)

write("tradeoff", ["model", "threshold", "sensitivity", "false_alarm_rate"],
      [[m, round(p["thr"], 3), rnd(p["sensitivity"]), rnd(p["false_alarm_rate"])]
       for m, e in r["models"].items() for p in e["sweep_test"]])

write("roc", ["model", "fpr", "tpr"],
      [[m, f, t] for m, e in r["models"].items() for f, t in zip(e["roc"]["fpr"], e["roc"]["tpr"])])

ps = r["per_scenario"]
rows = []
for i, row in enumerate(ps["Rules engine"]):
    pos = "detected" in row
    key = "detected" if pos else "false_alarm"
    vals = [rnd(ps[k][i][key]) for k in ["Rules engine", "Logistic regression", "Decision tree (depth 6)",
                                          "Gradient-boosted trees"]]
    rows.append([row["scenario"], LABEL[row["scenario"]], "yes" if pos else "no", row["n"],
                 "detected" if pos else "false alarm", *vals])
write("scenarios", ["scenario", "label", "anaphylaxis", "n_test", "metric", "rules", "logistic", "tree", "gbt"], rows)

rows = [[RULES, round(v)] for v in r["rules"]["ttd"]]
rows += [[m, round(v)] for m, e in r["models"].items() for v in e["ttd"]]
write("detection_time", ["model", "seconds_from_onset"], rows)

gb, lr = r["models"]["Gradient-boosted trees"], r["models"]["Logistic regression"]
write("features", ["feature", "description", "importance", "lr_weight"],
      [[f, d, gb["importance"][i], lr["coef"][i]] for i, (f, d) in enumerate(r["features"])])

rows = []
for k, ex in enumerate(r["examples"]):
    for j in range(len(ex["p"])):
        rows.append([k + 1, LABEL[ex["scenario"]], round(j * 10 / 60, 3), ex["hr"][j], ex["rr"][j],
                     ex["pi_ratio"][j], ex["hist_ratio"][j], ex["wz60"][j], ex["act"][j], int(ex["down"][j]),
                     ex["p"][j], ex["rule"][j], round(ex["t0"] / 60, 2),
                     "" if ex["t1"] is None else round(ex["t1"] / 60, 2)])
write("timelines", ["example", "scenario", "minute", "heart_rate", "breathing_rate", "perfusion_ratio",
                    "histamine_ratio", "wheezes_last_min", "activity_g", "collapsed_or_fell", "model_probability",
                    "rules_alarm", "event_start_min", "organ_onset_min"], rows)

write("recordings", ["scenario", "label", "anaphylaxis", "total", "train", "validation", "test"],
      [[d["scenario"], LABEL[d["scenario"]], "yes" if d["anaphylaxis"] else "no", d["n"], d["train"], d["val"],
        d["test"]] for d in r["dataset"]])
gbm = r["models"]["Gradient-boosted trees"]
print("gbt thr", gbm["thr"])
