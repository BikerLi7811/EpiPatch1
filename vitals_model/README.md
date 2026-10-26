# Vitals model: anaphylaxis decision from all sensors

Trains classifiers on 1 Hz vitals (ECG heart rate, Bio-Z breathing, PPG
perfusion/SpO₂, wheeze detections, IMU activity/collapse/fall, histamine).
They are compared with the firmware's own rule engine (`device/src/algo/anaphylaxis.cpp`).

**The data is simulated.** No public dataset records anaphylaxis with these
sensors. `sim.py` writes what the device's detectors would report in 13
situations: four kinds of anaphylaxis, plus normal days, sleep with apnea,
exercise, asthma, mild allergy, fainting, falls, panic and histamine-sensor
artifacts. The numbers compare methods; they are not clinical accuracy.
Re-train on recordings from the patch before using any threshold.

```bash
python sim.py data.npz 2000 7                # training/validation/test world
python sim.py shift.npz 800 11 shift         # harder world, robustness only
# write vitals.bin / shift.bin (int32 n,t,f header + float32), then:
python -m ziglang c++ -O2 -std=c++20 -D__GNUC_PYTHON__ -I../device/src -I<CMSIS-DSP>/Include \
  -I<CMSIS-DSP>/PrivateInclude replay.cpp ../device/src/algo/anaphylaxis.cpp -o replay.exe
./replay.exe vitals.bin rules.bin && ./replay.exe shift.bin shift_rules.bin
python train.py data.npz rules.bin results.json shift.npz shift_rules.bin
python export_report.py results.json report_data
```

## Results (simulated test set, 400 recordings; harder world, 800)

Each model's threshold was chosen on validation so that it raises no more false
alarms than the rules. The rules are the firmware's engine after the
literature-review fixes:

- high histamine counts as exposure for 60 minutes;
- a wheeze finding needs wheeze heard in at least 3 seconds within 60 seconds;
- the histamine offset default was fixed.

| Method | Episodes caught | False-alarm recordings | Median time to alarm | Harder world: caught / false alarms | Size |
|---|---|---|---|---|---|
| Firmware rules, before the fixes | 77 % | 13.4 % | 61 s | 73 % / 18.1 % | — |
| **Firmware rules, after the fixes** | **78 %** | **0.4 %** | 112 s | 69 % / 1.9 % | — |
| Logistic regression | 83 % | 2.8 % | 120 s | 68 % / 0.2 % | 61 params, 0.2 kB |
| Decision tree (depth 6) | 99 % | 10.9 % | 91 s | 98 % / 8.2 % | 85 nodes, 0.7 kB |
| **Gradient-boosted trees** | **95 %** | **0.8 %** | 140 s | 93 % / 0.8 % | 4,500 nodes, ~35 kB |

- **The repeated-wheeze rule removed almost all of the rules' false alarms.**
  Fainting dropped from 44 % to 0, histamine artifacts from 80 % to 0, mild
  allergy from 33 % to 0, and falls from 19 % to 0. Requiring three wheeze
  seconds delays the median alarm from 61 s to 112 s.
- **The models were re-scored against the stricter rules.** Their thresholds
  now have to match a false-alarm rate near zero, which costs them some
  sensitivity. The model weights are unchanged.
- **The rules still miss** most episodes where the histamine sensor barely
  responds. They catch 12 %, against 76 % for the boosted trees.
- **The exposure hold isn't tested by this simulator.** Simulated histamine
  stays high once it rises, so the 60-minute hold barely changes these numbers.
  The host tests cover it instead: histamine clears, then tachycardia 25 minutes
  later still triggers.
- `results_before.json` keeps the run from before the fixes.

`results.json` has everything: ROC curves, threshold sweeps, results per
situation, time to alarm, feature importance and example timelines.
