# EpiPatch

Firmware and models for a wearable that watches for anaphylaxis and fires an
adrenaline auto-injector if the wearer does not cancel. It is two patches,
each a Nordic nRF54L15 running Zephyr, linked by encrypted Bluetooth LE:

- **Chest patch** (senses and decides): ECG heart rate and chest-impedance breathing (ADS1292R, watched by the chip's low-power RISC-V core), PPG perfusion (MAX30101), a microphone wheeze detector, and a motion sensor for collapse, falls and taps (LSM6DSO).
- **Thigh patch** (senses and executes): a histamine sensor (AD5941), a motion sensor for falls and the triple-tap cancel (LSM6DSO), and the vibration countdown and epinephrine release motor.

> **Safety notice.** This is a research prototype. It has not been run on
> hardware, its thresholds are placeholders that have not been validated on
> patients, and the anaphylaxis model was tested only on simulated data. Do not
> use it to treat anyone.

| Folder | What it is |
|---|---|
| [`device/`](device/README.md) | Chest patch firmware: drivers, algorithms, decision engine, the FLPR coprocessor image, the Bluetooth link, and the PC tests for both patches |
| [`thigh/`](thigh/README.md) | Thigh patch firmware: AD5941 histamine, fall detection, countdown, tap cancel and release motor |
| [`wheeze/`](wheeze/) | The wheeze detector as a drop-in Zephyr module (described below) |
| [`app/`](app/) | A minimal sample that runs only the wheeze detector |
| [`training/`](training/) | Python: ICBHI preparation, CNN training, battery optimisation, export to C++ |
| [`vitals_model/`](vitals_model/README.md) | Vital-sign simulator, replay of the firmware's rules, model training and comparison |
| `host_test/` | PC test of the wheeze pipeline against the Python model |

The simulation data in `vitals_model/` is not in the repository (about 520 MB);
`vitals_model/README.md` shows how to regenerate it. The ICBHI 2017 dataset is
not included either; download it from the challenge site linked below.

Licensed under the [MIT License](LICENSE). The trained wheeze weights were
learned from the ICBHI 2017 database; cite Rocha et al., *Physiol. Meas.* 40
(2019) 035001 if you use them.

## Wheeze detector: PDM MEMS mic → nRF54L15

On-device wheeze detection modelled on Lee et al., *A Wearable Stethoscope for
Accurate Real-Time Lung Sound Monitoring and Automatic Wheezing Detection Based
on an AI Algorithm*, Engineering 53 (2025) 116–129. The method follows the
paper: log-mel spectrogram → small 2D CNN → hysteresis event counter.
Everything runs on the nRF54L15, tuned for battery life. No phone or app is
needed for detection.

It ships as a **drop-in Zephyr module** (`wheeze/`) with a three-function C API.

The **full device firmware** — ECG/Bio-Z (ADS1292R), PPG, chest + thigh IMUs,
histamine sensor, anaphylaxis decision, triple-tap cancel and MOSFET motor
deployment — is the Zephyr app in [`device/`](device/README.md), which uses this module.

```
PDM mic ──PDM20/EasyDMA──► 8 kHz PCM (16 kHz fallback) ──FIR÷2/÷4──► 4 kHz ──Hann·RFFT256──► |X|²
  ──32 mel (100–2000 Hz)──► ln ──40 frames (0.69 s), −mean──► tonality gate ──int8 CNN──► P(wheeze)
  ──smoothing + hysteresis──► your callback (+ optional 30 ms LED pulse)
```

## Integrating into your nRF54L15 project

### 1. Add the module

Copy `wheeze/` into your project (or keep it anywhere) and point the build at it,
in your app's `CMakeLists.txt` **before** `find_package(Zephyr)`:

```cmake
list(APPEND ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_SOURCE_DIR}/wheeze)
```

Or, if it lives in its own git repo, add it to your `west.yml`:

```yaml
    - name: wheeze
      url: <your-git-host>/wheeze
      revision: main
      path: modules/lib/wheeze
```

### 2. Enable it (`prj.conf`)

```
CONFIG_WHEEZE_DETECTOR=y
CONFIG_CPP=y
CONFIG_STD_CPP20=y
```

That's all the module needs. It selects the PDM/DMIC driver, FPU, CMSIS-DSP,
CMSIS-NN and the full libstdc++ itself. For the lowest current, also compile
out UART, console and logging, as `app/prj.conf` does.

### 3. Point it at the mic (devicetree overlay)

```dts
/ {
	aliases {
		wheeze-mic = &pdm20;
		wheeze-led = &led0;   /* optional, 30 ms pulse per event */
	};
};

&pdm20 {
	status = "okay";
	pinctrl-0 = <&pdm20_default_alt>;   /* CLK P1.12, DIN P1.13 on the DK */
	pinctrl-names = "default";
	clock-source = "PCLK32M";
};
```

`app/boards/nrf54l15dk_nrf54l15_cpuapp.overlay` has the full version with
pinctrl. PDM20 sits in the PERI power domain, so CLK/DIN must be **port 1**
pins. Mic SEL/LR → GND (left channel), VDD 1.8 V.

### 4. Call it (C or C++)

```c
#include <wheeze/wheeze.h>

static void on_wheeze(const struct wheeze_event *evt, void *user_data)
{
	if (evt->type == WHEEZE_EVT_END) {
		/* evt->count, evt->duration_ms, evt->timestamp_ms, evt->probability */
	}
}

int main(void)
{
	return wheeze_start(on_wheeze, NULL);   /* runs in its own thread */
}
```

| API | |
|---|---|
| `int wheeze_start(wheeze_callback_t cb, void *user_data)` | Starts the detector thread. Returns `-EALREADY` if running, `-ENODEV` if the PDM device isn't ready. `cb` may be `NULL`. |
| `int wheeze_stop(void)` | Powers the mic down and joins the thread. |
| `int wheeze_get_stats(struct wheeze_stats *)` | Event count, total wheeze time, whether it's wheezing now, mic on/off, PCM rate, continuous vs sampling. Thread-safe. |

The callback runs on the detector thread. Keep it short, e.g. queue a BLE
notification or set a flag.

### 5. Tune in menuconfig (`west build -t menuconfig` → *Wheeze detector*)

| Option | Default | |
|---|---|---|
| `CONFIG_WHEEZE_MIC_LP_CLK_MIN/MAX` | 400 000 / 1 100 000 Hz | PDM clock for the 8 kHz low-power path. **Set from your mic's datasheet** (its low-power-mode range). |
| `CONFIG_WHEEZE_MIC_CLK_MIN/MAX` | 1 000 000 / 3 250 000 Hz | PDM clock for the 16 kHz fallback. |
| `CONFIG_WHEEZE_MIC_RIGHT_CHANNEL` | n | Mic with SEL/LR tied high. |
| `CONFIG_WHEEZE_DUTY_CYCLE` | y | Adaptive mic duty-cycling, the biggest battery saving. Say `n` for gap-free monitoring. |
| `CONFIG_WHEEZE_DUTY_LISTEN_MS / SLEEP_MS / HOLD_MS` | 8000 / 22000 / 60000 | Sampling pattern while quiet, and how long to stay continuous after a candidate. |
| `CONFIG_WHEEZE_LED_PULSE_MS` | 30 | 0 = never drive the LED. |
| `CONFIG_WHEEZE_THREAD_STACK_SIZE / PRIORITY` | 4096 / 5 | |
| `CONFIG_WHEEZE_PROFILING` | n | Logs DSP/CNN µs/s, gate pass rate and mic-on % every 10 s (needs `CONFIG_LOG`). Bring-up only: costs current. |

### Resource footprint (width-8 model, shipped)

- **RRAM (code + data):** about 4 kB of weights and DSP tables, plus the
  detector code and the CMSIS-DSP/NN functions it links. Check the exact size
  with `west build -t rom_report`.
- **RAM:** about 65 kB of the 256 kB. That is about 24 kB of PDM DMA slab
  (3 × 8 kB), 20 kB of CNN activations, 4 kB CNN scratch, and about 18 kB of
  feature buffers, window and decimator state. Plus the 4 kB thread stack.
- **Peripherals:** PDM20 (exclusive), one GPIO (optional LED), one thread.
  Radio, timers and every other peripheral stay free for your application.

## Try the sample

```bash
west build -b nrf54l15dk/nrf54l15/cpuapp app
```

```bash
west flash
```

That is the battery build: no UART, LED pulses only. For logs and profiling
during bring-up:

```bash
west build -b nrf54l15dk/nrf54l15/cpuapp app -- -DEXTRA_CONF_FILE=debug.conf
```

## Results

Trained on ICBHI 2017 and evaluated on its **official test split**: 381
recordings from patients never seen in training, 2,753 breath cycles, 527 of
them with wheezes. "Breath-cycle" means a cycle counts as wheezing if any
window inside it fires.

| Model | Breath-cycle AUC | Sensitivity | Specificity | Params |
|---|---|---|---|---|
| width 16 (float) | 0.759 | 0.69 | 0.76 | 14.2 k |
| width 8 (float) | 0.727 | 0.59 | 0.77 | 3.6 k |
| **Shipped: width 8, 256 ms, gated, int8 C++ on the PC** | **0.731** | **0.64** | **0.75** | 3.6 k |

- Sensitivity and specificity use the threshold picked on validation for ≥ 90 %
  specificity. On unseen test patients, specificity lands at ~75 %, so expect
  some false alarms.
- By device, width 16 scores 0.88 AUC on Meditron, 0.82 on AKG C417L and 0.58
  on Littmann 3200. Performance depends heavily on the microphone, which is why
  fine-tuning on your own patch's recordings matters.
- Validation AUC is ~0.92, much higher than test. The validation set holds only
  about 12 patients, and the official test split has a different device mix.
- int8 quantisation costs nothing measurable: over all 117,719 test windows the
  firmware's C++ output correlates 0.996 with the float model.
- For comparison, the paper reports 99 % validation accuracy on 29 augmented
  clips, and 80.5 % wheeze-count agreement on one 79-minute COPD recording.
  Those are not comparable to a patient-disjoint benchmark.

## Battery optimisation

Levers are listed from largest saving to smallest.

| # | Lever | Where | Effect |
|---|---|---|---|
| 1 | **Adaptive mic duty-cycling** | `CONFIG_WHEEZE_DUTY_*`, `duty_cycler.hpp` | While quiet, the mic, PDM20 and its clock are on 8 s of every 30 s (27 %). Any wheeze candidate switches to continuous listening until 60 s pass without one. The mic is normally the largest steady draw. *Trade-off:* episode onset is noticed up to 22 s late, and a lone wheeze inside a gap is missed. |
| 2 | **Low PDM rate** | `CONFIG_WHEEZE_MIC_*`, `features.MIC_RATES` | Tries 8 kHz PCM at a 0.4–1.1 MHz PDM clock first, which is the low-power mode on most MEMS mics. Falls back to 16 kHz at 1–3.25 MHz. |
| 3 | **No UART, logging or console** | your `prj.conf` | A live UART costs more than the whole DSP+CNN pipeline. |
| 4 | **LED pulses, not steady** | `CONFIG_WHEEZE_LED_PULSE_MS` | A lit LED draws milliamps. |
| 5 | **CNN stride + tonality gate** | `training/optimize.py` → `model_data.hpp` | The CNN runs every 256 ms, and only when the window holds a tonal frame. Wheezes are narrow-band; breath noise and silence are not. The gate is O(1) per frame. |
| 6 | **Smaller model** | `optimize.py` | Width 8 has ~4× fewer MACs than width 16. |
| 7 | **256 ms DMA blocks, race to idle** | `wheeze_service.cpp` | 4 wake-ups/s. The work runs at 128 MHz on FPU and SIMD, then the CPU sits in WFI. DWT/trace stays off unless profiling. |

`optimize.py` sweeps width × stride × gate settings. For every configuration it
re-tunes the detection threshold on validation, then picks the cheapest one
within 0.01 val AUC and 0.03 val sensitivity of the best. Results on the ICBHI
test split (CNN compute while listening, before duty-cycling):

| Config | CNN runs | CNN compute | Test AUC | Sens | Spec |
|---|---|---|---|---|---|
| Original: width 16, every 64 ms, no gate | 100 % | 37.4 MMAC/s | 0.759 | 0.69 | 0.76 |
| Width 16, every 256 ms | 100 % | 9.4 MMAC/s (4.0× less) | 0.758 | 0.70 | 0.74 |
| **Shipped: width 8, every 256 ms, gated** | 65 % | **1.6 MMAC/s (22.8× less)** | 0.725 | 0.64 | 0.75 |

- **Running the CNN every 256 ms instead of every 64 ms is free.** Accuracy is
  unchanged because a wheeze lasts several windows.
- **Width 8 costs −0.03 AUC and −0.05 sensitivity on test.** For accuracy over
  battery, export the width-16, 256 ms configuration from
  `training/results/optimize.json` (4× less CNN work than the original).
- **The tonality gate skips 35 % of windows on ICBHI,** which is almost all
  breathing. Expect it to skip more on a patch worn all day.

**Measure it, don't trust estimates.** Use a Nordic PPK2 in source-meter mode
on the DK's nRF current-measurement header, with the battery build flashed.
Measure three phases separately: mic off (System ON idle), listening with the
CNN mostly gated off, and wheezing with the CNN running every 256 ms.

## Hardware features used on the nRF54L15

| Block | What it does here |
|---|---|
| **PDM20 + EasyDMA** | The mic is decimated to PCM in hardware and DMA'd into a 3-block slab. The CPU sleeps (WFI) inside `dmic_read()` between 256 ms blocks. PDM20 is stopped entirely during duty-cycle gaps. |
| **M33 FPU (FPv5-SP)** | All feature maths in f32 through CMSIS-DSP: `arm_fir_decimate_f32`, `arm_rfft_fast_f32`, `arm_cmplx_mag_squared_f32`, `arm_dot_prod_f32` (sparse mel), `arm_mean_f32`, plus `logf`/`expf`. |
| **M33 DSP extension** | The int8 CNN uses CMSIS-NN (`arm_convolve_wrapper_s8`, `arm_max_pool_s8`, `arm_avgpool_s8`, `arm_fully_connected_s8`). Its kernels compile to `SMLAD` (two 16-bit MACs per cycle) and `SXTAB16`, checked in the generated M33 assembly. |
| **DWT cycle counter** | `CONFIG_WHEEZE_PROFILING` only. |

Not used: the FLPR RISC-V coprocessor. The pipeline fits on the application
core with plenty of headroom, so offloading would only add IPC complexity.

## Layout

```
wheeze/                   ← the drop-in Zephyr module (this is what you integrate)
  zephyr/module.yml
  CMakeLists.txt, Kconfig
  include/wheeze/wheeze.h   public C API
  src/wheeze_service.cpp    thread, PDM20 (rate fallback, stop/start), duty-cycling, LED, API
  src/feature_extractor.*   decimate → RFFT → mel → log → window, tonality gate
  src/wheeze_net.*          CMSIS-NN int8 CNN + EventDetector
  src/duty_cycler.hpp       adaptive mic on/off
  src/generated/            model + DSP tables (written by training/export.py)
device/                   full wearable firmware (vitals, falls, histamine, deploy); see device/README.md
app/                      minimal C sample: prj.conf (battery), debug.conf, DK overlay
training/                 features, ICBHI prep, training, optimize, export; results/ = trained models
host_test/                builds the module's DSP/NN C++ for the PC and checks it against Python
```

C++ notes: the module is C++20 with no exceptions, RTTI or heap. Its globals
are `constinit` (compiler-enforced) or trivially constructed, so no static
constructors run at boot. The M33 cross-compile of the DSP/NN sources shows
no `__cxa_*` symbols and no `.init_array` entries. The public header is plain C, and was checked with `-std=c11 -pedantic`.

## Reproduce / retrain

```bash
pip install torch numpy scipy scikit-learn ziglang
# ICBHI 2017: https://bhichallenge.med.auth.gr  (ICBHI_final_database.zip + ICBHI_challenge_train_test.txt)
python training/prepare_icbhi.py ICBHI_final_database ICBHI_challenge_train_test.txt frames.npz
python training/train.py frames.npz run16 --epochs 30 --width 16
python training/train.py frames.npz run8 --epochs 30 --width 8
python training/optimize.py frames.npz optimize.json run16 run8
ICBHI_DIR=ICBHI_final_database python training/export.py frames.npz optimize.json wheeze/src/generated vectors
# PC check of the module's C++ (needs CMSIS-DSP + CMSIS-NN checkouts in <vendor>)
bash host_test/build.sh <vendor> && host_test/host_test.exe vectors && python host_test/compare.py vectors
```

## Important caveats

- **Built, not yet run on hardware.** The module, the `app/` sample and the
  `device/` firmware build with zero warnings against upstream Zephyr v4.4.2
  with Zephyr SDK 1.0.1 for `nrf54l15dk/nrf54l15/cpuapp`. The workspace
  manifest is `device/west.yml`. The wheeze-only sample uses 136 kB of flash
  and 78 kB of RAM. The DSP/NN C++ is also verified against Python on the PC.
  Nothing has been flashed to a board yet.
- **This is not the paper's data.** Their 29-clip training set is not public.
  This model is trained on ICBHI 2017 (920 recordings, 126 patients, 4
  stethoscope types, per-breath-cycle wheeze labels from clinicians). ICBHI
  labels whole breath cycles, not exact wheeze boundaries, so window labels are
  noisy.
- **Mic/coupling domain shift.** ICBHI was recorded with stethoscopes. A bare
  MEMS mic on skin has a different frequency response and noise. Training
  augments with random EQ, spectral tilt, additive noise and a mel shift, and
  the features are gain-normalised. Even so, plan to fine-tune on recordings
  from your own patch: record its audio, annotate it in the ICBHI `.txt`
  format, and add it to training.
- Not a medical device. Accuracy has not been validated on your hardware.
