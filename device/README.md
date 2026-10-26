# EpiPatch chest patch firmware (nRF54L15)

> **Prototype firmware for a drug-delivery device. It is not validated and not
> certified.** Every clinical threshold is a placeholder. Nothing here has run
> on hardware or on patients. An automatic injector needs clinical validation,
> a hazard analysis (ISO 14971), software lifecycle work (IEC 62304) and
> regulatory clearance before anyone wears it.

EpiPatch is two patches linked by encrypted Bluetooth LE, each with its own
nRF54L15:

- **Chest patch (this firmware)** senses, analyses and **decides**. Its low-power FLPR core watches the ECG and breathing and wakes its Arm Cortex-M33 core on an anomaly; the M33 analyses everything, including the thigh's data, and sends its verdict to the thigh.
- **Thigh patch ([../thigh](../thigh/README.md))** measures histamine (AD5941) and detects falls, and **executes**: it runs the vibration countdown, the three-tap cancel and the epinephrine release motor itself. A countdown that has started finishes there even if the link drops; with no link the thigh does nothing new.

| Signal | Sensor | Patch | What the chest gets from it |
|---|---|---|---|
| Breathing (Bio-Z) | ADS1292R ch 1, 32 kHz internal modulation | chest | breaths/min, apnea |
| Heart rate (ECG) | ADS1292R ch 2, lead-off detection | chest | Pan-Tompkins R-peaks → HR |
| Perfusion (PPG) | MAX30101/30102, red + IR | chest | perfusion index, pulse rate, uncalibrated SpO₂ (reported only) |
| Wheeze (mic) | PDM MEMS mic, `../wheeze` module | chest | wheeze events from the int8 CNN |
| Posture / collapse / fall | LSM6DSO | chest | standing / sitting / lying, fast trunk collapse, falls |
| Fall | LSM6DSO | thigh | falls (impact, then thigh horizontal and still), activity |
| Histamine | AD5941 amperometry | thigh | concentration vs. the wearer's own baseline |
| Cancel | LSM6DSO single-tap, either patch | both | 3 taps within 1.5 s; the chest forwards its taps to the thigh |
| Deploy | vibration motor + release motor (MOSFET) | thigh | countdown and release state, mirrored on the chest's buzzer |

## When the sensors are read

The ADS1292R runs all the time; everything else is read on a schedule, so the
application core sleeps most of the time.

```
FLPR coprocessor (RISC-V, flpr/)            application core (Arm M33, src/)
ADS1292R 125 Hz ─► QRS ─► HR ──┐            ┌─ idle: sleeps; IMU interrupts stay live
            └─► Bio-Z ─► RR ───┤            │   (tap = cancel, impact, free-fall, FIFO
                               ▼            │    watermark every 2.6 s for posture/collapse)
             heart-rate trigger ─┐          ├─ every 2 min: spot read, 15 s
             breathing trigger ──┴─ IPC ───►├─ trigger: burst, record everything for 1 min,
                                            │   then analyse the whole minute
             raw ECG + Bio-Z minute ◄───────┤   (raw ECG/Bio-Z kept on the FLPR side)
                                            └─ alert running: everything on until it's over
```

| Mode | Starts when | Sensors | Decision |
|---|---|---|---|
| Idle | after any read | ADS1292R (on the FLPR), IMU interrupts | none |
| Spot | every `APP_SPOT_PERIOD_S` (120 s) | PPG, mic and histamine on for `APP_SPOT_WINDOW_S` (15 s) | every second |
| Burst | a heart-rate or breathing trigger | everything on for `APP_BURST_S` (60 s); every PPG, IMU, ECG and Bio-Z sample kept | once, over the whole minute |
| Active | an alert or deployment is running | everything on | every second, until it is over |

**The two triggers** (`algo/vital_trigger.hpp`, thresholds in Kconfig) start a
recording; they are not findings themselves:

- **Heart rate:** at or above 110/min, or at or below 40/min (`APP_TRIG_HR_*`).
- **Breathing:** at or above 25/min, at or below 8/min, or apnea (`APP_TRIG_RR_*`, `APP_TRIG_APNEA`).

A condition must last 5 s (`APP_TRIG_CONFIRM_S`). Each trigger then stays quiet
until the burst it started has been analysed. If the condition still holds, it
fires again at once, so the device keeps recording while the abnormality lasts.
The high defaults sit below the tachycardia and fast-breathing findings, so the
recording is already running when a finding could form.

**Why a coprocessor:** the ADS1292R itself can't do this. It only digitises the
signals, and its GPIO pins change only when the MCU writes them over SPI.
Something has to compute heart rate and breathing rate from every sample. The
nRF54L15's FLPR RISC-V core does that with the same algorithm code as the main
core (`DEV_PORTABLE_DSP` replaces CMSIS-DSP there). It sends a message over IPC
(ICMsg on the VPR event interface) only when a trigger fires. Build with
`-DAFE_CORE=app` to read the ADS1292R on the main core instead. The triggers
and schedule are then the same, but the main core wakes 125 times a second.

**What a burst keeps and how it is analysed** (`src/burst.*`):

- **On the main core:** every PPG sample (25 Hz), every chest accelerometer sample (12.5 Hz, in mg), and a snapshot each second of heart rate, breathing, wheeze, posture/collapse/fall, and the thigh's histamine and fall (requested every 10 s during the burst).
- **On the coprocessor:** the raw ECG and Bio-Z minute, 6 bytes per sample (45 kB). It stays in the coprocessor's RAM until the next burst, and the main core can read it (`afe::recording()`), for example to export it.
- **Analysis:** at the end of the minute, the PPG is processed from the raw samples, and all 60 seconds are replayed through the decision engine. A trigger in any second of the minute starts the alert.

Trade-offs to know:

- **Slower alarm after a trigger:** a burst decides only at its end, so an alarm can come up to a minute later than with continuous monitoring.
- **Slower spot-read findings:** between spot reads, perfusion, wheeze and histamine are not watched. A finding that needs them can take up to 2 minutes longer to form.
- **What never sleeps:** falls, collapse and the cancel tap are always live.

## How the decision works

```
FLPR: ADS1292R ─► QRS ─► HR, Bio-Z ─► RR/apnea ─► triggers ──IPC──┐
main loop (event-driven; 1 s tick only while reading)              │
  LSM6DSO INT1: tap ─► TapCounter ─► Cancel ──BLE──► thigh         │
  LSM6DSO INT1: impact / free-fall / FIFO ─► BodyMotion            │
  SamplingSchedule: spot / burst / active  ◄───────────────────────┘
    ├─► MAX30101 FIFO (on while reading) ─► PI, pulse
    ├─► mic (wheeze_listen while reading) ─► wheeze
    ├─► thigh Report (histamine, thigh fall) ◄──BLE── thigh, on request + every 2 min
    └─► AnaphylaxisEngine ─► Decision ──BLE──► thigh: countdown, cancel, release
```

The thigh's data counts as missing after 5 minutes without a report
(`APP_THIGH_FRESH_S`): the chest then has no histamine and no thigh fall, and
keeps deciding on its own sensors (rule B still works).

**Findings**, grouped by organ system. A finding needs to persist before it counts:

- *Exposure*: histamine at least `RISE` nM above the personal baseline **and** at least `RATIO` × baseline, for 2 samples in a row. It then counts as exposure for 60 min (`APP_HIST_EXPOSURE_MIN`), because plasma histamine clears in 15–60 min while food reactions can take about 30 min to reach the airway or circulation.
- *Respiratory*: any of:
  - repeated wheeze: heard in at least 3 separate seconds within 60 s (the finding then holds 60 s). One stray detection is not enough, because the CNN flags about a quarter of normal breaths;
  - breathing ≥ 28/min for 30 s;
  - breathing ≤ 8/min for 15 s, or apnea (no breath for 20 s).
- *Cardiovascular*: any of:
  - HR ≥ max(120, resting baseline + 40) for 60 s;
  - perfusion index ≤ 50 % of baseline for 30 s;
  - collapse (trunk tilts 35°→60° in < 0.8 s, then motionless for 15 s);
  - a fall (impact or free-fall, then lying, still down 10 s later).

**Trigger rules** (each can be switched off in Kconfig):

- **A**: exposure AND (respiratory OR cardiovascular).
- **B**: respiratory AND (collapse OR fall). This one needs no histamine, because the sensor may lag.

**Guards against false deployment:**

- Heart- and breathing-rate findings are ignored while the IMUs show movement above 150 mg, so exercise doesn't count.
- A sensor that is off or invalid contributes nothing (ECG lead-off, PPG not on skin, histamine out of range).
- Baselines are learned only at rest and only when there are no findings.

**After the decision:** the chest sends every verdict to the thigh patch,
which runs the countdown, the cancel and the release, with all the motor
safety layers: see [../thigh/README.md](../thigh/README.md). The chest's
buzzer mirrors the thigh's countdown, and three taps on the chest send a
cancel.

**The Bluetooth link** (`src/patch_link.cpp`, `src/patch_ble.hpp`,
`src/patch_msgs.hpp`): the chest is the central and GATT client; the thigh is
the peripheral and GATT server. The two patches pair once with LE Secure
Connections and bond. Each refuses any further pairing while it has a bond,
the thigh accepts connections only from its bonded chest, and both
characteristics need an encrypted link, so no other device can send a
decision or a cancel. The pairing is "Just Works" (the patches have no
display or keypad), so pair them once in a controlled setting such as
assembly.

## Build

The app pulls in the wheeze module from `../wheeze` (see the top-level README).
It builds with **upstream Zephyr v4.4.2 and Zephyr SDK 1.0.1** (GCC 14.3).
[west.yml](west.yml) is a minimal workspace manifest: Zephyr, CMSIS, CMSIS-DSP, CMSIS-NN, hal_nordic, segger, and mbedTLS + TF-PSA-Crypto for Bluetooth pairing. An nRF Connect SDK workspace has all of these too. The same workspace builds the thigh patch (`../thigh`).

```bash
pip install west
mkdir ws && cp device/west.yml ws/manifest/west.yml     # (create ws/manifest first)
cd ws && west init -l manifest && west update --narrow -o=--depth=1
pip install -r zephyr/scripts/requirements-base.txt
# Zephyr SDK 1.0.1: unpack the "minimal" bundle + toolchain_gnu_*_arm-zephyr-eabi
# + toolchain_gnu_*_riscv64-zephyr-elf (for the FLPR coprocessor) into <sdk>/gnu/,
# then: cmake -P <sdk>/cmake/zephyr_sdk_export.cmake

# both cores (sysbuild adds the FLPR image from device/flpr)
west build --sysbuild -b nrf54l15dk/nrf54l15/cpuapp ../device
west build --sysbuild -b nrf54l15dk/nrf54l15/cpuapp ../device -- -Ddevice_EXTRA_CONF_FILE=debug.conf   # RTT logs
# ADS1292R on the application core instead (no coprocessor, no RISC-V toolchain)
west build --no-sysbuild -b nrf54l15dk/nrf54l15/cpuapp ../device -- -DAFE_CORE=app
west build -t menuconfig    # "EpiPatch chest patch": every threshold, timing and trigger
west flash                  # nRF54L15 DK over its on-board J-Link
```

Build sizes (production `prj.conf`, Bluetooth included, no warnings):

| Build | Flash | RAM |
|---|---|---|
| Chest application core, ADS1292R on the FLPR (default) | 297 kB of 1428 kB (20 %) | 132 kB of 160 kB (81 %); the FLPR has the other 96 kB |
| FLPR coprocessor | not built yet: needs the RISC-V toolchain | |
| Thigh patch (`../thigh`) | 170 kB of 1524 kB (11 %) | 31 kB of 256 kB (12 %) |

[prebuilt/](prebuilt/) still holds hex files from the earlier single-patch
version (continuous monitoring, DRDY on P0.00).

**Pairing the two patches:** flash both, power them up near each other with
no bond stored; the chest finds the thigh by its service and they pair and
bond automatically. After that each refuses any other device. To re-pair
(for example after replacing a patch), erase the settings partition on both.

## Wiring

All wiring is in [boards/nrf54l15dk_nrf54l15_cpuapp.overlay](boards/nrf54l15dk_nrf54l15_cpuapp.overlay). **The pin numbers are placeholders.** Check them against the nRF54L15 pin table for your PCB: SPI SCK and TWIM SCL must be on dedicated clock pins, and the SAADC inputs are P1.04–07 and P1.11–14.

| Part | Bus / pins |
|---|---|
| PDM mic | pdm20: CLK P1.12, DATA P1.13 |
| ADS1292R | spi21 at 500 kHz, mode 1: SCK P1.03, MOSI P1.08, MISO P1.09, CS P1.10; DRDY P1.07, PWDN P0.01, START P0.02. Owned by the FLPR ([flpr/boards/](flpr/boards/nrf54l15dk_nrf54l15_cpuflpr.overlay)), which gets GPIOTE20 (port 1 interrupts); the application core keeps GPIOTE30 (port 0) |
| MAX30101 | i2c22 at 0x57 (SCL P1.04, SDA P1.05); INT P0.03 |
| LSM6DSO chest | i2c22 at 0x6A (SA0 low); INT1 P0.05 |
| Buzzer | P1.14 (`alert-gpios`, optional; mirrors the thigh's countdown) |
| Status LED | DK LED0 |
| Radio | Bluetooth LE to the thigh patch |

Defaults that depend on your hardware (all in Kconfig):

- **IMU mounting:** `APP_CHEST_UP_AXIS` names the sensor axis that points to the head when the wearer stands.
- **Taps:** the threshold is 1.5 g at 417 Hz (`APP_TAP_*`). The LSM6DSO only detects single and double taps in hardware, so the firmware counts three single-tap interrupts.

The histamine sensor, its calibration, the thigh IMU and both motors are set
up in [../thigh](../thigh/README.md).

## Low-power design

- **ADS1292R** runs at 125 SPS. It has no FIFO, so the FLPR coprocessor wakes per sample, reads 9 bytes over SPI, and processes the signals in 200 ms blocks. The application core is not involved until a trigger fires.
- **MAX30101** is shut down between reads. While on, it samples at 50 sps with 2-sample averaging, and the FIFO almost-full interrupt fires about once a second.
- **LSM6DSO:**
  - The tap, impact and free-fall detectors run inside the IMU; the MCU wakes only on INT1.
  - Accelerometer data is batched in the IMU's FIFO at 12.5 Hz. A watermark interrupt at 32 samples (`APP_IMU_FIFO_WTM`) wakes the MCU about every 2.6 s to update posture and collapse.
  - The gyro is off.
- **Microphone:** off except during reads. The app calls `wheeze_listen()` for the read window, and the module's own duty-cycle gap is set to an hour (`CONFIG_WHEEZE_DUTY_SLEEP_MS`). A wheeze in progress still keeps it listening.
- **Bluetooth:** one connection to the thigh. Messages flow only at reads (a report request, then a decision each second) and when the thigh reports a change or its 2-minute heartbeat.
- **Watchdog:** the main loop wakes at least every 2 s to feed the 4 s watchdog, even when idle.
- **Production build:** no UART and no logging (`prj.conf`).

## Layout

```
device/
  CMakeLists.txt, Kconfig, prj.conf, debug.conf
  sysbuild.cmake           adds the FLPR image
  boards/…overlay          pin map; afe_flpr.overlay hands the ADS1292R to the FLPR
  dts/bindings/            wearable,ads1292r / max30101 / lsm6dso
  flpr/                    FLPR coprocessor firmware: ADS1292R, HR/RR, triggers, raw minute
  src/main.cpp             event loop, sampling schedule, actions
  src/afe_link.*           ECG/Bio-Z from the FLPR over IPC (or the afe thread with AFE_CORE=app)
  src/afe_msgs.hpp         messages between the two cores
  src/burst.*              one-minute recording and its analysis
  src/patch_link.*         Bluetooth link to the thigh (central, GATT client)
  src/patch_ble.hpp        the link's GATT service and UUIDs (shared with the thigh)
  src/patch_msgs.hpp       messages between the patches (shared with the thigh)
  src/board.hpp            every devicetree spec in one place
  src/persist.*            deployment latch (used by the thigh)
  src/drivers/             ads1292r, max30101, lsm6dso, motor, alert, irq_pin (motor: thigh)
  src/algo/                pure C++ (no Zephyr), host-tested; also used by the thigh:
    ecg_qrs, resp_rate, ppg, body_motion, tap_counter, histamine,
    anaphylaxis (findings + rules), deploy_controller (state machine),
    vital_trigger (HR/RR triggers), sampling_schedule (idle/spot/burst/active),
    thigh_view (thigh data on the chest), thigh_motion (thigh fall), amperometry (AD5941),
    generated/filters.hpp  (tools/gen_filters.py, scipy-designed biquads)
  tests/host/              PC tests: synthetic ECG/Bio-Z/PPG + scenarios
  tools/gen_filters.py
```

## Verification

`tests/host/build.sh <dir with CMSIS-DSP>` compiles the unmodified `src/algo` code for the PC twice: once against CMSIS-DSP, as on the application core, and once with the plain C++ filters the FLPR uses (`DEV_PORTABLE_DSP`). It generates synthetic recordings and runs **118 checks in each build; all pass**:

| Area | Result |
|---|---|
| Filters | CMSIS biquads match scipy `sosfilt` to ≤ 2.6e-6 |
| ECG | HR within 3 bpm at 60/100/150/72 bpm, with baseline wander, mains, noise and muscle bursts; 381 of 383 beats found; invalid when leads are off |
| Bio-Z | 13.6 / 33.3 / 12.0 per min for truths of 14 / 32 / 12; apnea flagged within 20 s |
| PPG | PI 1.97 % / 0.60 % for truths of 2.0 / 0.6; pulse 75 / 107 for truths of 75 / 110 (one 25 Hz sample of quantisation at 110); invalid off-skin |
| IMU (pass) | standing, sitting; fall and collapse after a 1 s drop with impact; flags clear on standing up |
| IMU (no false alarm) | a slow lie-down is *not* a collapse; a bump while upright is *not* a fall |
| Taps | 3 taps in 1.5 s cancel; 2 taps, slow taps and a ringing tap don't |
| Histamine | baseline learning; elevation after 2 samples; baseline frozen while elevated; open sensor never "elevated" |
| Engine (deploys) | alert, then fire at 20 s, single-dose latch; cancel, then cooldown, then a severe re-alert; tachycardia needs 60 s; perfusion drop |
| Engine (no deploy) | exercise doesn't trigger; invalid sensors don't trigger; no re-arm after a reboot |
| HR/RR triggers | fire after 5 s above/below the limits (HR high and low, RR high and low, apnea); fire once, then latched until re-armed; re-fire at once if still abnormal; silent with leads off |
| Schedule | spot read at boot and every 2 min for 15 s; trigger → 1 min burst; burst counts as a read; alert → everything on until it ends; triggers absorbed during a burst or alert |
| Burst | 60 s / 1,500 PPG / 780 IMU samples kept; PPG analysed from the raw samples; calm minute → no trigger; histamine + wheeze minute → trigger |
| Histamine timing | readings 2 min apart warm up and learn the baseline; two high readings 10 s apart → elevated |
| Thigh fall | impact, then thigh horizontal and still → fall; clears after 5 s upright; lying down without an impact or a bump while standing → no fall |
| Chest–thigh fusion | no or stale (> 5 min) thigh report → no histamine, no thigh fall; fresh report → histamine, thigh fall and activity merged; thigh countdown keeps the chest fully on; thigh histamine + thigh fall + chest wheeze → the chest decides to trigger |
| AD5941 maths | LPDAC codes match ADI's example formula; bias within one LSB; ADC code → volts → nA across RTIA |

**Also checked:**

- **Real Zephyr build:** the production, RTT-debug and wheeze-only sample builds compile and link for `nrf54l15dk/nrf54l15/cpuapp` with **zero warnings**. The first real build needed three fixes:
  - the wheeze module didn't enable the CMSIS-DSP transform, complex-math and statistics groups, so the link failed;
  - the SPI devicetree macro's delay argument is deprecated;
  - a warning about the service button's address.
- **No static constructors:** the firmware image has no `init_array` at all. Every global in `main.cpp` is `constinit`, so the compiler enforces this.
- **Algorithms on the M33:** they use FPU instructions and the CMSIS biquad calls.
- **Before the real build,** the drivers were type-checked against stand-in Zephyr headers. That caught a register/enum name clash in the IMU driver, which is fixed.

**Not done yet:**

- The FLPR coprocessor image hasn't been compiled: that needs the Zephyr SDK's RISC-V toolchain. Its algorithm code is covered by the portable-DSP test build.
- The two-core IPC and the Bluetooth link between the patches haven't run, so pairing, reconnection and message timing are untested.
- The AD5941 driver follows ADI's library and example register by register, but the bias, RTIA and calibration are placeholders until the histamine sensor's datasheet sets them.
- Nothing has run on hardware. The register sequences come from the datasheets but haven't been tried on real chips.
- Only built against upstream Zephyr 4.4.2. A specific nRF Connect SDK release may need small changes.
- The tests use synthetic signals, not patient recordings.
- Stridor is not separately modelled: the mic model was trained only on wheezes.
