# EpiPatch thigh patch firmware (nRF54L15)

> **Prototype firmware for a drug-delivery device. It is not validated and not
> certified.** Every threshold and the histamine calibration are placeholders.
> Nothing here has run on hardware or on patients.

The thigh patch sits where the epinephrine goes in. It measures histamine,
detects falls, and **carries out** the chest patch's decisions. The chest
patch ([../device](../device/README.md)) makes every decision.

| Part | Sensor / actuator | What it does |
|---|---|---|
| Histamine | AD5941, constant-potential amperometry | concentration vs. the wearer's own baseline, sent to the chest |
| Fall + tap | LSM6DSO | falls (impact, then the thigh horizontal and still for 10 s) and the three-tap cancel |
| Countdown | vibration motor on its own GPIO | 20 s countdown, faster in the last 5 s |
| Release | release motor through a MOSFET on the release GPIO | at most 3 s, or until the end-stop |
| Link | Bluetooth LE peripheral | reports to the chest; receives decisions, cancels and report requests |

## What it does

1. **Reports.** Every 2 minutes, and whenever the chest asks, it reads histamine and sends a Report: histamine, its baseline and whether it is raised, the thigh's fall and activity, and the countdown state. While the chest is reading or a countdown runs, it reads histamine every 10 s. A change in histamine, fall or countdown state is reported straight away.
2. **Countdown.** When the chest's decision says "trigger", it arms the release motor and runs the vibration countdown.
3. **Cancel.** Three taps on either patch cancel it (the chest forwards its taps). For 5 minutes afterwards, only a *severe* decision (breathing trouble plus collapse or fall) starts a new countdown.
4. **Release.** If not cancelled, the release motor runs: at most 3 s, or until the end-stop closes.
5. **Done.** The deployment is saved to RRAM. After the last dose (`APP_MAX_DEPLOYMENTS`, default 1) the patch does nothing more, even after a reboot. Holding the service button for 3 s at power-up clears the latch after the injector is replaced.

**Link loss:** a countdown that has started runs to the end here, and a tap
on the thigh still cancels it. Without a link the thigh does nothing new; it
never decides on its own.

**Release motor safety layers** (shared driver `../device/src/drivers/motor.*`):

- The gate is forced off by a `SYS_INIT` hook straight after the GPIO driver starts.
- The motor runs only while armed (from the start of the countdown), and only once per arm; a cancel disarms it.
- A timer in interrupt context cuts the gate after 3 s, so a stuck thread can't leave the motor on.
- A 4 s hardware watchdog resets the chip, which makes the pins inputs. **Fit a 100 kΩ pull-down on the MOSFET gate** so it stays off during reset.
- The vibration motor is on a separate GPIO, so buzzing never drives the release motor.

## Histamine on the AD5941

`src/drivers/ad5941.*` follows ADI's AD594x library and its amperometric
example register by register:

- After every reset it applies ADI's start-up register table and checks the chip IDs.
- The low-power DAC holds the sensor bias between the reference and sense electrodes all the time. Electrochemical sensors need a steady bias, so it is never switched off between readings.
- The low-power TIA turns the cell current into a voltage across RTIA.
- A reading powers the ADC, discards 4 SINC2 + notch results while the filter settles, averages 16, and powers the ADC down again.
- The current goes through the histamine monitor (`../device/src/algo/histamine.hpp`), which learns the personal baseline and flags "raised" after 2 readings in a row. The chest then holds exposure for 60 minutes.

The conversion maths (`../device/src/algo/amperometry.hpp`) is checked by the PC tests.

**Set these from the sensor's datasheet** (all in Kconfig, all placeholders now):

| Setting | Default | Meaning |
|---|---|---|
| `APP_HIST_VZERO_MV` | 1100 | sense-electrode potential |
| `APP_HIST_BIAS_MV` | 0 | working potential, reference minus sense electrode |
| `APP_HIST_RTIA` | 24 (196 kΩ) | TIA gain: about ±6 µA full scale with PGA 1.5 |
| `APP_HIST_OFFSET_NA`, `APP_HIST_NM_PER_UA` | 0, 1000 | calibration: current at zero histamine, and slope |
| `APP_HIST_MIN_NA`, `APP_HIST_MAX_NA` | −500, 6000 | outside this range the reading counts as a sensor fault |

## Build

Same workspace as the chest patch ([../device/README.md](../device/README.md#build)):

```bash
west build -b nrf54l15dk/nrf54l15/cpuapp ../thigh
```

```bash
west build -b nrf54l15dk/nrf54l15/cpuapp ../thigh -- -DEXTRA_CONF_FILE=debug.conf
```

The production build is 170 kB of 1524 kB flash and 31 kB of 256 kB RAM, with
no warnings and no static constructors.

## Wiring

All wiring is in [boards/nrf54l15dk_nrf54l15_cpuapp.overlay](boards/nrf54l15dk_nrf54l15_cpuapp.overlay). **The pin numbers are placeholders.**

| Part | Bus / pins |
|---|---|
| AD5941 | spi21 at 8 MHz, mode 0: SCK P1.03, MOSI P1.08, MISO P1.09, CS P1.10; RESET P0.01; 3-electrode cell on CE0 / RE0 / SE0 |
| LSM6DSO | i2c22 at 0x6A (SCL P1.04, SDA P1.05); INT1 P0.05 |
| Release motor | MOSFET gate P1.15 (`motor-gpios`); optional `endstop-gpios` |
| Vibration motor | P1.14 (`alert-gpios`), through its own transistor |
| Status LED | DK LED0 |
| Service button | DK BUTTON3 |

## Layout

```
thigh/
  CMakeLists.txt, Kconfig, prj.conf, debug.conf
  boards/…overlay          pin map
  src/main.cpp             event loop: histamine, fall, countdown, cancel, release, reports
  src/board.hpp            every devicetree spec in one place
  src/patch_link.*         Bluetooth link to the chest (peripheral, GATT server)
  src/drivers/ad5941.*     AD5941 amperometry driver
```

It shares the chest's algorithms and drivers (`../device/src`): histamine,
thigh_motion, tap_counter, deploy_controller, amperometry, the LSM6DSO,
motor and alert drivers, the deployment latch and the link's message format.
