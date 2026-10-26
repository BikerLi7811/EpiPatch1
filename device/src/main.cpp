// EpiPatch chest patch, nRF54L15: senses, analyses and decides.
//
// Who does what
//   FLPR    (low-power RISC-V core, flpr/) ADS1292R at 125 Hz -> heart rate,
//           breathing rate and the two vital-sign triggers. It wakes the main
//           core only when a trigger fires. (With -DAFE_CORE=app an afe thread
//           on the main core does the same; see afe_link.cpp.)
//   main    (Arm Cortex-M33, prio 4) event loop: chest IMU interrupts at any
//           time (tap / impact / free-fall / FIFO watermark), the sampling
//           schedule, the analysis and the decision
//   wheeze  (prio 5) PDM mic -> wheeze CNN (wheeze module), listening when asked
//   BT      Bluetooth LE link to the thigh patch (patch_link.cpp)
//
// The thigh patch (../thigh) owns the histamine sensor, its own fall
// detection, the vibration motor and the release motor. It reports its data
// here; the chest folds it into the decision and sends the verdict back. The
// thigh then runs the countdown, the tap cancel and the release itself.
//
// Sampling schedule (algo/sampling_schedule.hpp)
//   every 2 min      spot read: PPG, mic and thigh histamine for 15 s, decision every second
//   ECG/Bio-Z trigger burst: everything on for 1 min, every sample kept, then
//                    the whole minute is analysed (burst.hpp)
//   countdown on     everything on, decision every second, until it is over
#include <algorithm>
#include <array>
#include <atomic>
#include <cstdint>
#include <span>

#include <zephyr/drivers/watchdog.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/spinlock.h>

#include <wheeze/wheeze.h>

#include "afe_link.hpp"
#include "algo/anaphylaxis.hpp"
#include "algo/body_motion.hpp"
#include "algo/ppg.hpp"
#include "algo/sampling_schedule.hpp"
#include "algo/tap_counter.hpp"
#include "algo/thigh_view.hpp"
#include "algo/vital_trigger.hpp"
#include "board.hpp"
#include "burst.hpp"
#include "drivers/alert.hpp"
#include "drivers/lsm6dso.hpp"
#include "drivers/max30101.hpp"
#include "patch_link.hpp"

LOG_MODULE_REGISTER(device, CONFIG_APP_LOG_LEVEL);

namespace {

using namespace dev;
using algo::DeployState;
using algo::SamplingSchedule;
using afe_msg::StreamMode;

// ---- events into the main loop ---------------------------------------------

enum : uint32_t {
	kEvPpg = BIT(0),
	kEvChest = BIT(1),
	kEvTrigger = BIT(2),
	kEvReport = BIT(3),
	kEvAll = kEvPpg | kEvChest | kEvTrigger | kEvReport,
};

k_event g_events;

void post(void *bit) { k_event_post(&g_events, static_cast<uint32_t>(reinterpret_cast<uintptr_t>(bit))); }

void *ev_arg(uint32_t bit) { return reinterpret_cast<void *>(static_cast<uintptr_t>(bit)); }

// ---- hardware ----------------------------------------------------------------

// constinit: the compiler guarantees no constructor runs at boot
constinit drv::Max30101 ppg{board::kPpgI2c, board::kPpgInt};
constinit drv::Lsm6dso imu_chest{board::kChestI2c, board::kChestInt};
constinit drv::Alert alert{board::kAlert, board::kStatusLed};

// ---- algorithms --------------------------------------------------------------

// value-initialised ({}), so they are zero-filled at link time instead of
// constructed at boot; each is configured by reset() in configure_algorithms()
constinit algo::PpgProcessor ppg_proc{};
constinit algo::BodyMotion motion{};
constinit algo::TapCounter taps{};
constinit algo::AnaphylaxisEngine engine{};
constinit SamplingSchedule schedule{};
constinit algo::ThighView thigh{};

std::atomic<uint8_t> pending_trigger{0}; // VitalTrigger bits, set by the AFE side

// latest thigh report, written by the Bluetooth thread
constinit k_spinlock report_lock{};
constinit patch_msg::Report pending_report{};
bool report_waiting;

bool ok_ppg, ok_chest;
bool ppg_on;
uint32_t last_ppg_ms, next_hist_ms;
DeployState shown_state = DeployState::Monitoring;

// ---- configuration from Kconfig ----------------------------------------------

algo::Axis axis(int v) { return static_cast<algo::Axis>(std::clamp(v, 0, 5)); }

algo::VitalTrigger::Config trigger_config()
{
	return {
		.hr_high = static_cast<float>(CONFIG_APP_TRIG_HR_HIGH_BPM),
		.hr_low = static_cast<float>(CONFIG_APP_TRIG_HR_LOW_BPM),
		.rr_high = static_cast<float>(CONFIG_APP_TRIG_RR_HIGH_BPM),
		.rr_low = static_cast<float>(CONFIG_APP_TRIG_RR_LOW_BPM),
		.apnea = IS_ENABLED(CONFIG_APP_TRIG_APNEA),
		.confirm_ms = CONFIG_APP_TRIG_CONFIRM_S * 1000u,
	};
}

void configure_algorithms()
{
	ppg_proc.reset({});
	motion.reset({
		.chest_up = axis(CONFIG_APP_CHEST_UP_AXIS),
		.collapse_transition_ms = CONFIG_APP_COLLAPSE_TRANSITION_MS,
		.collapse_still_ms = CONFIG_APP_COLLAPSE_STILL_S * 1000u,
		.fall_confirm_ms = CONFIG_APP_FALL_CONFIRM_S * 1000u,
		.still_g = CONFIG_APP_STILL_MG / 1000.0f,
	});
	taps.reset({.taps = CONFIG_APP_TAP_COUNT, .window_ms = CONFIG_APP_TAP_WINDOW_MS});
	engine.reset({
		.active_g = CONFIG_APP_ACTIVE_MG / 1000.0f,
		.rest_g = CONFIG_APP_STILL_MG / 1000.0f,
		.wheeze_min_s = CONFIG_APP_WHEEZE_MIN_S,
		.wheeze_window_ms = CONFIG_APP_WHEEZE_WINDOW_S * 1000u,
		.wheeze_recent_ms = CONFIG_APP_WHEEZE_RECENT_S * 1000u,
		.rr_high_bpm = static_cast<float>(CONFIG_APP_RR_HIGH_BPM),
		.rr_high_hold_ms = CONFIG_APP_RR_HIGH_HOLD_S * 1000u,
		.rr_low_bpm = static_cast<float>(CONFIG_APP_RR_LOW_BPM),
		.rr_low_hold_ms = CONFIG_APP_RR_LOW_HOLD_S * 1000u,
		.hr_high_bpm = static_cast<float>(CONFIG_APP_HR_HIGH_BPM),
		.hr_rise_bpm = static_cast<float>(CONFIG_APP_HR_RISE_BPM),
		.hr_hold_ms = CONFIG_APP_HR_HOLD_S * 1000u,
		.pi_drop_ratio = CONFIG_APP_PI_DROP_PCT / 100.0f,
		.pi_hold_ms = CONFIG_APP_PI_HOLD_S * 1000u,
		.exposure_hold_ms = CONFIG_APP_HIST_EXPOSURE_MIN * 60000u,
		.rule_exposure = IS_ENABLED(CONFIG_APP_RULE_EXPOSURE),
		.rule_resp_collapse = IS_ENABLED(CONFIG_APP_RULE_RESP_COLLAPSE),
	});
	thigh.reset({.fresh_ms = CONFIG_APP_THIGH_FRESH_S * 1000u});
}

// ---- callbacks (other threads / ISRs) ------------------------------------------

void on_trigger(uint8_t bits)
{
	pending_trigger.fetch_or(bits);
	k_event_post(&g_events, kEvTrigger);
}

void on_report(const patch_msg::Report &r)
{
	const k_spinlock_key_t key = k_spin_lock(&report_lock);
	pending_report = r;
	report_waiting = true;
	k_spin_unlock(&report_lock, key);
	k_event_post(&g_events, kEvReport);
}

// ---- main-loop handlers ----------------------------------------------------------

// the chest's buzzer/LED follow the thigh's countdown
void mirror_thigh_state(DeployState s)
{
	if (s == shown_state) {
		return;
	}
	shown_state = s;
	switch (s) {
	case DeployState::Alert:
		alert.play(drv::Alert::kCountdown, true);
		LOG_WRN("thigh patch: countdown running");
		break;
	case DeployState::Cancelled:
		alert.play(drv::Alert::kCancelAck, true);
		LOG_WRN("thigh patch: cancelled");
		break;
	case DeployState::Deploying:
		LOG_WRN("thigh patch: releasing");
		break;
	case DeployState::Deployed:
		alert.play(drv::Alert::kDeployed, false);
		LOG_WRN("thigh patch: deployed");
		break;
	case DeployState::Fault:
		alert.play(drv::Alert::kFault, true);
		LOG_ERR("thigh patch: release motor fault");
		break;
	case DeployState::Monitoring:
		alert.play(drv::Alert::kOff, false);
		break;
	}
}

void apply_report(uint32_t now)
{
	patch_msg::Report r;
	{
		const k_spinlock_key_t key = k_spin_lock(&report_lock);
		if (!report_waiting) {
			k_spin_unlock(&report_lock, key);
			return;
		}
		r = pending_report;
		report_waiting = false;
		k_spin_unlock(&report_lock, key);
	}
	thigh.on_report(r, now);
	mirror_thigh_state(thigh.state());
}

bool alerting(uint32_t now) { return thigh.alerting(now); }

// the verdict goes to the thigh, which runs the countdown and release
void decide(const algo::Assessment &a)
{
	link::send_decision(a);
	if (a.trigger) {
		LOG_WRN("DECISION: findings 0x%03x%s -> thigh patch", a.findings, a.severe ? " (severe)" : "");
	}
}

void read_imu_fifo(uint32_t now)
{
	std::array<algo::Vec3, drv::Lsm6dso::kMaxFifo> buf;
	const int n = imu_chest.read_fifo(buf);
	if (n <= 0) {
		return;
	}
	const auto s = std::span(buf).first(static_cast<std::size_t>(n));
	// posture and collapse always run live, so they work between scheduled reads
	motion.chest(s, now, drv::Lsm6dso::kFifoDtMs);
	if (schedule.mode() == SamplingSchedule::Mode::Burst) {
		burst::add_imu(true, s);
	}
}

void handle_imu(uint32_t now)
{
	for (int i = 0; i < 4; ++i) {
		const int ev = imu_chest.events();
		if (ev < 0) {
			return;
		}
		if ((ev & drv::Lsm6dso::kTap) && taps.on_tap(now)) {
			link::send_cancel(); // the thigh runs the countdown
			LOG_WRN("cancel tapped on the chest");
		}
		if (ev & drv::Lsm6dso::kImpact) {
			motion.impact(now);
		}
		if (ev & drv::Lsm6dso::kFreeFall) {
			motion.freefall(now);
		}
		if (ev & drv::Lsm6dso::kFifo) {
			read_imu_fifo(now);
		}
		if (!imu_chest.irq_active()) {
			return;
		}
	}
}

void read_ppg(uint32_t now)
{
	std::array<uint32_t, drv::Max30101::kFifoDepth> red, ir;
	const int n = ppg.read(red, ir);
	if (n > 0) {
		const auto cnt = static_cast<std::size_t>(n);
		const auto r = std::span(red).first(cnt);
		const auto i = std::span(ir).first(cnt);
		if (schedule.mode() == SamplingSchedule::Mode::Burst) {
			burst::add_ppg(r, i); // analysed at the end of the minute
		} else {
			ppg_proc.process(r, i);
		}
	}
	last_ppg_ms = now;
}

// histamine is measured on the thigh: ask for a fresh reading
void request_histamine(uint32_t now)
{
	link::request_report();
	next_hist_ms = now + CONFIG_APP_HIST_PERIOD_S * 1000u;
}

// PPG and mic on for listen_ms (0 = until sensors_off(), renewed by tick)
void sensors_on(uint32_t now, uint32_t listen_ms)
{
	if (ok_ppg && !ppg_on) {
		ppg_on = ppg.wake() == 0;
		ppg_proc.reset({});
		last_ppg_ms = now;
	}
	wheeze_listen(listen_ms ? listen_ms : 3000);
	request_histamine(now);
}

void sensors_off()
{
	if (ppg_on) {
		ppg.shutdown();
		ppg_on = false;
	}
	afe::stream(StreamMode::Off);
}

Vitals snapshot(uint32_t now)
{
	const afe::Reading r = afe::latest();
	const bool afe_fresh = r.updated_ms != 0 && now - r.updated_ms < 2000;

	wheeze_stats ws{};
	wheeze_get_stats(&ws);

	Vitals v{};
	v.hr_valid = afe_fresh && r.hr_valid;
	v.hr_bpm = r.hr_bpm;
	v.ecg_leads_off = r.leads_off;
	v.rr_valid = afe_fresh && r.rr_valid;
	v.rr_bpm = r.rr_bpm;
	v.apnea = afe_fresh && r.apnea;
	v.ppg_valid = ppg_on && ppg_proc.valid();
	v.pi_pct = ppg_proc.pi_pct();
	v.pulse_bpm = ppg_proc.pulse_bpm();
	v.spo2_est = ppg_proc.spo2_est();
	v.wheezing = ws.wheezing;
	v.wheeze_events = ws.events;
	v.posture = motion.posture();
	v.activity_g = motion.activity_g();
	v.collapsed = motion.collapsed();
	v.fall = motion.fall();
	thigh.merge(v, now); // histamine, thigh fall and activity
	return v;
}

void log_vitals(const Vitals &v, uint16_t findings)
{
	LOG_INF("HR %s%d RR %s%d PI %d.%02d%% post %d act %dmg hist %s%d/%d nM wz %d find 0x%03x "
		"thigh %d mode %d",
		v.hr_valid ? "" : "~", static_cast<int>(v.hr_bpm), v.rr_valid ? "" : "~",
		static_cast<int>(v.rr_bpm), static_cast<int>(v.pi_pct), static_cast<int>(v.pi_pct * 100) % 100,
		static_cast<int>(v.posture), static_cast<int>(v.activity_g * 1000), v.hist_valid ? "" : "~",
		static_cast<int>(v.hist_nm), static_cast<int>(v.hist_baseline_nm), v.wheezing, findings,
		static_cast<int>(shown_state), static_cast<int>(schedule.mode()));
}

// once a second while the schedule is awake (spot, burst or countdown)
void tick(uint32_t now)
{
	if (ok_chest) {
		read_imu_fifo(now);
	}
	if (ppg_on && now - last_ppg_ms > 2000) {
		read_ppg(now); // missed interrupt: poll
	}
	if (static_cast<int32_t>(now - next_hist_ms) >= 0) {
		request_histamine(now);
	}
	motion.update(now);
	const Vitals v = snapshot(now);

	if (schedule.mode() == SamplingSchedule::Mode::Burst) {
		burst::add_second(v, now); // decided at the end of the minute
		return;
	}
	if (schedule.mode() == SamplingSchedule::Mode::Active) {
		wheeze_listen(3000); // keep the mic on until the countdown is over
	}
	const algo::Assessment a = engine.update(v, now);
	decide(a);

	static uint32_t log_div;
	if (++log_div >= 5) {
		log_div = 0;
		log_vitals(v, a.findings);
	}
}

void end_burst()
{
	const burst::Info info = burst::info();
	const algo::Assessment a = burst::analyse(engine, ppg_proc);
	const afe::Recording rec = afe::recording();
	LOG_INF("burst analysed: %u s, %u PPG, %u IMU, %u ECG/Bio-Z samples; findings 0x%03x%s",
		static_cast<unsigned>(info.seconds), static_cast<unsigned>(info.ppg_samples),
		static_cast<unsigned>(info.chest_samples), static_cast<unsigned>(rec.samples), a.findings,
		a.trigger ? " -> TRIGGER" : "");
	decide(a);
	afe::rearm();
}

void handle_schedule(SamplingSchedule::Action act, uint32_t now)
{
	using A = SamplingSchedule::Action;
	switch (act) {
	case A::StartSpot:
		sensors_on(now, CONFIG_APP_SPOT_WINDOW_S * 1000u);
		afe::stream(StreamMode::Status);
		break;
	case A::StartBurst:
		burst::begin(now);
		sensors_on(now, CONFIG_APP_BURST_S * 1000u);
		afe::stream(StreamMode::Record);
		break;
	case A::StartActive:
		sensors_on(now, 0);
		afe::stream(StreamMode::Status);
		break;
	case A::EndSpot:
		sensors_off();
		break;
	case A::EndActive:
		sensors_off();
		afe::rearm();
		break;
	case A::EndBurst:
		sensors_off();
		end_burst();
		break;
	case A::None:
		break;
	}
}

// ---- boot helpers ------------------------------------------------------------------

struct Watchdog {
	const device *dev = nullptr;
	int channel = -1;

	void start()
	{
#if DT_NODE_HAS_STATUS(DT_ALIAS(watchdog0), okay)
		dev = DEVICE_DT_GET(DT_ALIAS(watchdog0));
		if (!device_is_ready(dev)) {
			dev = nullptr;
			return;
		}
		wdt_timeout_cfg cfg{};
		cfg.window.min = 0;
		cfg.window.max = CONFIG_APP_WATCHDOG_MS;
		cfg.flags = WDT_FLAG_RESET_SOC;
		channel = wdt_install_timeout(dev, &cfg);
		if (channel < 0 || wdt_setup(dev, WDT_OPT_PAUSE_HALTED_BY_DBG) != 0) {
			dev = nullptr;
		}
#endif
	}
	void feed()
	{
		if (dev) {
			wdt_feed(dev, channel);
		}
	}
};

drv::Lsm6dso::Config imu_config()
{
	return {
		.odr_hz = static_cast<uint16_t>(CONFIG_APP_TAP_ODR_HZ),
		.low_power = false,
		.tap = true,
		.tap_mg = CONFIG_APP_TAP_THRESHOLD_MG,
		.impact_mg = CONFIG_APP_IMPACT_MG,
		.freefall_mg = CONFIG_APP_FREEFALL_MG,
		.freefall_ms = CONFIG_APP_FREEFALL_MS,
		.fifo_wtm = CONFIG_APP_IMU_FIFO_WTM,
	};
}

} // namespace

int main()
{
	k_event_init(&g_events);
	alert.init();
	configure_algorithms();

	const int afe_err = afe::start(trigger_config(), CONFIG_APP_APNEA_S * 1000u, on_trigger);
	ok_ppg = ppg.init(CONFIG_APP_PPG_RED_PA, CONFIG_APP_PPG_IR_PA, post, ev_arg(kEvPpg)) == 0;
	if (ok_ppg) {
		ppg.shutdown(); // until the first scheduled read
	}
	ok_chest = imu_chest.init(imu_config(), post, ev_arg(kEvChest)) == 0;
	const int bt_err = link::start(on_report);
	LOG_INF("chest: ecg/bioz %d (%s) ppg %d imu %d bluetooth %d", afe_err == 0,
		IS_ENABLED(CONFIG_APP_AFE_ON_FLPR) ? "FLPR" : "app core", ok_ppg, ok_chest, bt_err == 0);

	const int werr = wheeze_start(nullptr, nullptr);
	if (werr) {
		LOG_ERR("wheeze detector not started (%d)", werr);
	}

	Watchdog wdt;
	wdt.start();

	uint32_t now = k_uptime_get_32();
	schedule.reset(
		{
			.period_ms = CONFIG_APP_SPOT_PERIOD_S * 1000u,
			.spot_ms = CONFIG_APP_SPOT_WINDOW_S * 1000u,
			.burst_ms = CONFIG_APP_BURST_S * 1000u,
		},
		now);
	next_hist_ms = now;
	uint32_t next_tick = now + 1000;

	for (;;) {
		// sleep until an interrupt, the next tick (when awake) or the next
		// scheduled read (when idle); wake at least twice per watchdog period
		now = k_uptime_get_32();
		uint32_t wait = CONFIG_APP_WATCHDOG_MS / 2;
		if (schedule.awake()) {
			const auto to_tick = static_cast<int32_t>(next_tick - now);
			wait = std::min<uint32_t>(wait, to_tick > 0 ? static_cast<uint32_t>(to_tick) : 0);
		} else {
			wait = std::min(wait, schedule.idle_wait_ms(now));
		}
		const uint32_t ev = k_event_wait(&g_events, kEvAll, false, K_MSEC(wait));
		k_event_clear(&g_events, ev);
		now = k_uptime_get_32();

		if (ev & kEvChest) {
			handle_imu(now);
		}
		if ((ev & kEvPpg) && ppg_on) {
			read_ppg(now);
		}
		if (ev & kEvReport) {
			apply_report(now);
		}
		if (ev & kEvTrigger) {
			const uint8_t bits = pending_trigger.exchange(0);
			const auto act = schedule.trigger(now);
			LOG_INF("ECG/Bio-Z trigger 0x%02x%s", bits,
				act == SamplingSchedule::Action::StartBurst ? ": recording 1 min" : "");
			// otherwise a burst or a countdown already has everything on; the
			// trigger is re-armed when that ends
			if (act == SamplingSchedule::Action::StartBurst) {
				handle_schedule(act, now);
				next_tick = now + 1000;
			}
		}

		const bool was_awake = schedule.awake();
		for (auto act = schedule.update(now, alerting(now)); act != SamplingSchedule::Action::None;
		     act = schedule.update(now, alerting(now))) {
			handle_schedule(act, now);
		}
		if (schedule.awake()) {
			if (!was_awake) {
				next_tick = now + 1000;
			}
			if (static_cast<int32_t>(now - next_tick) >= 0) {
				tick(now);
				next_tick += 1000;
				if (static_cast<int32_t>(now - next_tick) >= 0) {
					next_tick = now + 1000; // fell behind; don't burst
				}
			}
		}
		wdt.feed();
	}
	return 0;
}
