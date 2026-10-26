// EpiPatch thigh patch, nRF54L15: histamine sensing, fall detection, and the
// countdown, cancel and release of the epinephrine injector.
//
// The chest patch makes every decision. This patch
//   - reads histamine on the AD5941 (every 2 min, every 10 s while the chest
//     is reading or a countdown runs) and reports it, with its fall detection,
//     to the chest over Bluetooth;
//   - when the chest's decision says "trigger", runs the countdown on the
//     vibration motor, lets three taps on either patch cancel it, and then
//     runs the release motor through its MOSFET.
// Once a countdown has started it finishes here even if the link drops. With
// no link the patch does nothing new. After the last dose it does nothing more.
//
// Threads: main (prio 4) event loop; Bluetooth (patch_link.cpp).
// Timers (ISR): release-motor cut-off, vibration/LED patterns.
#include <algorithm>
#include <array>
#include <atomic>
#include <cstdint>
#include <span>

#include <zephyr/drivers/watchdog.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/spinlock.h>

#include "algo/deploy_controller.hpp"
#include "algo/histamine.hpp"
#include "algo/tap_counter.hpp"
#include "algo/thigh_motion.hpp"
#include "board.hpp"
#include "drivers/ad5941.hpp"
#include "drivers/alert.hpp"
#include "drivers/lsm6dso.hpp"
#include "drivers/motor.hpp"
#include "patch_link.hpp"
#include "persist.hpp"

// "device": the module the shared drivers in ../device/src/drivers log under
LOG_MODULE_REGISTER(device, CONFIG_APP_LOG_LEVEL);

namespace {

using namespace dev;
using algo::DeployController;
using algo::DeployState;

// ---- events into the main loop ---------------------------------------------

enum : uint32_t {
	kEvImu = BIT(0),
	kEvMotor = BIT(1),
	kEvDecision = BIT(2),
	kEvCancel = BIT(3),
	kEvReportNow = BIT(4),
	kEvAll = kEvImu | kEvMotor | kEvDecision | kEvCancel | kEvReportNow,
};

k_event g_events;

void post(void *bit) { k_event_post(&g_events, static_cast<uint32_t>(reinterpret_cast<uintptr_t>(bit))); }

void *ev_arg(uint32_t bit) { return reinterpret_cast<void *>(static_cast<uintptr_t>(bit)); }

// ---- hardware ----------------------------------------------------------------

constinit drv::Ad5941 afe{board::kAd5941Spi, board::kAd5941Reset};
constinit drv::Lsm6dso imu{board::kThighI2c, board::kThighInt};
constinit drv::Motor motor{board::kMotorGate, board::kMotorEndstop};
constinit drv::Alert vibration{board::kAlert, board::kStatusLed};

// ---- algorithms --------------------------------------------------------------

constinit algo::HistamineMonitor hist{};
constinit algo::ThighMotion motion{};
constinit algo::TapCounter taps{};
constinit DeployController deploy{};

// written by the Bluetooth thread
constinit k_spinlock rx_lock{};
constinit patch_msg::Decision pending_decision{};
bool decision_waiting;
std::atomic<bool> motor_result{false};

bool ok_afe, ok_imu;
uint32_t next_hist_ms, last_hist_ms, active_until_ms, report_seq;
DeployState reported_state = DeployState::Monitoring;
uint8_t reported_flags;

// ---- configuration from Kconfig ----------------------------------------------

void configure_algorithms(uint8_t deployments)
{
	// The histamine monitor works in "raw units": here the AD5941 current [nA]
	hist.reset({
		.offset_mv = static_cast<float>(CONFIG_APP_HIST_OFFSET_NA),
		.nm_per_mv = CONFIG_APP_HIST_NM_PER_UA / 1000.0f,
		.min_mv = static_cast<float>(CONFIG_APP_HIST_MIN_NA),
		.max_mv = static_cast<float>(CONFIG_APP_HIST_MAX_NA),
		.rise_nm = static_cast<float>(CONFIG_APP_HIST_RISE_NM),
		.ratio = CONFIG_APP_HIST_RATIO_PCT / 100.0f,
		.abs_nm = static_cast<float>(CONFIG_APP_HIST_ABS_NM),
		.period_ms = CONFIG_APP_HIST_IDLE_PERIOD_S * 1000u,
		.warmup_ms = CONFIG_APP_HIST_WARMUP_S * 1000u,
		.confirm = CONFIG_APP_HIST_CONFIRM,
	});
	motion.reset({
		.up = static_cast<algo::Axis>(std::clamp(CONFIG_APP_THIGH_UP_AXIS, 0, 5)),
		.fall_confirm_ms = CONFIG_APP_FALL_CONFIRM_S * 1000u,
		.still_g = CONFIG_APP_STILL_MG / 1000.0f,
	});
	taps.reset({.taps = CONFIG_APP_TAP_COUNT, .window_ms = CONFIG_APP_TAP_WINDOW_MS});
	deploy.reset(
		{
			.countdown_ms = CONFIG_APP_COUNTDOWN_S * 1000u,
			.cooldown_ms = CONFIG_APP_COOLDOWN_S * 1000u,
			.max_deployments = CONFIG_APP_MAX_DEPLOYMENTS,
			.redose_ms = CONFIG_APP_REDOSE_S * 1000u,
		},
		deployments);
}

// ---- callbacks (Bluetooth thread / ISRs) ---------------------------------------

void on_decision(const patch_msg::Decision &d)
{
	const k_spinlock_key_t key = k_spin_lock(&rx_lock);
	pending_decision = d;
	decision_waiting = true;
	k_spin_unlock(&rx_lock, key);
	k_event_post(&g_events, kEvDecision);
}

void on_cancel() { k_event_post(&g_events, kEvCancel); }

void on_report_now() { k_event_post(&g_events, kEvReportNow); }

void on_motor_done(bool ok, void *)
{
	motor_result = ok;
	k_event_post(&g_events, kEvMotor);
}

// ---- reporting to the chest ------------------------------------------------------

uint8_t report_flags(uint32_t now)
{
	return static_cast<uint8_t>((hist.valid() ? patch_msg::kHistValid : 0) |
				    (hist.elevated() ? patch_msg::kHistElevated : 0) |
				    (motion.fall() ? patch_msg::kFall : 0) |
				    (ok_imu && motion.valid(now) ? patch_msg::kImuValid : 0));
}

void send_report(uint32_t now)
{
	reported_flags = report_flags(now);
	reported_state = deploy.state();
	link::send_report({
		.type = patch_msg::Type::Report,
		.flags = reported_flags,
		.deploy_state = static_cast<uint8_t>(reported_state),
		.deployments = deploy.deployments(),
		.hist_nm = hist.nm(),
		.hist_baseline_nm = hist.baseline_nm(),
		.activity_g = motion.activity_g(),
		.seq = ++report_seq,
	});
}

// report as soon as anything the chest decides on changes
void report_if_changed(uint32_t now)
{
	if (report_flags(now) != reported_flags || deploy.state() != reported_state) {
		send_report(now);
	}
}

// ---- main-loop handlers ----------------------------------------------------------

void handle_action(DeployController::Action act, uint32_t now)
{
	using Action = DeployController::Action;
	switch (act) {
	case Action::StartAlert:
		motor.arm();
		vibration.play(drv::Alert::kCountdown, true);
		LOG_WRN("COUNTDOWN: releasing in %d s unless cancelled", CONFIG_APP_COUNTDOWN_S);
		break;
	case Action::Cancelled:
		motor.disarm();
		vibration.play(drv::Alert::kCancelAck, true);
		LOG_WRN("cancelled");
		break;
	case Action::Fire: {
		LOG_WRN("RELEASING");
		const int err = motor.fire(CONFIG_APP_MOTOR_RUN_MS, IS_ENABLED(CONFIG_APP_MOTOR_REQUIRE_ENDSTOP));
		if (err) {
			LOG_ERR("release motor failed (%d)", err);
			handle_action(deploy.motor_done(false, now), now);
		}
		break;
	}
	case Action::Deployed:
	case Action::Fault:
		persist::save(deploy.deployments());
		vibration.play(act == Action::Deployed ? drv::Alert::kDeployed : drv::Alert::kFault, true);
		LOG_WRN("%s (deployments %d)", act == Action::Deployed ? "deployed" : "RELEASE MOTOR FAULT",
			deploy.deployments());
		break;
	case Action::Resume:
		vibration.play(drv::Alert::kOff, false);
		break;
	case Action::None:
		break;
	}
	if (act != Action::None) {
		send_report(now);
	}
}

bool counting()
{
	const DeployState s = deploy.state();
	return s == DeployState::Alert || s == DeployState::Deploying || s == DeployState::Cancelled;
}

void apply_decision(uint32_t now)
{
	patch_msg::Decision d;
	{
		const k_spinlock_key_t key = k_spin_lock(&rx_lock);
		if (!decision_waiting) {
			k_spin_unlock(&rx_lock, key);
			return;
		}
		d = pending_decision;
		decision_waiting = false;
		k_spin_unlock(&rx_lock, key);
	}
	algo::Assessment a{};
	a.findings = d.findings;
	a.trigger = d.trigger != 0;
	a.severe = d.severe != 0;
	if (a.trigger) {
		LOG_WRN("chest decision: trigger, findings 0x%03x%s", a.findings, a.severe ? " (severe)" : "");
	}
	// the chest is reading: keep histamine fresh for a while
	active_until_ms = now + 30000;
	handle_action(deploy.update(a, now), now);
}

void cancel(uint32_t now, const char *from)
{
	const auto act = deploy.cancel(now);
	if (act != DeployController::Action::None) {
		LOG_WRN("cancel from the %s", from);
	}
	handle_action(act, now);
}

void read_fifo(uint32_t now)
{
	std::array<algo::Vec3, drv::Lsm6dso::kMaxFifo> buf;
	const int n = imu.read_fifo(buf);
	if (n > 0) {
		motion.samples(std::span(buf).first(static_cast<std::size_t>(n)), now, drv::Lsm6dso::kFifoDtMs);
	}
}

void handle_imu(uint32_t now)
{
	for (int i = 0; i < 4; ++i) {
		const int ev = imu.events();
		if (ev < 0) {
			return;
		}
		if ((ev & drv::Lsm6dso::kTap) && taps.on_tap(now)) {
			cancel(now, "thigh");
		}
		if (ev & drv::Lsm6dso::kImpact) {
			motion.impact(now);
		}
		if (ev & drv::Lsm6dso::kFreeFall) {
			motion.freefall(now);
		}
		if (ev & drv::Lsm6dso::kFifo) {
			read_fifo(now);
		}
		if (!imu.irq_active()) {
			return;
		}
	}
}

void read_histamine(uint32_t now)
{
	float na = 0.0f;
	const bool ok = ok_afe && afe.read_na(na) == 0;
	hist.sample(ok ? na : static_cast<float>(CONFIG_APP_HIST_MIN_NA) - 1.0f, now);
	last_hist_ms = now;
	const bool busy = counting() || static_cast<int32_t>(active_until_ms - now) > 0;
	next_hist_ms = now + (busy ? CONFIG_APP_HIST_ACTIVE_PERIOD_S : CONFIG_APP_HIST_IDLE_PERIOD_S) * 1000u;
}

// ---- boot helpers ------------------------------------------------------------------

// Holding the service button for 3 s at power-up clears the "already
// deployed" latch after the injector has been replaced.
bool service_clear_requested()
{
	if (!DT_NODE_HAS_PROP(DEV_USER_NODE, service_gpios) ||
	    gpio_pin_configure_dt(&board::kService, GPIO_INPUT)) {
		return false;
	}
	for (int i = 0; i < 30; ++i) {
		if (gpio_pin_get_dt(&board::kService) <= 0) {
			return false;
		}
		k_msleep(100);
	}
	return true;
}

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

} // namespace

int main()
{
	k_event_init(&g_events);
	vibration.init();
	if (motor.init(on_motor_done, nullptr) != 0) {
		LOG_ERR("release motor gate GPIO not ready");
	}

	uint8_t deployments = 0;
	if (persist::load(deployments) != 0) {
		LOG_WRN("deployment latch not readable; assuming none");
	}
	if (deployments > 0 && service_clear_requested()) {
		deployments = 0;
		persist::save(0);
		vibration.play(drv::Alert::kCancelAck, true);
		LOG_WRN("deployment latch cleared");
	}
	configure_algorithms(deployments);

	ok_afe = afe.init({
			 .vzero_mv = static_cast<float>(CONFIG_APP_HIST_VZERO_MV),
			 .bias_mv = static_cast<float>(CONFIG_APP_HIST_BIAS_MV),
			 .rtia = CONFIG_APP_HIST_RTIA,
			 .pga = CONFIG_APP_HIST_PGA,
			 .averages = CONFIG_APP_HIST_AVERAGES,
		 }) == 0;
	ok_imu = imu.init(
			 {
				 .odr_hz = static_cast<uint16_t>(CONFIG_APP_TAP_ODR_HZ),
				 .low_power = false,
				 .tap = true,
				 .tap_mg = CONFIG_APP_TAP_THRESHOLD_MG,
				 .impact_mg = CONFIG_APP_IMPACT_MG,
				 .freefall_mg = CONFIG_APP_FREEFALL_MG,
				 .freefall_ms = CONFIG_APP_FREEFALL_MS,
				 .fifo_wtm = CONFIG_APP_IMU_FIFO_WTM,
			 },
			 post, ev_arg(kEvImu)) == 0;
	const int bt_err = link::start({.decision = on_decision, .cancel = on_cancel, .report_now = on_report_now});
	LOG_INF("thigh: histamine %d imu %d bluetooth %d deployments %d", ok_afe, ok_imu, bt_err == 0,
		deployments);
	if (deploy.state() == DeployState::Deployed) {
		vibration.play(drv::Alert::kDeployed, false); // LED only after a reboot
	}

	Watchdog wdt;
	wdt.start();

	uint32_t now = k_uptime_get_32();
	next_hist_ms = now;
	uint32_t next_tick = now + 1000;

	for (;;) {
		// 1 s ticks while a countdown runs; otherwise sleep until the next
		// histamine reading, waking at least twice per watchdog period
		now = k_uptime_get_32();
		uint32_t wait = CONFIG_APP_WATCHDOG_MS / 2;
		const auto to_hist = static_cast<int32_t>(next_hist_ms - now);
		wait = std::min<uint32_t>(wait, to_hist > 0 ? static_cast<uint32_t>(to_hist) : 0);
		if (counting()) {
			const auto to_tick = static_cast<int32_t>(next_tick - now);
			wait = std::min<uint32_t>(wait, to_tick > 0 ? static_cast<uint32_t>(to_tick) : 0);
		}
		const uint32_t ev = k_event_wait(&g_events, kEvAll, false, K_MSEC(wait));
		k_event_clear(&g_events, ev);
		now = k_uptime_get_32();

		if (ev & kEvImu) {
			handle_imu(now);
		}
		if (ev & kEvMotor) {
			handle_action(deploy.motor_done(motor_result, now), now);
		}
		if (ev & kEvCancel) {
			cancel(now, "chest");
		}
		if (ev & kEvDecision) {
			apply_decision(now);
		}
		if (ev & kEvReportNow) {
			active_until_ms = now + 30000;
			if (now - last_hist_ms > 5000) {
				read_histamine(now);
			}
			send_report(now);
		}

		if (static_cast<int32_t>(now - next_hist_ms) >= 0) {
			read_histamine(now);
			send_report(now); // also the link's heartbeat
		}
		if (counting()) {
			if (static_cast<int32_t>(now - next_tick) >= 0) {
				// countdown, cooldown and release timing run here, link or no link
				handle_action(deploy.update({}, now), now);
				if (deploy.state() == DeployState::Alert && deploy.countdown_left_ms(now) <= 5000 &&
				    vibration.current() != &drv::Alert::kCountdownEnd) {
					vibration.play(drv::Alert::kCountdownEnd, true);
				}
				next_tick = now + 1000;
			}
		} else {
			next_tick = now + 1000;
		}
		report_if_changed(now);
		wdt.feed();
	}
	return 0;
}
