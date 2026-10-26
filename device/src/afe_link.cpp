#include "afe_link.hpp"

#include <array>
#include <atomic>
#include <cstring>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/spinlock.h>

#if defined(CONFIG_APP_AFE_ON_FLPR)
#include <zephyr/ipc/ipc_service.h>
#else
#include "algo/ecg_qrs.hpp"
#include "algo/resp_rate.hpp"
#include "board.hpp"
#include "drivers/ads1292r.hpp"
#endif

LOG_MODULE_REGISTER(afe, CONFIG_APP_LOG_LEVEL);

namespace dev::afe {
namespace {

namespace msg = dev::afe_msg;

constinit k_spinlock lock{};
constinit Reading last{};
constinit Recording rec{};
TriggerFn on_trigger_fn;

void publish(const Reading &r)
{
	const k_spinlock_key_t key = k_spin_lock(&lock);
	last = r;
	k_spin_unlock(&lock, key);
}

} // namespace

Reading latest()
{
	const k_spinlock_key_t key = k_spin_lock(&lock);
	const Reading r = last;
	k_spin_unlock(&lock, key);
	return r;
}

Recording recording()
{
	const k_spinlock_key_t key = k_spin_lock(&lock);
	const Recording r = rec;
	k_spin_unlock(&lock, key);
	return r;
}

#if defined(CONFIG_APP_AFE_ON_FLPR)

// ---- the FLPR coprocessor reads the ADS1292R; we talk to it over IPC ----------

namespace {

ipc_ept ep;
std::atomic<bool> bound{false};
msg::Config config_msg;
k_work send_config_work;

void send_config(k_work *) { ipc_service_send(&ep, &config_msg, sizeof(config_msg)); }

void on_bound(void *)
{
	bound = true;
	k_work_submit(&send_config_work); // not from inside the IPC callback
	LOG_INF("AFE coprocessor linked");
}

void on_received(const void *data, size_t len, void *)
{
	if (len < 1) {
		return;
	}
	switch (static_cast<msg::Type>(*static_cast<const uint8_t *>(data))) {
	case msg::Type::Trigger:
		if (len == sizeof(msg::Trigger) && on_trigger_fn) {
			on_trigger_fn(static_cast<const msg::Trigger *>(data)->bits);
		}
		break;
	case msg::Type::Status:
		if (len == sizeof(msg::Status)) {
			const auto *s = static_cast<const msg::Status *>(data);
			publish({
				.hr_valid = (s->flags & msg::kHrValid) != 0,
				.hr_bpm = s->hr_bpm,
				.leads_off = (s->flags & msg::kLeadsOff) != 0,
				.rr_valid = (s->flags & msg::kRrValid) != 0,
				.rr_bpm = s->rr_bpm,
				.apnea = (s->flags & msg::kApnea) != 0,
				.updated_ms = k_uptime_get_32(),
			});
		}
		break;
	case msg::Type::Recording:
		if (len == sizeof(msg::Recording)) {
			const auto *r = static_cast<const msg::Recording *>(data);
			const k_spinlock_key_t key = k_spin_lock(&lock);
			// the coprocessor's RAM is part of the same address space
			rec = {reinterpret_cast<const uint8_t *>(static_cast<uintptr_t>(r->addr)), r->samples,
			       r->sps};
			k_spin_unlock(&lock, key);
		}
		break;
	default:
		break;
	}
}

template <typename T> void send(const T &m)
{
	if (bound) {
		ipc_service_send(&ep, &m, sizeof(m));
	}
}

} // namespace

int start(const algo::VitalTrigger::Config &trig, uint32_t apnea_ms, TriggerFn on_trigger)
{
	on_trigger_fn = on_trigger;
	config_msg = {
		.type = msg::Type::Config,
		.trig_apnea = trig.apnea,
		.reserved = {},
		.hr_high = trig.hr_high,
		.hr_low = trig.hr_low,
		.rr_high = trig.rr_high,
		.rr_low = trig.rr_low,
		.confirm_ms = trig.confirm_ms,
		.apnea_ms = apnea_ms,
	};
	k_work_init(&send_config_work, send_config);

	static const ipc_ept_cfg cfg = {
		.name = "afe",
		.cb = {.bound = on_bound, .received = on_received},
	};
	const device *ipc = DEVICE_DT_GET(DT_NODELABEL(ipc0));
	int err = ipc_service_open_instance(ipc);
	if (err && err != -EALREADY) {
		return err;
	}
	return ipc_service_register_endpoint(ipc, &ep, &cfg);
}

void stream(msg::StreamMode mode) { send(msg::Stream{.type = msg::Type::Stream, .mode = mode}); }

void rearm() { send(msg::Rearm{.type = msg::Type::Rearm}); }

#else

// ---- the ADS1292R is read on this core, by the afe thread ---------------------

namespace {

constinit drv::Ads1292r ads{board::kAdsSpi, board::kAdsDrdy, board::kAdsPwdn, board::kAdsStart};
constinit algo::QrsDetector qrs{};
constinit algo::RespRate resp{};
constinit algo::VitalTrigger trig{};

constexpr std::size_t kRecordSamples = CONFIG_APP_BURST_S * drv::Ads1292r::kSps;
constinit uint8_t record_buf[kRecordSamples * 6]{};
std::atomic<uint8_t> mode{static_cast<uint8_t>(msg::StreamMode::Off)};
std::atomic<bool> rearm_pending{false};

K_THREAD_STACK_DEFINE(afe_stack, CONFIG_APP_AFE_STACK_SIZE);
k_thread afe_thread;

void put24(uint8_t *p, int32_t v)
{
	p[0] = static_cast<uint8_t>(v >> 16);
	p[1] = static_cast<uint8_t>(v >> 8);
	p[2] = static_cast<uint8_t>(v);
}

void afe_loop(void *, void *, void *)
{
	constexpr std::size_t kBlock = drv::Ads1292r::kSps / 5; // 200 ms
	std::array<float, kBlock> ecg, bioz;
	std::size_t n = 0, blocks = 0, rec_n = 0;
	uint8_t loff = 0;
	int misses = 0;
	auto last_mode = msg::StreamMode::Off;

	for (;;) {
		if (ads.wait(K_MSEC(100)) != 0) {
			if (++misses >= 10) {
				// no DRDY for a second: restart the AFE
				LOG_WRN("ADS1292R stalled, re-initialising");
				ads.init();
				misses = 0;
			}
			continue;
		}
		misses = 0;
		drv::Ads1292r::Sample s;
		if (ads.read(s) != 0) {
			continue;
		}

		const auto m = static_cast<msg::StreamMode>(mode.load());
		if (m == msg::StreamMode::Record && last_mode != msg::StreamMode::Record) {
			rec_n = 0;
		}
		if (m == msg::StreamMode::Record && rec_n < kRecordSamples) {
			put24(&record_buf[rec_n * 6], s.ecg);
			put24(&record_buf[rec_n * 6 + 3], s.resp);
			if (++rec_n == kRecordSamples) {
				const k_spinlock_key_t key = k_spin_lock(&lock);
				rec = {record_buf, static_cast<uint32_t>(rec_n), drv::Ads1292r::kSps};
				k_spin_unlock(&lock, key);
			}
		}
		last_mode = m;

		ecg[n] = s.ecg * drv::Ads1292r::kEcgUvPerLsb;
		bioz[n] = static_cast<float>(s.resp);
		loff |= s.loff;
		if (++n < kBlock) {
			continue;
		}
		n = 0;
		// breathing is measured through the same two electrodes as the ECG
		const bool off = loff & (drv::Ads1292r::kLoffEcg | drv::Ads1292r::kLoffResp);
		loff = 0;
		qrs.process(ecg, off);
		resp.process(bioz, off);
		const Reading r{
			.hr_valid = qrs.hr_valid(),
			.hr_bpm = qrs.hr_bpm(),
			.leads_off = off,
			.rr_valid = resp.valid(),
			.rr_bpm = resp.rate_bpm(),
			.apnea = resp.apnea(),
			.updated_ms = k_uptime_get_32(),
		};
		publish(r);
		if (++blocks < 5) {
			continue;
		}
		blocks = 0;

		// ---- once a second: the triggers ----
		if (rearm_pending.exchange(false)) {
			trig.rearm();
		}
		const uint8_t fired = trig.update(
			{
				.hr_valid = r.hr_valid,
				.hr_bpm = r.hr_bpm,
				.rr_valid = r.rr_valid,
				.rr_bpm = r.rr_bpm,
				.apnea = r.apnea,
				.leads_off = off,
			},
			r.updated_ms);
		if (fired && on_trigger_fn) {
			on_trigger_fn(fired);
		}
	}
}

} // namespace

int start(const algo::VitalTrigger::Config &cfg, uint32_t apnea_ms, TriggerFn on_trigger)
{
	on_trigger_fn = on_trigger;
	qrs.reset();
	resp.reset({.apnea_ms = apnea_ms});
	trig.reset(cfg);
	const int err = ads.init();
	if (err) {
		return err;
	}
	k_thread_create(&afe_thread, afe_stack, K_THREAD_STACK_SIZEOF(afe_stack), afe_loop, nullptr, nullptr,
			nullptr, CONFIG_APP_AFE_THREAD_PRIORITY, K_FP_REGS, K_NO_WAIT);
	k_thread_name_set(&afe_thread, "afe");
	return 0;
}

void stream(msg::StreamMode m) { mode = static_cast<uint8_t>(m); }

void rearm() { rearm_pending = true; }

#endif

} // namespace dev::afe
