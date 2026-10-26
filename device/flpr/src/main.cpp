// FLPR coprocessor (RISC-V): owns the ADS1292R so the application core can
// sleep. Reads every sample (125 Hz), computes heart rate and breathing rate
// with the same algorithm code as the application core, and runs the two
// vital-sign triggers. When one fires it sends a Trigger message; the
// application core then records and analyses a minute of everything.
// On request it sends a Status every second and records a full minute of raw
// ECG + Bio-Z samples into its own RAM.
#include <algorithm>
#include <array>
#include <atomic>
#include <cstdint>
#include <cstring>

#include <zephyr/ipc/ipc_service.h>
#include <zephyr/kernel.h>
#include <zephyr/spinlock.h>

#include "afe_msgs.hpp"
#include "algo/ecg_qrs.hpp"
#include "algo/resp_rate.hpp"
#include "algo/vital_trigger.hpp"
#include "drivers/ads1292r.hpp"

namespace {

using namespace dev;
namespace msg = dev::afe_msg;

#define ADS_NODE DT_NODELABEL(ads1292r)

constinit drv::Ads1292r ads{
	SPI_DT_SPEC_GET(ADS_NODE, SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_MODE_CPHA),
	GPIO_DT_SPEC_GET(ADS_NODE, drdy_gpios),
	GPIO_DT_SPEC_GET_OR(ADS_NODE, pwdn_gpios, {}),
	GPIO_DT_SPEC_GET_OR(ADS_NODE, start_gpios, {}),
};
constinit algo::QrsDetector qrs{};
constinit algo::RespRate resp{};
constinit algo::VitalTrigger trig{};

// ---- one minute of raw samples, readable by the application core ------------

constexpr std::size_t kRecordSamples = CONFIG_FLPR_RECORD_S * drv::Ads1292r::kSps;
constexpr std::size_t kBytesPerSample = 6;
uint8_t record_buf[kRecordSamples * kBytesPerSample];
std::size_t record_n;

void put24(uint8_t *p, int32_t v)
{
	p[0] = static_cast<uint8_t>(v >> 16);
	p[1] = static_cast<uint8_t>(v >> 8);
	p[2] = static_cast<uint8_t>(v);
}

// ---- IPC -------------------------------------------------------------------------

ipc_ept ep;
K_SEM_DEFINE(bound_sem, 0, 1);
std::atomic<bool> bound{false};

// written by the IPC callback, applied by the main loop
k_spinlock rx_lock;
msg::Config pending_cfg;
bool cfg_pending;
std::atomic<uint8_t> stream_mode{static_cast<uint8_t>(msg::StreamMode::Off)};
std::atomic<bool> rearm_pending{false};

void on_bound(void *) { bound = true; k_sem_give(&bound_sem); }

void on_received(const void *data, size_t len, void *)
{
	if (len < 1) {
		return;
	}
	const auto type = static_cast<msg::Type>(*static_cast<const uint8_t *>(data));
	if (type == msg::Type::Config && len == sizeof(msg::Config)) {
		const k_spinlock_key_t key = k_spin_lock(&rx_lock);
		std::memcpy(&pending_cfg, data, sizeof(pending_cfg));
		cfg_pending = true;
		k_spin_unlock(&rx_lock, key);
	} else if (type == msg::Type::Stream && len == sizeof(msg::Stream)) {
		stream_mode = static_cast<uint8_t>(static_cast<const msg::Stream *>(data)->mode);
	} else if (type == msg::Type::Rearm) {
		rearm_pending = true;
	}
}

template <typename T> void send(const T &m)
{
	if (bound) {
		ipc_service_send(&ep, &m, sizeof(m));
	}
}

void apply_config()
{
	msg::Config c;
	{
		const k_spinlock_key_t key = k_spin_lock(&rx_lock);
		if (!cfg_pending) {
			k_spin_unlock(&rx_lock, key);
			return;
		}
		c = pending_cfg;
		cfg_pending = false;
		k_spin_unlock(&rx_lock, key);
	}
	trig.reset({
		.hr_high = c.hr_high,
		.hr_low = c.hr_low,
		.rr_high = c.rr_high,
		.rr_low = c.rr_low,
		.apnea = c.trig_apnea != 0,
		.confirm_ms = c.confirm_ms,
	});
	resp.reset({.apnea_ms = c.apnea_ms});
}

int ipc_init()
{
	const device *ipc = DEVICE_DT_GET(DT_NODELABEL(ipc0));
	static const ipc_ept_cfg cfg = {
		.name = "afe",
		.cb = {.bound = on_bound, .received = on_received},
	};
	int err = ipc_service_open_instance(ipc);
	if (err && err != -EALREADY) {
		return err;
	}
	return ipc_service_register_endpoint(ipc, &ep, &cfg);
}

} // namespace

int main()
{
	qrs.reset();
	resp.reset({});
	trig.reset({}); // defaults until the application core sends its settings
	if (ipc_init() != 0) {
		return 0;
	}
	k_sem_take(&bound_sem, K_SECONDS(5));
	if (ads.init() != 0) {
		return 0; // the application core sees no Status and reports the ECG as invalid
	}

	constexpr std::size_t kBlock = drv::Ads1292r::kSps / 5; // 200 ms
	std::array<float, kBlock> ecg, bioz;
	std::size_t n = 0, blocks = 0;
	uint8_t loff = 0;
	int misses = 0;
	auto last_mode = msg::StreamMode::Off;

	for (;;) {
		if (ads.wait(K_MSEC(100)) != 0) {
			if (++misses >= 10) {
				ads.init(); // no DRDY for a second: restart the AFE
				misses = 0;
			}
			continue;
		}
		misses = 0;
		drv::Ads1292r::Sample s;
		if (ads.read(s) != 0) {
			continue;
		}

		const auto mode = static_cast<msg::StreamMode>(stream_mode.load());
		if (mode == msg::StreamMode::Record && last_mode != msg::StreamMode::Record) {
			record_n = 0; // a new minute starts
		}
		if (mode == msg::StreamMode::Record && record_n < kRecordSamples) {
			uint8_t *p = &record_buf[record_n * kBytesPerSample];
			put24(p, s.ecg);
			put24(p + 3, s.resp);
			if (++record_n == kRecordSamples) {
				send(msg::Recording{
					.type = msg::Type::Recording,
					.reserved = {},
					.addr = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(record_buf)),
					.samples = static_cast<uint32_t>(record_n),
					.sps = drv::Ads1292r::kSps,
				});
			}
		}
		last_mode = mode;

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
		if (++blocks < 5) {
			continue;
		}
		blocks = 0;

		// ---- once a second ----
		apply_config();
		if (rearm_pending.exchange(false)) {
			trig.rearm();
		}
		const algo::VitalTrigger::Input in{
			.hr_valid = qrs.hr_valid(),
			.hr_bpm = qrs.hr_bpm(),
			.rr_valid = resp.valid(),
			.rr_bpm = resp.rate_bpm(),
			.apnea = resp.apnea(),
			.leads_off = off,
		};
		const uint8_t fired = trig.update(in, k_uptime_get_32());
		if (fired) {
			send(msg::Trigger{
				.type = msg::Type::Trigger,
				.bits = fired,
				.reserved = {},
				.hr_bpm = in.hr_bpm,
				.rr_bpm = in.rr_bpm,
			});
		}
		if (mode != msg::StreamMode::Off) {
			send(msg::Status{
				.type = msg::Type::Status,
				.flags = static_cast<uint8_t>((in.hr_valid ? msg::kHrValid : 0) |
							      (in.rr_valid ? msg::kRrValid : 0) |
							      (in.apnea ? msg::kApnea : 0) |
							      (off ? msg::kLeadsOff : 0)),
				.reserved = {},
				.hr_bpm = in.hr_bpm,
				.rr_bpm = in.rr_bpm,
			});
		}
	}
	return 0;
}
