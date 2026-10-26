/*
 * Wheeze detector service: PDM MEMS mic -> CMSIS-DSP log-mel -> CMSIS-NN CNN,
 * running in its own thread behind the C API in <wheeze/wheeze.h>.
 *
 * Battery strategy (largest consumer first):
 *  1. Mic duty-cycling (DutyCycler): mic + PDM20 + clock are off most of the
 *     time while nothing is happening, continuous once a wheeze candidate shows.
 *  2. Low PDM rate: 8 kHz PCM lets the mic run at a ~0.5-1 MHz PDM clock
 *     (low-power mode on most MEMS mics); falls back to 16 kHz.
 *  3. No logging unless CONFIG_WHEEZE_LOG_LEVEL asks for it; LED only pulses.
 *  4. CPU work cut by training/optimize.py: CNN every kInferStride frames, and
 *     only when the O(1) tonality gate says the window can contain a wheeze.
 *  5. 256 ms EasyDMA blocks: 4 wake-ups/s; the CPU races to idle (WFI) in
 *     dmic_read()/k_sem_take() at 128 MHz with FPU + DSP-extension SIMD.
 */
#include <zephyr/kernel.h>
#include <zephyr/audio/dmic.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <cmsis_core.h>

#include <array>
#include <cerrno>
#include <span>

#include <wheeze/wheeze.h>

#include "duty_cycler.hpp"
#include "feature_extractor.hpp"
#include "wheeze_net.hpp"

LOG_MODULE_REGISTER(wheeze, CONFIG_WHEEZE_LOG_LEVEL);

namespace {

constexpr bool kProfile = IS_ENABLED(CONFIG_WHEEZE_PROFILING);

#if DT_NODE_EXISTS(DT_ALIAS(wheeze_mic))
#define WHEEZE_MIC_NODE DT_ALIAS(wheeze_mic)
#else
#define WHEEZE_MIC_NODE DT_ALIAS(dmic0)
#endif

constexpr uint32_t kBlockMs = 256; /* one wake-up per 256 ms of audio */
constexpr uint32_t kMaxFsMic = 16000;
constexpr std::size_t kMaxBlockBytes = kMaxFsMic * kBlockMs / 1000 * sizeof(int16_t);
constexpr std::size_t kBlockCount = 3;
constexpr uint32_t kFrameMs = 1000 * wheeze::dsp::kHop / wheeze::dsp::kFs;
constexpr uint32_t kInferMs = wheeze::model::kInferStride * kFrameMs;
constexpr uint32_t kSettleFrames = 8; /* skip ~128 ms after mic power-up */
constexpr uint32_t kStatsMs = 10000;
static_assert((kMaxFsMic * kBlockMs / 1000) % wheeze::FeatureExtractor::kDecimBlock == 0);

K_MEM_SLAB_DEFINE_STATIC(mic_slab, kMaxBlockBytes, kBlockCount, 4);
K_THREAD_STACK_DEFINE(wheeze_stack, CONFIG_WHEEZE_THREAD_STACK_SIZE);

class Microphone {
public:
	explicit constexpr Microphone(const device *dev) : dev_(dev) {}

	bool ready() const { return device_is_ready(dev_); }

	/* Tries every supported rate, lowest power first. Returns the one in use. */
	const wheeze::dsp::MicRate *start()
	{
		for (const auto &rate : wheeze::dsp::mic_rates) {
			const bool lp = rate.fs < kMaxFsMic;

			if (configure(rate, lp ? CONFIG_WHEEZE_MIC_LP_CLK_MIN : CONFIG_WHEEZE_MIC_CLK_MIN,
				      lp ? CONFIG_WHEEZE_MIC_LP_CLK_MAX : CONFIG_WHEEZE_MIC_CLK_MAX) &&
			    dmic_trigger(dev_, DMIC_TRIGGER_START) == 0) {
				return &rate;
			}
		}
		return nullptr;
	}

	/* Powers the mic down (PDM clock stops) and returns queued blocks to the slab. */
	void stop()
	{
		void *buf;
		size_t size;

		dmic_trigger(dev_, DMIC_TRIGGER_STOP);
		while (dmic_read(dev_, 0, &buf, &size, 0) == 0) {
			k_mem_slab_free(&mic_slab, buf);
		}
	}

	/* Blocks (CPU in WFI) until EasyDMA has filled the next block. */
	template <typename Fn> bool read(Fn &&consume)
	{
		void *buf;
		size_t size;

		if (dmic_read(dev_, 0, &buf, &size, 2 * kBlockMs) < 0) {
			return false;
		}
		consume(std::span<const int16_t>(static_cast<const int16_t *>(buf),
						 size / sizeof(int16_t)));
		k_mem_slab_free(&mic_slab, buf);
		return true;
	}

private:
	bool configure(const wheeze::dsp::MicRate &rate, uint32_t clk_min, uint32_t clk_max)
	{
		pcm_stream_cfg stream = {};
		dmic_cfg cfg = {};

		stream.pcm_rate = rate.fs;
		stream.pcm_width = 16;
		stream.block_size = rate.fs * kBlockMs / 1000 * sizeof(int16_t);
		stream.mem_slab = &mic_slab;
		cfg.io.min_pdm_clk_freq = clk_min;
		cfg.io.max_pdm_clk_freq = clk_max;
		cfg.io.min_pdm_clk_dc = 40;
		cfg.io.max_pdm_clk_dc = 60;
		cfg.streams = &stream;
		cfg.channel.req_num_streams = 1;
		cfg.channel.req_num_chan = 1;
		cfg.channel.req_chan_map_lo = dmic_build_channel_map(
			0, 0, IS_ENABLED(CONFIG_WHEEZE_MIC_RIGHT_CHANNEL) ? PDM_CHAN_RIGHT : PDM_CHAN_LEFT);
		return dmic_configure(dev_, &cfg) == 0;
	}

	const device *dev_;
};

/* LED that only flashes briefly: a lit LED draws milliamps, far more than the detector. */
class PulseLed {
public:
	explicit constexpr PulseLed(const gpio_dt_spec &spec) : spec_(spec), off_{} {}

	void init()
	{
		k_work_init_delayable(&off_, [](k_work *w) {
			auto *self = CONTAINER_OF(k_work_delayable_from_work(w), PulseLed, off_);
			gpio_pin_set_dt(&self->spec_, 0);
		});
		if (spec_.port) {
			gpio_pin_configure_dt(&spec_, GPIO_OUTPUT_INACTIVE);
		}
	}
	void pulse()
	{
		if (spec_.port && CONFIG_WHEEZE_LED_PULSE_MS > 0) {
			gpio_pin_set_dt(&spec_, 1);
			k_work_reschedule(&off_, K_MSEC(CONFIG_WHEEZE_LED_PULSE_MS));
		}
	}

private:
	gpio_dt_spec spec_;
	k_work_delayable off_;
};

/* DWT cycle counter, CONFIG_WHEEZE_PROFILING only: TRCENA keeps the debug
 * domain powered, which costs current. */
struct CycleCounter {
	static void enable()
	{
		if constexpr (kProfile) {
			CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
			DWT->CYCCNT = 0;
			DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
		}
	}
	static uint32_t now() { return kProfile ? DWT->CYCCNT : 0; }
	static uint32_t to_us(uint64_t cycles) { return cycles / (SystemCoreClock / 1000000); }
};

struct Profile {
	uint32_t windows = 0, cnn_runs = 0, listen_ms = 0, since_ms = 0;
	uint64_t cyc_dsp = 0, cyc_nn = 0;
};

constexpr wheeze::DutyCycler::Config duty_config()
{
	wheeze::DutyCycler::Config c;
	c.enabled = IS_ENABLED(CONFIG_WHEEZE_DUTY_CYCLE);
	c.listen_ms = CONFIG_WHEEZE_DUTY_LISTEN_MS;
	c.sleep_ms = CONFIG_WHEEZE_DUTY_SLEEP_MS;
	c.hold_ms = CONFIG_WHEEZE_DUTY_HOLD_MS;
	return c;
}

/* ---- service state (all in .bss) ---- */
/* constinit: initialised at compile time, no static constructors run at boot. */
constinit Microphone mic(DEVICE_DT_GET(WHEEZE_MIC_NODE));
#if DT_NODE_EXISTS(DT_ALIAS(wheeze_led))
constinit PulseLed led(GPIO_DT_SPEC_GET(DT_ALIAS(wheeze_led), gpios));
#else
constinit PulseLed led(gpio_dt_spec{});
#endif
wheeze::FeatureExtractor features;
wheeze::WheezeNet net;
std::array<float, wheeze::kWindowSize> win_f;
alignas(4) std::array<int8_t, wheeze::kWindowSize> win_q;

k_thread thread;
k_tid_t tid;
atomic_t stop_req;
k_sem wake_sem;
k_spinlock stats_lock;
wheeze_stats stats;
wheeze_callback_t user_cb;
void *user_data;

void publish(const wheeze::EventDetector &det, const wheeze::DutyCycler &duty, bool mic_on,
	     uint32_t rate_hz)
{
	const k_spinlock_key_t key = k_spin_lock(&stats_lock);

	stats.events = det.events();
	stats.wheeze_ms_total = det.wheeze_ms_total();
	stats.wheezing = det.wheezing();
	stats.mic_on = mic_on;
	stats.continuous = duty.continuous(k_uptime_get_32());
	stats.mic_rate_hz = mic_on ? rate_hz : 0;
	k_spin_unlock(&stats_lock, key);
}

void notify(wheeze_event_type type, const wheeze::EventDetector &det, uint32_t now)
{
	if (user_cb) {
		const wheeze_event evt = {
			.type = type,
			.timestamp_ms = now,
			.count = det.events(),
			.duration_ms = type == WHEEZE_EVT_END ? det.last_event_ms() : 0,
			.probability = det.smoothed(),
		};
		user_cb(&evt, user_data);
	}
}

/* One detection step on the current window. Returns P(wheeze) (0 if gated). */
float infer(Profile &pr)
{
	pr.windows++;
	if (wheeze::model::kGateEnabled && features.tonal_frames() < wheeze::model::kGateMinFrames) {
		return 0.0f; /* no tonal content: cannot be a wheeze, skip the CNN */
	}
	const uint32_t t0 = CycleCounter::now();
	features.window(win_f);
	wheeze::quantize(win_f, win_q, wheeze::model::kInScale, wheeze::model::kInZp);
	const auto p = net.run(win_q);
	pr.cyc_nn += CycleCounter::now() - t0;
	pr.cnn_runs++;
	if (!p) {
		LOG_ERR("CNN kernel error");
		return 0.0f;
	}
	return *p;
}

/* wheeze_listen(): keep the mic on until this uptime (0 = no request) */
atomic_t listen_until;

void apply_listen_request(wheeze::DutyCycler &duty)
{
	const auto until = static_cast<uint32_t>(atomic_get(&listen_until));
	if (until) {
		duty.force_until(until);
	}
}

void run(void *, void *, void *)
{
	wheeze::EventDetector det(wheeze::EventDetector::Config::for_period_ms(kInferMs));
	wheeze::DutyCycler duty(duty_config());
	Profile pr;

	CycleCounter::enable();
	while (!atomic_get(&stop_req)) {
		/* ---- (re)start listening ---- */
		const auto *rate = mic.start();

		if (!rate) {
			LOG_ERR("PDM start failed, retrying");
			k_sem_take(&wake_sem, K_SECONDS(1));
			continue;
		}
		features.init(*rate, wheeze::model::kGateTonality);
		det.reset_stream();
		const uint32_t start_ms = k_uptime_get_32();
		uint32_t last_infer = 0;

		duty.on_listen_start(start_ms);
		apply_listen_request(duty);
		publish(det, duty, true, rate->fs);
		LOG_INF("listening: PDM %u Hz, CNN every %u ms, gate %s", rate->fs, kInferMs,
			wheeze::model::kGateEnabled ? "on" : "off");

		/* ---- listen until the duty-cycler (or wheeze_stop) says stop ---- */
		while (!atomic_get(&stop_req)) {
			const bool ok = mic.read([&](std::span<const int16_t> pcm) {
				for (; pcm.size() >= wheeze::FeatureExtractor::kDecimBlock;
				     pcm = pcm.subspan(wheeze::FeatureExtractor::kDecimBlock)) {
					const uint32_t t0 = CycleCounter::now();

					features.push_pcm(pcm.first(wheeze::FeatureExtractor::kDecimBlock));
					pr.cyc_dsp += CycleCounter::now() - t0;

					const uint32_t n = features.frames_total();
					if (n < wheeze::dsp::kNumFrames + kSettleFrames ||
					    n - last_infer < wheeze::model::kInferStride) {
						continue;
					}
					last_infer = n;

					const uint32_t now = k_uptime_get_32();
					const auto change = det.update(infer(pr), now);

					if (det.wheezing() || det.smoothed() >= det.config().thr_off) {
						duty.on_candidate(now);
					}
					if (change == wheeze::EventDetector::Change::Started) {
						notify(WHEEZE_EVT_START, det, now);
					} else if (change == wheeze::EventDetector::Change::Ended) {
						led.pulse();
						LOG_INF("wheeze event #%u  %u ms", det.events(), det.last_event_ms());
						notify(WHEEZE_EVT_END, det, now);
					}
				}
			});
			const uint32_t now = k_uptime_get_32();

			if (!ok) {
				LOG_ERR("PDM read timeout");
				break; /* restart the mic */
			}
			publish(det, duty, true, rate->fs);
			pr.listen_ms += kBlockMs;
			if constexpr (kProfile) {
				if (now - pr.since_ms >= kStatsMs) {
					const uint32_t span = now - pr.since_ms;

					LOG_INF("events %u | CNN ran on %u/%u windows | dsp %u us/s, cnn %u us/s | "
						"mic on %u%%",
						det.events(), pr.cnn_runs, pr.windows,
						CycleCounter::to_us(pr.cyc_dsp * 1000 / span),
						CycleCounter::to_us(pr.cyc_nn * 1000 / span),
						100 * pr.listen_ms / span);
					pr = Profile{};
					pr.since_ms = now;
				}
			}
			apply_listen_request(duty);
			if (!det.wheezing() && duty.should_stop(now)) {
				break;
			}
		}

		/* ---- mic off, CPU in System ON idle (wheeze_stop() wakes us early) ---- */
		mic.stop();
		publish(det, duty, false, 0);
		apply_listen_request(duty);
		if (!atomic_get(&stop_req) && !duty.continuous(k_uptime_get_32())) {
			k_sem_take(&wake_sem, K_MSEC(duty.sleep_ms()));
		}
	}
}

} // namespace

extern "C" int wheeze_start(wheeze_callback_t cb, void *data)
{
	if (tid) {
		return -EALREADY;
	}
	if (!mic.ready()) {
		return -ENODEV;
	}
	led.init();
	user_cb = cb;
	user_data = data;
	atomic_clear(&stop_req);
	k_sem_init(&wake_sem, 0, 1);
	tid = k_thread_create(&thread, wheeze_stack, K_THREAD_STACK_SIZEOF(wheeze_stack), run,
			      nullptr, nullptr, nullptr, CONFIG_WHEEZE_THREAD_PRIORITY, K_FP_REGS,
			      K_NO_WAIT);
	k_thread_name_set(tid, "wheeze");
	return 0;
}

extern "C" int wheeze_stop(void)
{
	if (!tid) {
		return -EALREADY;
	}
	atomic_set(&stop_req, 1);
	k_sem_give(&wake_sem);
	k_thread_join(&thread, K_FOREVER);
	tid = nullptr;
	return 0;
}

extern "C" int wheeze_listen(uint32_t ms)
{
	if (!tid) {
		return -ENODEV;
	}
	/* 0 means "no request", so never store it as a deadline */
	const uint32_t until = (k_uptime_get_32() + ms) | 1u;
	atomic_set(&listen_until, static_cast<atomic_val_t>(until));
	k_sem_give(&wake_sem);
	return 0;
}

extern "C" int wheeze_get_stats(wheeze_stats *out)
{
	if (!out) {
		return -EINVAL;
	}
	const k_spinlock_key_t key = k_spin_lock(&stats_lock);

	*out = stats;
	k_spin_unlock(&stats_lock, key);
	return 0;
}
