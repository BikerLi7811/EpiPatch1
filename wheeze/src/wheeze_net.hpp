/*
 * int8 wheeze CNN on CMSIS-NN (uses the Cortex-M33 DSP extension: SMLAD/SXTAB16
 * SIMD MACs when built with __ARM_FEATURE_DSP).
 *
 *   in  40x32x1  -> conv3x3 W1 + ReLU -> maxpool 2
 *       20x16xW1 -> conv3x3 W2 + ReLU -> maxpool 2
 *       10x8xW2  -> conv3x3 W3 + ReLU -> global avg pool -> FC 1 -> logit
 */
#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <span>

#include "feature_extractor.hpp"
#include "generated/model_data.hpp"

namespace wheeze {

class WheezeNet {
public:
	/* P(wheeze) in [0,1], or nullopt if a CMSIS-NN kernel rejected its arguments. */
	std::optional<float> run(std::span<const int8_t, kWindowSize> in);

private:
	/* Feature-map rows (time) x cols (mel) per stage; channel counts are model::kW1..3. */
	static constexpr int kRows0 = dsp::kNumFrames, kCols0 = dsp::kNumMels;
	static constexpr int kRows1 = kRows0 / 2, kCols1 = kCols0 / 2;
	static constexpr int kRows2 = kRows1 / 2, kCols2 = kCols1 / 2;
	/* Ping-pong activation arena; largest tensor is conv1 output. */
	static constexpr std::size_t kArena = kRows0 * kCols0 * model::kW1;
	static constexpr std::size_t kScratch = 4096;

	struct ConvLayer {
		const int8_t *weights;
		const int32_t *bias, *mult, *shift;
		int cout;
		int32_t out_zp;
	};

	bool conv(const int8_t *in, int h, int w, int cin, int32_t in_zp, const ConvLayer &l,
		  int8_t *out);
	bool maxpool2(const int8_t *in, int h, int w, int c, int8_t *out);

	alignas(4) std::array<int8_t, kArena> a_;
	alignas(4) std::array<int8_t, kArena> b_;
	alignas(4) std::array<int8_t, kScratch> scratch_;
};

/*
 * Turns the per-window probability stream into wheeze on/off + event counts.
 * Same idea as the paper's counter (0.9/0.1 hysteresis), plus smoothing and a
 * minimum duration so isolated windows don't count as events.
 */
class EventDetector {
public:
	static constexpr std::size_t kMaxSmooth = 4;

	struct Config {
		float thr_on = model::kThreshold;
		float thr_off = 0.6f * model::kThreshold;
		uint32_t smooth = 3;         /* moving average length (<= kMaxSmooth) */
		uint32_t confirm = 2;        /* consecutive smoothed hits to switch on */
		uint32_t min_event_ms = 250; /* wheezes are continuous sounds >~100-250 ms */

		/* Same time constants (~190 ms smoothing, ~128 ms confirmation) at any
		 * inference period. */
		static constexpr Config for_period_ms(uint32_t period_ms)
		{
			Config c;
			c.smooth = period_ms >= 192 ? 1 : (192 + period_ms / 2) / period_ms;
			c.confirm = (128 + period_ms - 1) / period_ms;
			return c;
		}
	};
	enum class Change { None, Started, Ended };

	EventDetector() = default;
	explicit EventDetector(const Config &cfg) : cfg_(cfg) {}

	/* Feed one probability taken at time now_ms. */
	Change update(float p, uint32_t now_ms);

	/* New audio stream (mic was powered down): forget smoothing state, keep counts. */
	void reset_stream()
	{
		hist_.fill(0.0f);
		idx_ = 0;
		smoothed_ = 0.0f;
		hits_ = 0;
		on_ = false;
	}

	const Config &config() const { return cfg_; }

	bool wheezing() const { return on_; }
	float smoothed() const { return smoothed_; }
	uint32_t events() const { return events_; }
	uint32_t wheeze_ms_total() const { return wheeze_ms_; }
	uint32_t last_event_ms() const { return last_dur_; }

private:
	Config cfg_{};
	std::array<float, kMaxSmooth> hist_{};
	std::size_t idx_ = 0;
	float smoothed_ = 0.0f;
	uint32_t hits_ = 0;
	uint32_t on_since_ = 0;
	bool on_ = false;
	uint32_t events_ = 0, wheeze_ms_ = 0, last_dur_ = 0;
};

} // namespace wheeze
