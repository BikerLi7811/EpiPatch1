// Histamine sensor: AFE output voltage -> concentration, personal baseline,
// and an "elevated" flag with confirmation and hysteresis.
#pragma once

#include <cstdint>

namespace dev::algo {

class HistamineMonitor {
public:
	struct Config {
		float offset_mv = 500.0f;    // AFE output at zero histamine (>= min_mv)
		float nm_per_mv = 1.0f;      // calibration slope
		float min_mv = 50.0f;        // outside [min, max]: open/shorted sensor
		float max_mv = 3200.0f;
		float rise_nm = 10.0f;       // elevated: at least this far above baseline...
		float ratio = 1.5f;          // ...and this many times the baseline
		float abs_nm = 0.0f;         // >0: also elevated at or above this, baseline or not
		uint32_t period_ms = 10000;  // nominal sampling period (sample(mv) without a time)
		uint32_t warmup_ms = 300000; // sensor settling + first baseline
		float baseline_tau_s = 1800.0f;
		uint8_t confirm = 2;         // consecutive samples to change state
	};

	void reset(const Config &cfg)
	{
		cfg_ = cfg;
		valid_ = elevated_ = false;
		nm_ = baseline_ = 0.0f;
		have_base_ = false;
		run_ms_ = 0;
		streak_ = 0;
		have_t_ = false;
	}

	// One reading every period_ms.
	void sample(float mv) { sample(mv, have_t_ ? last_t_ + cfg_.period_ms : 0); }

	// One reading taken at now_ms; the gap since the last one sets how far the
	// warm-up advances and how fast the baseline adapts (readings may be
	// minutes apart while the device is idle, 10 s apart during a burst).
	void sample(float mv, uint32_t now_ms)
	{
		const uint32_t dt = have_t_ ? now_ms - last_t_ : cfg_.period_ms;
		have_t_ = true;
		last_t_ = now_ms;
		valid_ = mv >= cfg_.min_mv && mv <= cfg_.max_mv;
		if (!valid_) {
			// a broken sensor never asserts (or keeps asserting) histamine
			elevated_ = false;
			streak_ = 0;
			return;
		}
		nm_ = (mv - cfg_.offset_mv) * cfg_.nm_per_mv;
		if (nm_ < 0.0f) {
			nm_ = 0.0f;
		}
		if (run_ms_ < cfg_.warmup_ms) {
			run_ms_ += dt;
		}
		if (!have_base_) {
			baseline_ = nm_;
			have_base_ = true;
		}
		const bool warm = run_ms_ >= cfg_.warmup_ms;

		const bool above = warm && ((nm_ - baseline_ >= cfg_.rise_nm && nm_ >= cfg_.ratio * baseline_) ||
					    (cfg_.abs_nm > 0.0f && nm_ >= cfg_.abs_nm));
		const bool below = nm_ - baseline_ < 0.5f * cfg_.rise_nm &&
				   !(cfg_.abs_nm > 0.0f && nm_ >= cfg_.abs_nm);
		if (elevated_ ? below : above) {
			if (++streak_ >= cfg_.confirm) {
				elevated_ = !elevated_;
				streak_ = 0;
			}
		} else {
			streak_ = 0;
		}
		// learn the personal baseline only while not elevated
		if (!elevated_ && !above) {
			float alpha = warm ? (dt / 1000.0f) / cfg_.baseline_tau_s
					   : 0.2f; // converge quickly during warm-up
			if (alpha > 1.0f) {
				alpha = 1.0f;
			}
			baseline_ += alpha * (nm_ - baseline_);
		}
	}

	bool valid() const { return valid_; }
	bool elevated() const { return elevated_; }
	float nm() const { return nm_; }
	float baseline_nm() const { return baseline_; }

private:
	Config cfg_;
	bool valid_ = false, elevated_ = false, have_base_ = false;
	float nm_ = 0.0f, baseline_ = 0.0f;
	uint32_t run_ms_ = 0;
	uint8_t streak_ = 0;
	bool have_t_ = false;
	uint32_t last_t_ = 0;
};

} // namespace dev::algo
