// When the application core reads its sensors.
//
//   Idle    sensors off (PPG shut down, mic asleep); the ECG/Bio-Z triggers and
//           the IMU interrupts (tap, impact, free-fall, FIFO watermark) stay live
//   Spot    every period_ms: everything on for spot_ms, decision every second
//   Burst   an ECG/Bio-Z trigger fired: everything on for burst_ms, every sample
//           kept, then the whole minute is analysed at once
//   Active  an alert or deployment is under way: everything on, decision every
//           second, until it is over
//
// A burst counts as a scheduled read, so the next spot is period_ms after it.
#pragma once

#include <cstdint>

#include "algo/dsp.hpp"

namespace dev::algo {

class SamplingSchedule {
public:
	enum class Mode : uint8_t { Idle, Spot, Burst, Active };
	enum class Action : uint8_t { None, StartSpot, EndSpot, StartBurst, EndBurst, StartActive, EndActive };

	struct Config {
		uint32_t period_ms = 120000;
		uint32_t spot_ms = 15000;
		uint32_t burst_ms = 60000;
	};

	// The first spot read happens at the first update().
	void reset(const Config &cfg, uint32_t now_ms)
	{
		cfg_ = cfg;
		mode_ = Mode::Idle;
		start_ = now_ms;
		next_spot_ = now_ms;
	}

	// An ECG/Bio-Z trigger fired.
	Action trigger(uint32_t now_ms)
	{
		if (mode_ != Mode::Idle && mode_ != Mode::Spot) {
			return Action::None; // already recording everything
		}
		mode_ = Mode::Burst;
		start_ = now_ms;
		next_spot_ = now_ms + cfg_.burst_ms + cfg_.period_ms;
		return Action::StartBurst;
	}

	// Call on every wake-up. alerting: an alert countdown or deployment is
	// running. Returns at most one action; call again until None.
	Action update(uint32_t now_ms, bool alerting)
	{
		if (alerting && mode_ != Mode::Active && mode_ != Mode::Burst) {
			mode_ = Mode::Active;
			return Action::StartActive;
		}
		switch (mode_) {
		case Mode::Active:
			if (!alerting) {
				mode_ = Mode::Idle;
				next_spot_ = now_ms + cfg_.period_ms;
				return Action::EndActive;
			}
			break;
		case Mode::Spot:
			if (after_eq(now_ms, start_ + cfg_.spot_ms)) {
				mode_ = Mode::Idle;
				return Action::EndSpot;
			}
			break;
		case Mode::Burst:
			if (after_eq(now_ms, start_ + cfg_.burst_ms)) {
				mode_ = Mode::Idle;
				return Action::EndBurst;
			}
			break;
		case Mode::Idle:
			if (after_eq(now_ms, next_spot_)) {
				mode_ = Mode::Spot;
				start_ = now_ms;
				next_spot_ = now_ms + cfg_.period_ms;
				return Action::StartSpot;
			}
			break;
		}
		return Action::None;
	}

	Mode mode() const { return mode_; }
	// sensors on and a 1 s tick needed
	bool awake() const { return mode_ != Mode::Idle; }
	// ms until update() has something to do while idle (0 when not idle)
	uint32_t idle_wait_ms(uint32_t now_ms) const
	{
		if (mode_ != Mode::Idle || after_eq(now_ms, next_spot_)) {
			return 0;
		}
		return next_spot_ - now_ms;
	}

private:
	Config cfg_{};
	Mode mode_ = Mode::Idle;
	uint32_t start_{};
	uint32_t next_spot_{};
};

} // namespace dev::algo
