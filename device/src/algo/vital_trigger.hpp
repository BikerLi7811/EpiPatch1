// The two ECG/Bio-Z triggers that start a one-minute burst recording:
//   heart-rate trigger   HR at or above hr_high, or at or below hr_low
//   breathing trigger    RR at or above rr_high, at or below rr_low, or apnea
// A condition has to hold for confirm_ms before it fires. Each trigger fires
// once and then stays quiet until rearm() (called when the burst it started
// has been analysed); a condition that still holds then fires again at once,
// so a persistent abnormality keeps the device recording.
//
// Runs wherever the ADS1292R is read: the FLPR coprocessor, or the afe thread
// when the ECG is read on the application core.
#pragma once

#include <cstdint>

#include "algo/dsp.hpp"

namespace dev::algo {

class VitalTrigger {
public:
	enum Bits : uint8_t {
		kHrHigh = 1u << 0,
		kHrLow = 1u << 1,
		kRrHigh = 1u << 2,
		kRrLow = 1u << 3,
		kApnea = 1u << 4,
		kHeart = kHrHigh | kHrLow,
		kBreathing = kRrHigh | kRrLow | kApnea,
	};

	struct Config {
		float hr_high = 110.0f;
		float hr_low = 40.0f;
		float rr_high = 25.0f;
		float rr_low = 8.0f;
		bool apnea = true;
		uint32_t confirm_ms = 5000;
	};

	struct Input {
		bool hr_valid;
		float hr_bpm;
		bool rr_valid;
		float rr_bpm;
		bool apnea;
		bool leads_off;
	};

	void reset(const Config &cfg)
	{
		cfg_ = cfg;
		heart_ = breathing_ = {};
	}

	// Call once per second (or per processed block). Returns the bits of the
	// condition(s) behind every trigger that fired on this call, 0 if none.
	uint8_t update(const Input &in, uint32_t now_ms)
	{
		uint8_t hr = 0, rr = 0;
		if (!in.leads_off && in.hr_valid) {
			hr |= in.hr_bpm >= cfg_.hr_high ? kHrHigh : 0;
			hr |= in.hr_bpm <= cfg_.hr_low ? kHrLow : 0;
		}
		if (!in.leads_off) {
			if (in.rr_valid) {
				rr |= in.rr_bpm >= cfg_.rr_high ? kRrHigh : 0;
				rr |= in.rr_bpm <= cfg_.rr_low ? kRrLow : 0;
			}
			rr |= cfg_.apnea && in.apnea ? kApnea : 0;
		}
		return heart_.update(hr, now_ms, cfg_.confirm_ms) | breathing_.update(rr, now_ms, cfg_.confirm_ms);
	}

	void rearm()
	{
		heart_.fired = false;
		breathing_.fired = false;
	}

	// trigger currently latched (fired and not yet re-armed)
	bool heart_latched() const { return heart_.fired; }
	bool breathing_latched() const { return breathing_.fired; }

private:
	struct Channel {
		uint8_t active{};   // condition bits now
		uint32_t since{};   // when the condition started
		bool fired{};

		uint8_t update(uint8_t bits, uint32_t now, uint32_t confirm_ms)
		{
			if (!bits) {
				active = 0;
				return 0;
			}
			if (!active) {
				since = now;
			}
			active = bits;
			if (fired || !after_eq(now, since + confirm_ms)) {
				return 0;
			}
			fired = true;
			return bits;
		}
	};

	Config cfg_{};
	Channel heart_{}, breathing_{};
};

} // namespace dev::algo
