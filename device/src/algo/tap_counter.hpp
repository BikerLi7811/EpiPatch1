// Turns the IMU's single-tap interrupts into an N-tap gesture (default 3).
// The LSM6DSO only recognises single and double taps in hardware.
#pragma once

#include <array>
#include <cstdint>

namespace dev::algo {

class TapCounter {
public:
	static constexpr uint8_t kMaxTaps = 8;

	struct Config {
		uint8_t taps = 3;
		uint32_t window_ms = 1500; // all N taps within this
		uint32_t min_gap_ms = 80;  // closer taps are the same tap ringing
	};

	void reset(const Config &cfg)
	{
		cfg_ = cfg;
		if (cfg_.taps < 1 || cfg_.taps > kMaxTaps) {
			cfg_.taps = 3;
		}
		n_ = 0;
	}

	// returns true exactly once when the gesture completes
	bool on_tap(uint32_t t_ms)
	{
		if (n_ > 0 && t_ms - t_[n_ - 1] < cfg_.min_gap_ms) {
			return false;
		}
		// drop taps that fell out of the window
		uint8_t keep = 0;
		for (uint8_t i = 0; i < n_; ++i) {
			if (t_ms - t_[i] <= cfg_.window_ms) {
				t_[keep++] = t_[i];
			}
		}
		n_ = keep;
		t_[n_++] = t_ms;
		if (n_ >= cfg_.taps) {
			n_ = 0;
			return true;
		}
		return false;
	}

	uint8_t pending() const { return n_; }

private:
	Config cfg_{};
	std::array<uint32_t, kMaxTaps> t_{};
	uint8_t n_ = 0;
};

} // namespace dev::algo
