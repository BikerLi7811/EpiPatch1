// Perfusion index, pulse rate and an (uncalibrated) SpO2 estimate from
// MAX30101 red/IR samples at 25 Hz.
#pragma once

#include <cstdint>
#include <span>

#include "algo/dsp.hpp"
#include "algo/generated/filters.hpp"

namespace dev::algo {

class PpgProcessor {
public:
	static constexpr float kFs = coeffs::kPpgFs;
	static constexpr float kFullScale = 262143.0f; // 18-bit ADC

	struct Config {
		float min_dc = 20000.0f; // below: no skin contact
	};

	void reset(const Config &cfg);
	void process(std::span<const uint32_t> red, std::span<const uint32_t> ir);

	bool contact() const { return contact_; }
	bool valid() const;
	float pi_pct() const { return pi_pct_; }
	float pulse_bpm() const { return pulse_bpm_; }
	float spo2_est() const { return spo2_; }

private:
	void sample(float red, float ir);
	void beat(float ac_ir, float ac_red);

	Config cfg_{};
	Biquad<coeffs::kPpgBandpassStages> bp_ir_{}; // beat detection (IR)
	bool primed_{};
	float prime_ir_{}, prime_red_{};
	float dc_ir_{}, dc_red_{};
	bool contact_{};
	uint32_t t_{};
	float s1_{}, s2_{};      // previous two (inverted) IR AC samples
	float pk_env_{};         // running height of detector peaks
	float hi_ir_{}, lo_ir_{}, hi_red_{}, lo_red_{}; // extremes of the current beat
	bool have_beat_{};
	uint32_t last_beat_{};
	History<float, 6> iv_{};
	History<float, 4> ac_ir_{}, ac_red_{};
	float pi_pct_{}, pulse_bpm_{}, spo2_{};
};

} // namespace dev::algo
