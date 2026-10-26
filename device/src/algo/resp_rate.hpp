// Breathing rate and apnea from the ADS1292R Bio-Z (impedance) channel.
#pragma once

#include <cstdint>
#include <span>

#include "algo/dsp.hpp"
#include "algo/generated/filters.hpp"

namespace dev::algo {

class RespRate {
public:
	static constexpr uint32_t kDecim = 25; // 125 Hz -> 5 Hz
	static constexpr float kFs = coeffs::kRespFs;

	struct Config {
		uint32_t apnea_ms = 20000; // no breath for this long -> apnea
	};

	void reset(const Config &cfg);

	// raw Bio-Z samples at 125 Hz (any linear unit); leads_off resets
	void process(std::span<const float> x, bool leads_off);

	bool valid() const;
	float rate_bpm() const { return rate_bpm_; }
	bool apnea() const;
	float amplitude() const { return breath_amp_; } // mean peak-to-peak of recent breaths

private:
	void sample(float x);
	void breath(uint32_t t);

	Config cfg_{};
	Biquad<coeffs::kRespBandpassStages> bp_{};
	float acc_{};
	uint32_t acc_n_{};
	bool primed_{};
	float prime_{};
	uint32_t t_{};          // 5 Hz samples since reset
	float env_{};           // mean |y|
	bool armed_{};          // went below -h since the last breath
	float hi_{}, lo_{};       // extremes of the current cycle
	bool have_breath_{};
	uint32_t last_breath_{};
	float breath_amp_{};
	History<float, 5> iv_{}; // breath intervals [s]
	float rate_bpm_{};
};

} // namespace dev::algo
