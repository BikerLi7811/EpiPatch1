// Streaming Pan-Tompkins QRS detector -> heart rate (ECG at 125 Hz).
#pragma once

#include <array>
#include <cstdint>
#include <span>

#include "algo/dsp.hpp"
#include "algo/generated/filters.hpp"

namespace dev::algo {

class QrsDetector {
public:
	static constexpr float kFs = coeffs::kEcgFs;

	void reset();

	// ecg_uv: consecutive ECG samples in microvolts.
	// leads_off: the AFE reported an electrode off during this block.
	void process(std::span<const float> ecg_uv, bool leads_off);

	bool hr_valid() const;
	float hr_bpm() const { return hr_bpm_; }
	uint32_t beats() const { return beats_; }
	// sample index (since reset) of every beat, for tests
	uint32_t last_beat_sample() const { return last_qrs_; }

private:
	static constexpr uint32_t kMwiLen = 19;     // 150 ms integration window
	static constexpr uint32_t kSettle = 63;     // 0.5 s band-pass settling
	static constexpr uint32_t kLearn = 313;     // then 2 s threshold learning
	static constexpr uint32_t kRefractory = 25; // 200 ms
	static constexpr uint32_t kTWave = 45;      // 360 ms
	static constexpr uint32_t kMinRr = 34;      // 0.27 s = 222 /min
	static constexpr uint32_t kMaxRr = 250;     // 2.0 s = 30 /min

	void sample(float bp);
	void on_peak(uint32_t idx, float v);
	void accept(uint32_t idx, float v, bool searchback);

	Biquad<coeffs::kEcgBandpassStages> bp_{};
	bool primed_{};
	bool relearn_{};
	float prime_{};            // first sample: electrode DC offset
	std::array<float, 4> dx_{};          // last 4 band-passed samples
	std::array<float, kMwiLen> mwi_buf_{};
	float mwi_sum_{};
	uint32_t mwi_pos_{};
	float m1_{}, m2_{};                    // previous two MWI values
	uint32_t n_{};                       // samples since reset

	float learn_max_{}, learn_sum_{};
	float spki_{}, npki_{};
	uint32_t last_qrs_{};
	float last_qrs_val_{};
	bool have_qrs_{};
	uint32_t noise_idx_{};
	float noise_max_{};

	History<float, 8> rr_{};             // seconds
	uint32_t beats_{};
	float hr_bpm_{};
};

} // namespace dev::algo
