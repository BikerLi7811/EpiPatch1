// One-second snapshot of everything the decision engine looks at.
#pragma once

#include <cstdint>

namespace dev {

enum class Posture : uint8_t { Unknown, Standing, Sitting, Lying, Transition };

struct Vitals {
	// ECG (ADS1292R channel 2)
	bool hr_valid;
	float hr_bpm;
	bool ecg_leads_off;

	// Bio-Z respiration (ADS1292R channel 1)
	bool rr_valid;
	float rr_bpm;
	bool apnea;

	// PPG (MAX30101, IR channel)
	bool ppg_valid;
	float pi_pct;      // perfusion index AC/DC*100
	float pulse_bpm;
	float spo2_est;    // uncalibrated ratio-of-ratios estimate, reported only

	// Microphone (wheeze module)
	bool wheezing;
	uint32_t last_wheeze_ms; // 0 = never
	uint32_t wheeze_events;

	// IMUs
	Posture posture;
	float activity_g;  // max of chest/thigh dynamic acceleration
	bool collapsed;
	bool fall;

	// Histamine sensor
	bool hist_valid;
	bool hist_elevated;
	float hist_nm;
	float hist_baseline_nm;
};

} // namespace dev
