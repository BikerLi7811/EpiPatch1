// Combines the vital signs into organ-system findings and a deploy decision.
//
// Modelled on the clinical anaphylaxis criteria (acute exposure plus
// respiratory and/or cardiovascular involvement), using what this device
// can sense:
//   exposure        histamine above the wearer's baseline now or within the last
//                   hour (plasma histamine clears in 15-60 min, while reactions
//                   can take 30 min or more to reach the airway or circulation)
//   respiratory     repeated wheeze (mic), fast breathing, apnea / very slow breathing (Bio-Z)
//   cardiovascular  tachycardia (ECG), perfusion drop (PPG), collapse or fall (IMUs)
// Default trigger rules (each can be disabled):
//   A: exposure AND (respiratory OR cardiovascular)
//   B: respiratory AND (collapse OR fall)       -- severe, histamine not required
// Heart- and breathing-rate findings are ignored while the wearer is moving
// (exercise), and invalid sensors never contribute a finding.
//
// THE THRESHOLDS ARE PLACEHOLDERS. They have not been validated on patients.
#pragma once

#include <array>
#include <cstdint>

#include "algo/vitals.hpp"

namespace dev::algo {

enum Finding : uint16_t {
	kWheeze = 1u << 0,
	kTachypnea = 1u << 1,
	kApnea = 1u << 2,
	kTachycardia = 1u << 3,
	kPerfusionDrop = 1u << 4,
	kCollapse = 1u << 5,
	kFall = 1u << 6,
	kHistamine = 1u << 7,
};

struct Assessment {
	uint16_t findings;
	bool exposure;
	bool respiratory;
	bool cardiovascular;
	bool trigger; // deploy criteria met
	bool severe;  // respiratory + collapse/fall: re-alerts even after a cancel
};

class AnaphylaxisEngine {
public:
	struct Config {
		float active_g = 0.15f;           // above: moving, ignore HR/RR findings
		float rest_g = 0.05f;             // below: at rest, learn baselines
		uint8_t wheeze_min_s = 3;          // wheeze heard in at least this many seconds...
		uint32_t wheeze_window_ms = 60000; // ...within this window (one stray detection is not a finding)
		uint32_t wheeze_recent_ms = 60000; // then the finding holds this long
		float rr_high_bpm = 28.0f;
		uint32_t rr_high_hold_ms = 30000;
		float rr_low_bpm = 8.0f;
		uint32_t rr_low_hold_ms = 15000;
		float hr_high_bpm = 120.0f;
		float hr_rise_bpm = 40.0f;        // also above baseline + this
		uint32_t hr_hold_ms = 60000;
		float pi_drop_ratio = 0.5f;       // PI at or below this x baseline
		uint32_t pi_hold_ms = 30000;
		uint32_t baseline_min_ms = 120000; // rest time before a baseline counts
		float baseline_tau_s = 600.0f;
		uint32_t exposure_hold_ms = 3600000; // histamine counts as exposure this long after it was elevated
		bool rule_exposure = true;        // rule A
		bool rule_resp_collapse = true;   // rule B
	};

	void reset(const Config &cfg);
	Assessment update(const Vitals &v, uint32_t now_ms); // call once per second

	bool hr_baseline_valid() const { return hr_rest_ms_ >= cfg_.baseline_min_ms; }
	float hr_baseline() const { return hr_base_; }
	static constexpr uint8_t kMaxWheezeSeconds = 8;
	float pi_baseline() const { return pi_base_; }

private:
	bool wheeze_finding(const Vitals &v, uint32_t now);

	// true once cond has held continuously for hold_ms
	struct Hold {
		bool on = false;
		uint32_t since = 0;
		bool update(bool cond, uint32_t now, uint32_t hold_ms);
	};

	Config cfg_{};
	Hold tachypnea_{}, slow_rr_{}, tachycardia_{}, perfusion_{};
	float hr_base_{}, pi_base_{};
	uint32_t hr_rest_ms_{}, pi_rest_ms_{};
	uint32_t last_ms_{};
	bool started_{};
	std::array<uint32_t, kMaxWheezeSeconds> wz_t_{}; // seconds with a wheeze, oldest first
	uint8_t wz_n_{};
	uint32_t wz_events_{};  // wheeze_events at the last update
	bool wz_ok_{};
	uint32_t wz_ok_ms_{};   // last time the repeat rule was met
	bool hist_seen_{};
	uint32_t hist_ms_{};    // last time histamine was elevated
};

} // namespace dev::algo
