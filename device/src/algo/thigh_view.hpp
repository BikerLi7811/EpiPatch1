// The chest patch's view of the thigh patch, from its Reports over Bluetooth:
// histamine (the thigh owns the AD5941), the thigh's fall detection and
// activity, and where its countdown/release state machine is.
//
// merge() folds that into the chest's own vital signs before the decision
// (sensor fusion across the two patches). A report older than fresh_ms
// counts as missing: histamine invalid (no exposure finding), no thigh fall.
#pragma once

#include <algorithm>
#include <cstdint>

#include "algo/deploy_controller.hpp"
#include "algo/vitals.hpp"
#include "patch_msgs.hpp"

namespace dev::algo {

class ThighView {
public:
	struct Config {
		uint32_t fresh_ms = 300000;
	};

	void reset(const Config &cfg)
	{
		cfg_ = cfg;
		have_ = false;
		last_ = {};
		at_ms_ = 0;
	}

	void on_report(const patch_msg::Report &r, uint32_t now_ms)
	{
		last_ = r;
		at_ms_ = now_ms;
		have_ = true;
	}

	bool fresh(uint32_t now_ms) const { return have_ && now_ms - at_ms_ <= cfg_.fresh_ms; }

	void merge(Vitals &v, uint32_t now_ms) const
	{
		if (!fresh(now_ms)) {
			v.hist_valid = false;
			v.hist_elevated = false;
			return;
		}
		v.hist_valid = (last_.flags & patch_msg::kHistValid) != 0;
		v.hist_elevated = v.hist_valid && (last_.flags & patch_msg::kHistElevated) != 0;
		v.hist_nm = last_.hist_nm;
		v.hist_baseline_nm = last_.hist_baseline_nm;
		if (last_.flags & patch_msg::kImuValid) {
			v.fall = v.fall || (last_.flags & patch_msg::kFall) != 0;
			v.activity_g = std::max(v.activity_g, last_.activity_g);
		}
	}

	DeployState state() const { return static_cast<DeployState>(last_.deploy_state); }

	// countdown or release under way on the thigh (as last reported, if fresh)
	bool alerting(uint32_t now_ms) const
	{
		return fresh(now_ms) && (state() == DeployState::Alert || state() == DeployState::Deploying);
	}

private:
	Config cfg_{};
	patch_msg::Report last_{};
	uint32_t at_ms_{};
	bool have_{};
};

} // namespace dev::algo
