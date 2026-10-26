#include "algo/anaphylaxis.hpp"

#include <algorithm>

#include "algo/dsp.hpp"

namespace dev::algo {

bool AnaphylaxisEngine::Hold::update(bool cond, uint32_t now, uint32_t hold_ms)
{
	if (!cond) {
		on = false;
		return false;
	}
	if (!on) {
		on = true;
		since = now;
	}
	return after_eq(now, since + hold_ms);
}

void AnaphylaxisEngine::reset(const Config &cfg)
{
	cfg_ = cfg;
	if (cfg_.wheeze_min_s < 1 || cfg_.wheeze_min_s > kMaxWheezeSeconds) {
		cfg_.wheeze_min_s = 3;
	}
	tachypnea_ = slow_rr_ = tachycardia_ = perfusion_ = {};
	hr_base_ = pi_base_ = 0.0f;
	hr_rest_ms_ = pi_rest_ms_ = 0;
	last_ms_ = 0;
	started_ = false;
	wz_n_ = 0;
	wz_ok_ = false;
	hist_seen_ = false;
}

bool AnaphylaxisEngine::wheeze_finding(const Vitals &v, uint32_t now)
{
	// a second counts if the detector is in a wheeze or a new event was counted
	const bool heard = v.wheezing || (started_ && v.wheeze_events != wz_events_);
	wz_events_ = v.wheeze_events;
	uint8_t keep = 0;
	for (uint8_t i = 0; i < wz_n_; ++i) {
		if (now - wz_t_[i] < cfg_.wheeze_window_ms) {
			wz_t_[keep++] = wz_t_[i];
		}
	}
	wz_n_ = keep;
	if (heard) {
		if (wz_n_ == kMaxWheezeSeconds) {
			for (uint8_t i = 1; i < wz_n_; ++i) {
				wz_t_[i - 1] = wz_t_[i];
			}
			--wz_n_;
		}
		wz_t_[wz_n_++] = now;
	}
	if (wz_n_ >= cfg_.wheeze_min_s) {
		wz_ok_ = true;
		wz_ok_ms_ = now;
	}
	return wz_ok_ && now - wz_ok_ms_ <= cfg_.wheeze_recent_ms;
}

Assessment AnaphylaxisEngine::update(const Vitals &v, uint32_t now_ms)
{
	const uint32_t dt = started_ ? now_ms - last_ms_ : 0;
	const bool wheeze = wheeze_finding(v, now_ms);
	started_ = true;
	last_ms_ = now_ms;

	const bool moving = v.activity_g > cfg_.active_g;
	const bool resting = v.activity_g < cfg_.rest_g;
	uint16_t f = 0;

	// respiratory
	if (wheeze) {
		f |= kWheeze;
	}
	if (tachypnea_.update(v.rr_valid && !moving && v.rr_bpm >= cfg_.rr_high_bpm, now_ms,
			      cfg_.rr_high_hold_ms)) {
		f |= kTachypnea;
	}
	// apnea is already time-qualified by the breathing detector
	if (v.apnea || slow_rr_.update(v.rr_valid && !moving && v.rr_bpm <= cfg_.rr_low_bpm, now_ms,
				       cfg_.rr_low_hold_ms)) {
		f |= kApnea;
	}

	// cardiovascular
	float hr_limit = cfg_.hr_high_bpm;
	if (hr_baseline_valid()) {
		hr_limit = std::max(hr_limit, hr_base_ + cfg_.hr_rise_bpm);
	}
	if (tachycardia_.update(v.hr_valid && !moving && v.hr_bpm >= hr_limit, now_ms, cfg_.hr_hold_ms)) {
		f |= kTachycardia;
	}
	const bool pi_base_ok = pi_rest_ms_ >= cfg_.baseline_min_ms && pi_base_ > 0.0f;
	if (perfusion_.update(v.ppg_valid && !moving && pi_base_ok && v.pi_pct <= cfg_.pi_drop_ratio * pi_base_,
			      now_ms, cfg_.pi_hold_ms)) {
		f |= kPerfusionDrop;
	}
	if (v.collapsed) {
		f |= kCollapse;
	}
	if (v.fall) {
		f |= kFall;
	}
	if (v.hist_valid && v.hist_elevated) {
		hist_seen_ = true;
		hist_ms_ = now_ms;
	}
	if (hist_seen_ && now_ms - hist_ms_ <= cfg_.exposure_hold_ms) {
		f |= kHistamine;
	}

	Assessment a{};
	a.findings = f;
	a.exposure = f & kHistamine;
	a.respiratory = f & (kWheeze | kTachypnea | kApnea);
	a.cardiovascular = f & (kTachycardia | kPerfusionDrop | kCollapse | kFall);
	const bool down = f & (kCollapse | kFall);
	a.severe = a.respiratory && down;
	a.trigger = (cfg_.rule_exposure && a.exposure && (a.respiratory || a.cardiovascular)) ||
		    (cfg_.rule_resp_collapse && a.severe);

	// personal baselines: learned only at rest with nothing abnormal going on
	// (fast ~30 s average until the first baseline is established, then slow)
	const float secs = dt / 1000.0f;
	const auto alpha = [&](uint32_t rest_ms) {
		const float tau = rest_ms < cfg_.baseline_min_ms ? 30.0f : cfg_.baseline_tau_s;
		return std::min(1.0f, secs / tau);
	};
	if (resting && f == 0) {
		if (v.hr_valid) {
			hr_base_ = hr_rest_ms_ == 0 ? v.hr_bpm : hr_base_ + alpha(hr_rest_ms_) * (v.hr_bpm - hr_base_);
			hr_rest_ms_ = std::min(hr_rest_ms_ + std::max(dt, 1u), cfg_.baseline_min_ms);
		}
		if (v.ppg_valid) {
			pi_base_ = pi_rest_ms_ == 0 ? v.pi_pct : pi_base_ + alpha(pi_rest_ms_) * (v.pi_pct - pi_base_);
			pi_rest_ms_ = std::min(pi_rest_ms_ + std::max(dt, 1u), cfg_.baseline_min_ms);
		}
	}
	return a;
}

} // namespace dev::algo
