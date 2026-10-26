// Fall detection and activity from the thigh patch's own LSM6DSO.
//
// The thigh can't see the trunk, so it can't tell a collapse (that is the
// chest's job). A fall here is: an impact or free-fall interrupt, then within
// fall_window_ms the thigh turns horizontal and stays still for
// fall_confirm_ms. Standing upright for recover_ms clears it. Sitting down
// normally has no impact, so it doesn't count; a hard drop into a chair might.
// The chest combines this with its own collapse/fall detection.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <span>

#include "algo/body_motion.hpp"
#include "algo/dsp.hpp"

namespace dev::algo {

class ThighMotion {
public:
	struct Config {
		Axis up = Axis::PosX;          // sensor axis toward the hip when standing
		float upright_deg = 35.0f;
		float horizontal_deg = 60.0f;
		uint32_t fall_window_ms = 5000;
		uint32_t fall_confirm_ms = 10000;
		uint32_t recover_ms = 5000;
		float still_g = 0.05f;
	};

	void reset(const Config &cfg)
	{
		cfg_ = cfg;
		g_ = {};
		tilt_ = activity_ = 0.0f;
		seen_ = have_impact_ = fall_ = false;
		last_ms_ = impact_ms_ = still_since_ = upright_since_ = 0;
		still_ = upright_ = false;
	}

	// accelerometer samples [g] at dt_ms spacing, the last one taken at t_last_ms
	void samples(std::span<const Vec3> v, uint32_t t_last_ms, uint32_t dt_ms)
	{
		const float a_g = static_cast<float>(dt_ms) / (dt_ms + 400.0f);   // gravity, ~0.4 s
		const float a_act = static_cast<float>(dt_ms) / (dt_ms + 2000.0f); // activity, ~2 s
		const auto n = static_cast<uint32_t>(v.size());
		for (uint32_t i = 0; i < n; ++i) {
			const Vec3 &a = v[i];
			const uint32_t t = t_last_ms - (n - 1 - i) * dt_ms;
			if (!seen_) {
				g_ = a;
				seen_ = true;
			}
			g_.x += a_g * (a.x - g_.x);
			g_.y += a_g * (a.y - g_.y);
			g_.z += a_g * (a.z - g_.z);
			const Vec3 d{a.x - g_.x, a.y - g_.y, a.z - g_.z};
			activity_ += a_act * (norm(d) - activity_);
			const float gn = norm(g_);
			const float c = gn > 0.2f ? std::clamp(up(g_) / gn, -1.0f, 1.0f) : 1.0f;
			tilt_ = std::acos(c) * 57.2957795f;
			last_ms_ = t;
			step(t);
		}
	}

	void impact(uint32_t t_ms)
	{
		impact_ms_ = t_ms;
		have_impact_ = true;
	}
	void freefall(uint32_t t_ms) { impact(t_ms); }

	bool valid(uint32_t now_ms) const { return seen_ && now_ms - last_ms_ < 10000; }
	bool fall() const { return fall_; }
	float activity_g() const { return seen_ ? activity_ : 0.0f; }
	float tilt_deg() const { return tilt_; }

private:
	float up(const Vec3 &v) const
	{
		switch (cfg_.up) {
		case Axis::PosX: return v.x;
		case Axis::NegX: return -v.x;
		case Axis::PosY: return v.y;
		case Axis::NegY: return -v.y;
		case Axis::PosZ: return v.z;
		case Axis::NegZ: return -v.z;
		}
		return 0.0f;
	}
	static float norm(const Vec3 &v) { return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z); }

	void step(uint32_t t)
	{
		const bool horizontal = tilt_ > cfg_.horizontal_deg;
		const bool still = horizontal && activity_ < cfg_.still_g;
		if (still && !still_) {
			still_since_ = t;
		}
		still_ = still;

		if (have_impact_ && !fall_) {
			if (after_eq(t, impact_ms_ + cfg_.fall_window_ms) && !horizontal) {
				have_impact_ = false; // got up again (or never went down)
			} else if (still && after_eq(t, impact_ms_) &&
				   after_eq(t, std::max(still_since_, impact_ms_) + cfg_.fall_confirm_ms) &&
				   after_eq(impact_ms_ + cfg_.fall_window_ms, still_since_)) {
				fall_ = true;
				have_impact_ = false;
			}
		}

		const bool upright = tilt_ < cfg_.upright_deg;
		if (upright && !upright_) {
			upright_since_ = t;
		}
		upright_ = upright;
		if (fall_ && upright && after_eq(t, upright_since_ + cfg_.recover_ms)) {
			fall_ = false;
		}
	}

	Config cfg_{};
	Vec3 g_{};
	float tilt_{}, activity_{};
	bool seen_{}, have_impact_{}, fall_{}, still_{}, upright_{};
	uint32_t last_ms_{}, impact_ms_{}, still_since_{}, upright_since_{};
};

} // namespace dev::algo
