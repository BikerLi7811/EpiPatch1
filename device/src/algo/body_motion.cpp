#include "algo/body_motion.hpp"

#include <algorithm>
#include <cmath>

#include "algo/dsp.hpp"

namespace dev::algo {

namespace {

constexpr uint32_t kStaleMs = 5000;
constexpr float kRadToDeg = 57.2957795f;

float up_component(const Vec3 &v, Axis a)
{
	switch (a) {
	case Axis::PosX: return v.x;
	case Axis::NegX: return -v.x;
	case Axis::PosY: return v.y;
	case Axis::NegY: return -v.y;
	case Axis::PosZ: return v.z;
	case Axis::NegZ: return -v.z;
	}
	return 0.0f;
}

float norm(const Vec3 &v) { return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z); }

} // namespace

void BodyMotion::reset(const Config &cfg)
{
	cfg_ = cfg;
	chest_ = {};
	thigh_ = {};
	was_horizontal_ = false;
	last_upright_ms_ = 0;
	lying_since_ms_ = 0;
	fast_drop_ = false;
	last_active_ms_ = 0;
	trigger_ms_ = 0;
	have_trigger_ = false;
	upright_since_ms_ = 0;
	posture_ = Posture::Unknown;
	collapsed_ = fall_ = false;
}

void BodyMotion::chest(std::span<const Vec3> s, uint32_t t_last_ms, uint32_t dt_ms)
{
	feed(chest_, cfg_.chest_up, s, t_last_ms, dt_ms, true);
}

void BodyMotion::thigh(std::span<const Vec3> s, uint32_t t_last_ms, uint32_t dt_ms)
{
	feed(thigh_, cfg_.thigh_up, s, t_last_ms, dt_ms, false);
}

void BodyMotion::feed(Sensor &st, Axis up, std::span<const Vec3> v, uint32_t t_last_ms,
		      uint32_t dt_ms, bool is_chest)
{
	const float a_g = static_cast<float>(dt_ms) / (dt_ms + 400.0f);   // gravity, ~0.4 s
	const float a_act = static_cast<float>(dt_ms) / (dt_ms + 2000.0f); // activity, ~2 s
	const auto n = static_cast<uint32_t>(v.size());
	for (uint32_t i = 0; i < n; ++i) {
		const Vec3 &a = v[i];
		const uint32_t t = t_last_ms - (n - 1 - i) * dt_ms;
		if (!st.seen) {
			st.g = a;
			st.seen = true;
		}
		st.g.x += a_g * (a.x - st.g.x);
		st.g.y += a_g * (a.y - st.g.y);
		st.g.z += a_g * (a.z - st.g.z);
		const Vec3 d{a.x - st.g.x, a.y - st.g.y, a.z - st.g.z};
		st.activity += a_act * (norm(d) - st.activity);
		const float gn = norm(st.g);
		const float c = gn > 0.2f ? std::clamp(up_component(st.g, up) / gn, -1.0f, 1.0f) : 1.0f;
		st.tilt = std::acos(c) * kRadToDeg;
		st.last_ms = t;
		if (is_chest) {
			if (st.activity > cfg_.still_g) {
				last_active_ms_ = t;
			}
			chest_tilt_at(t);
		}
	}
}

void BodyMotion::chest_tilt_at(uint32_t t)
{
	const bool upright = chest_.tilt < cfg_.upright_deg;
	const bool horizontal = chest_.tilt > cfg_.horizontal_deg;
	if (upright) {
		if (!after_eq(last_upright_ms_ + 1000, t) || upright_since_ms_ == 0) {
			upright_since_ms_ = t; // new upright run
		}
		last_upright_ms_ = t;
	}
	if (horizontal && !was_horizontal_) {
		lying_since_ms_ = t;
		fast_drop_ = last_upright_ms_ != 0 && t - last_upright_ms_ <= cfg_.collapse_transition_ms;
	}
	was_horizontal_ = horizontal;
}

void BodyMotion::impact(uint32_t t_ms)
{
	trigger_ms_ = t_ms;
	have_trigger_ = true;
}

void BodyMotion::freefall(uint32_t t_ms) { impact(t_ms); }

float BodyMotion::activity_g() const
{
	float a = 0.0f;
	if (chest_.seen) {
		a = std::max(a, chest_.activity);
	}
	if (thigh_.seen) {
		a = std::max(a, thigh_.activity);
	}
	return a;
}

void BodyMotion::update(uint32_t now_ms)
{
	const bool chest_ok = chest_.seen && now_ms - chest_.last_ms < kStaleMs;
	const bool thigh_ok = thigh_.seen && now_ms - thigh_.last_ms < kStaleMs;

	if (!chest_ok) {
		posture_ = Posture::Unknown;
	} else if (was_horizontal_) {
		posture_ = Posture::Lying;
	} else if (chest_.tilt < cfg_.upright_deg) {
		if (!thigh_ok) {
			posture_ = Posture::Standing;
		} else if (thigh_.tilt < cfg_.upright_deg) {
			posture_ = Posture::Standing;
		} else if (thigh_.tilt > cfg_.horizontal_deg) {
			posture_ = Posture::Sitting;
		} else {
			posture_ = Posture::Transition;
		}
	} else {
		posture_ = Posture::Transition;
	}

	const bool lying = chest_ok && was_horizontal_;

	// fall: impact/free-fall close to the moment the trunk went horizontal,
	// having been upright just before, and still down fall_confirm_ms later
	if (!fall_ && lying && have_trigger_) {
		const auto gap = static_cast<int32_t>(lying_since_ms_ - trigger_ms_);
		const bool near = std::abs(gap) <= static_cast<int32_t>(cfg_.fall_window_ms);
		const bool was_up = last_upright_ms_ != 0 &&
				    after_eq(last_upright_ms_, trigger_ms_ - cfg_.fall_window_ms);
		if (near && was_up && after_eq(now_ms, trigger_ms_ + cfg_.fall_confirm_ms)) {
			fall_ = true;
		}
	}

	// collapse: went down fast (or fell) and has not moved since
	if (!collapsed_ && lying && (fast_drop_ || fall_)) {
		const uint32_t since =
			after_eq(last_active_ms_, lying_since_ms_) ? last_active_ms_ : lying_since_ms_;
		if (after_eq(now_ms, since + cfg_.collapse_still_ms)) {
			collapsed_ = true;
		}
	}

	// back on their feet (or sitting up) for a while: clear
	const bool upright_now = chest_ok && chest_.tilt < cfg_.upright_deg;
	if (upright_now && upright_since_ms_ != 0 &&
	    after_eq(now_ms, upright_since_ms_ + cfg_.recover_ms)) {
		collapsed_ = fall_ = false;
		fast_drop_ = false;
		if (have_trigger_ && after_eq(now_ms, trigger_ms_ + cfg_.fall_window_ms)) {
			have_trigger_ = false;
		}
	}
}

} // namespace dev::algo
