#include "algo/ppg.hpp"

#include <algorithm>

namespace dev::algo {

namespace {
constexpr float kDcAlpha = 1.0f / (1.5f * PpgProcessor::kFs); // 1.5 s
constexpr uint32_t kWarmup = 50;                               // 2 s
constexpr uint32_t kRefractory = 8;                            // 0.32 s = 187 /min
constexpr uint32_t kMaxIv = 50;                                // 2 s = 30 /min
} // namespace

void PpgProcessor::reset(const Config &cfg)
{
	cfg_ = cfg;
	bp_ir_.init(coeffs::kPpgBandpass);
	primed_ = false;
	prime_ir_ = prime_red_ = 0.0f;
	dc_ir_ = dc_red_ = 0.0f;
	contact_ = false;
	t_ = 0;
	s1_ = s2_ = 0.0f;
	pk_env_ = 0.0f;
	hi_ir_ = lo_ir_ = hi_red_ = lo_red_ = 0.0f;
	have_beat_ = false;
	last_beat_ = 0;
	iv_.clear();
	ac_ir_.clear();
	ac_red_.clear();
	pi_pct_ = pulse_bpm_ = spo2_ = 0.0f;
}

void PpgProcessor::process(std::span<const uint32_t> red, std::span<const uint32_t> ir)
{
	const auto n = std::min(red.size(), ir.size());
	for (std::size_t i = 0; i < n; ++i) {
		sample(static_cast<float>(red[i]), static_cast<float>(ir[i]));
	}
}

void PpgProcessor::sample(float red, float ir)
{
	if (!primed_) {
		prime_ir_ = dc_ir_ = ir;
		prime_red_ = dc_red_ = red;
		primed_ = true;
	}
	dc_ir_ += kDcAlpha * (ir - dc_ir_);
	dc_red_ += kDcAlpha * (red - dc_red_);
	const bool contact = dc_ir_ > cfg_.min_dc && dc_ir_ < 0.97f * kFullScale;
	if (contact != contact_) {
		// sensor put on / taken off: restart beat statistics
		contact_ = contact;
		const auto c = cfg_;
		reset(c);
		contact_ = contact;
		prime_ir_ = dc_ir_ = ir;
		prime_red_ = dc_red_ = red;
		primed_ = true;
	}
	// more blood absorbs more light, so the systolic peak is a dip: invert
	const float s = -bp_ir_.step(ir - prime_ir_);
	const float hp_ir = ir - dc_ir_, hp_red = red - dc_red_;
	// beat amplitude from the DC-removed signal itself: the band-pass would
	// shave off the pulse harmonics (the sensor already averages 2 samples)
	const float a_ir = hp_ir, a_red = hp_red;
	++t_;
	if (t_ < kWarmup || !contact_) {
		s2_ = s1_;
		s1_ = s;
		return;
	}
	hi_ir_ = std::max(hi_ir_, a_ir);
	lo_ir_ = std::min(lo_ir_, a_ir);
	hi_red_ = std::max(hi_red_, a_red);
	lo_red_ = std::min(lo_red_, a_red);
	if (have_beat_ && t_ - last_beat_ > kMaxIv / 2) {
		pk_env_ *= 0.95f; // weaker pulse (vasoconstriction, shock): let the threshold come down
	}
	if (s1_ > s2_ && s1_ >= s && s1_ > 0.0f && s1_ > 0.4f * pk_env_ &&
	    (!have_beat_ || t_ - 1 - last_beat_ >= kRefractory)) {
		beat(hi_ir_ - lo_ir_, hi_red_ - lo_red_);
		pk_env_ = pk_env_ == 0.0f ? s1_ : 0.75f * pk_env_ + 0.25f * s1_;
		hi_ir_ = lo_ir_ = a_ir;
		hi_red_ = lo_red_ = a_red;
	}
	s2_ = s1_;
	s1_ = s;
}

void PpgProcessor::beat(float ac_ir, float ac_red)
{
	const uint32_t t = t_ - 1;
	if (have_beat_) {
		const uint32_t dt = t - last_beat_;
		if (dt <= kMaxIv) {
			iv_.push(static_cast<float>(dt) / kFs);
			pulse_bpm_ = 60.0f / iv_.median();
			ac_ir_.push(ac_ir);
			ac_red_.push(ac_red);
			pi_pct_ = 100.0f * ac_ir_.mean() / dc_ir_;
			const float ratio = (ac_red_.mean() / dc_red_) / (ac_ir_.mean() / dc_ir_);
			spo2_ = std::clamp(110.0f - 25.0f * ratio, 0.0f, 100.0f);
		}
	}
	have_beat_ = true;
	last_beat_ = t;
}

bool PpgProcessor::valid() const
{
	return contact_ && iv_.size() >= 3 && t_ - last_beat_ < static_cast<uint32_t>(3 * kFs);
}

} // namespace dev::algo
