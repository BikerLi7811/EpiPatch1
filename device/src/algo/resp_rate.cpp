#include "algo/resp_rate.hpp"

#include <algorithm>
#include <cmath>

namespace dev::algo {

namespace {
constexpr uint32_t kWarmup = 50;       // 10 s for the filter to settle
constexpr float kEnvAlpha = 1.0f / 50; // ~10 s envelope
constexpr float kMinIv = 1.2f;         // 50 /min
constexpr float kMaxIv = 15.0f;        // 4 /min
} // namespace

void RespRate::reset(const Config &cfg)
{
	cfg_ = cfg;
	bp_.init(coeffs::kRespBandpass);
	acc_ = 0.0f;
	acc_n_ = 0;
	primed_ = false;
	prime_ = 0.0f;
	t_ = 0;
	env_ = 0.0f;
	armed_ = false;
	hi_ = lo_ = 0.0f;
	have_breath_ = false;
	last_breath_ = 0;
	breath_amp_ = 0.0f;
	iv_.clear();
	rate_bpm_ = 0.0f;
}

void RespRate::process(std::span<const float> x, bool leads_off)
{
	if (leads_off) {
		reset(cfg_);
		return;
	}
	for (float v : x) {
		acc_ += v;
		if (++acc_n_ == kDecim) {
			sample(acc_ / kDecim);
			acc_ = 0.0f;
			acc_n_ = 0;
		}
	}
}

void RespRate::sample(float x)
{
	if (!primed_) {
		prime_ = x; // remove the large electrode/body DC before the high-pass
		primed_ = true;
	}
	const float y = bp_.step(x - prime_);
	++t_;
	if (t_ < kWarmup) {
		return;
	}
	env_ += kEnvAlpha * (std::fabs(y) - env_);
	// hysteresis: half the mean |y|, but never below a tenth of a real breath
	const float h = std::max(0.5f * env_, 0.1f * breath_amp_ * 0.5f);
	hi_ = std::max(hi_, y);
	lo_ = std::min(lo_, y);
	if (y < -h) {
		armed_ = true;
	} else if (armed_ && y > h) {
		armed_ = false;
		breath(t_);
	}
}

void RespRate::breath(uint32_t t)
{
	const float p2p = hi_ - lo_;
	hi_ = lo_ = 0.0f;
	if (have_breath_) {
		const float iv = static_cast<float>(t - last_breath_) / kFs;
		if (iv >= kMinIv && iv <= kMaxIv) {
			iv_.push(iv);
			rate_bpm_ = 60.0f / iv_.median();
			breath_amp_ = breath_amp_ == 0.0f ? p2p : 0.8f * breath_amp_ + 0.2f * p2p;
		}
	}
	have_breath_ = true;
	last_breath_ = t;
}

bool RespRate::valid() const
{
	if (iv_.size() < 3) {
		return false;
	}
	// stale once we've waited twice the usual interval
	return static_cast<float>(t_ - last_breath_) / kFs < 2.0f * iv_.median() + 2.0f;
}

bool RespRate::apnea() const
{
	// only after we've seen real breathing (not while warming up / leads off)
	return iv_.size() >= 3 && static_cast<float>(t_ - last_breath_) / kFs * 1000.0f >= cfg_.apnea_ms;
}

} // namespace dev::algo
