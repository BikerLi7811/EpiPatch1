#include "algo/ecg_qrs.hpp"

#include <algorithm>

namespace dev::algo {

void QrsDetector::reset()
{
	bp_.init(coeffs::kEcgBandpass);
	primed_ = false;
	relearn_ = false;
	prime_ = 0.0f;
	dx_.fill(0.0f);
	mwi_buf_.fill(0.0f);
	mwi_sum_ = 0.0f;
	mwi_pos_ = 0;
	m1_ = m2_ = 0.0f;
	n_ = 0;
	learn_max_ = learn_sum_ = 0.0f;
	spki_ = npki_ = 0.0f;
	last_qrs_ = 0;
	last_qrs_val_ = 0.0f;
	have_qrs_ = false;
	noise_idx_ = 0;
	noise_max_ = 0.0f;
	rr_.clear();
	beats_ = 0;
	hr_bpm_ = 0.0f;
}

void QrsDetector::process(std::span<const float> ecg_uv, bool leads_off)
{
	if (leads_off) {
		// electrode off: everything learned so far is meaningless
		reset();
		return;
	}
	if (!primed_ && !ecg_uv.empty()) {
		prime_ = ecg_uv[0];
		primed_ = true;
	}
	std::array<float, 64> in, bp;
	while (!ecg_uv.empty()) {
		const auto len = std::min(ecg_uv.size(), bp.size());
		for (std::size_t i = 0; i < len; ++i) {
			in[i] = ecg_uv[i] - prime_;
		}
		bp_.process(std::span(in).first(len), std::span(bp).first(len));
		for (std::size_t i = 0; i < len; ++i) {
			sample(bp[i]);
		}
		ecg_uv = ecg_uv.subspan(len);
	}
	if (relearn_) {
		reset(); // at a block boundary, so the next block re-primes cleanly
	}
}

void QrsDetector::sample(float bp)
{
	// 5-point derivative, squaring, moving-window integration
	const float d = (2.0f * bp + dx_[0] - dx_[2] - 2.0f * dx_[3]) * 0.125f;
	dx_[3] = dx_[2];
	dx_[2] = dx_[1];
	dx_[1] = dx_[0];
	dx_[0] = bp;
	const float sq = d * d;
	mwi_sum_ += sq - mwi_buf_[mwi_pos_];
	mwi_buf_[mwi_pos_] = sq;
	mwi_pos_ = (mwi_pos_ + 1) % kMwiLen;
	const float mwi = std::max(mwi_sum_, 0.0f) / kMwiLen;

	if (n_ < kSettle) {
		// band-pass start-up transient
	} else if (n_ < kLearn) {
		learn_max_ = std::max(learn_max_, mwi);
		learn_sum_ += mwi;
		if (n_ + 1 == kLearn) {
			spki_ = 0.5f * learn_max_;
			npki_ = learn_sum_ / (kLearn - kSettle);
		}
	} else if (m1_ > m2_ && m1_ >= mwi) {
		on_peak(n_ - 1, m1_);
	}
	m2_ = m1_;
	m1_ = mwi;
	++n_;

	// search back for a missed beat after 1.66 x the mean RR
	if (have_qrs_ && rr_.size() >= 2 && noise_max_ > 0.0f) {
		const auto limit = static_cast<uint32_t>(1.66f * rr_.mean() * kFs);
		const float thr2 = 0.5f * (npki_ + 0.25f * (spki_ - npki_));
		if (n_ - last_qrs_ > limit && noise_max_ > thr2 && noise_idx_ - last_qrs_ >= kMinRr) {
			accept(noise_idx_, noise_max_, true);
		}
	}
	const uint32_t quiet = have_qrs_ ? n_ - last_qrs_ : n_ - kLearn;
	if (n_ > kLearn && quiet > 3 * kMaxRr) {
		// lost the rhythm (asystole, or a flat lead): re-learn the thresholds
		relearn_ = true;
	}
}

void QrsDetector::on_peak(uint32_t idx, float v)
{
	const float thr1 = npki_ + 0.25f * (spki_ - npki_);
	if (v <= thr1) {
		npki_ = 0.125f * v + 0.875f * npki_;
		if (v > noise_max_) {
			noise_max_ = v;
			noise_idx_ = idx;
		}
		return;
	}
	if (have_qrs_) {
		const uint32_t dt = idx - last_qrs_;
		if (dt < kRefractory) {
			// same complex: keep the larger peak's position
			if (v > last_qrs_val_) {
				last_qrs_val_ = v;
			}
			return;
		}
		if (dt < kTWave && v < 0.5f * last_qrs_val_) {
			npki_ = 0.125f * v + 0.875f * npki_; // T wave
			return;
		}
	}
	accept(idx, v, false);
}

void QrsDetector::accept(uint32_t idx, float v, bool searchback)
{
	spki_ = searchback ? 0.25f * v + 0.75f * spki_ : 0.125f * v + 0.875f * spki_;
	if (have_qrs_) {
		const uint32_t dt = idx - last_qrs_;
		if (dt >= kMinRr && dt <= kMaxRr) {
			const float rr = static_cast<float>(dt) / kFs;
			// drop an RR that disagrees wildly with the recent median (ectopic / missed)
			if (rr_.size() < 4 || (rr > 0.6f * rr_.median() && rr < 1.6f * rr_.median())) {
				rr_.push(rr);
			} else {
				rr_.clear();
				rr_.push(rr);
			}
			hr_bpm_ = 60.0f / rr_.median();
		}
	}
	have_qrs_ = true;
	last_qrs_ = idx;
	last_qrs_val_ = v;
	noise_max_ = 0.0f;
	++beats_;
}

bool QrsDetector::hr_valid() const
{
	return rr_.size() >= 4 && n_ - last_qrs_ < 3 * static_cast<uint32_t>(kFs);
}

} // namespace dev::algo
