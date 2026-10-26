#include "feature_extractor.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace wheeze {

void FeatureExtractor::init(const dsp::MicRate &rate, float gate_tonality)
{
	decim_state_.fill(0.0f);
	hist_.fill(0.0f);
	hist_fill_ = 0;
	head_ = 0;
	frames_total_ = 0;
	tonality_.fill(0.0f);
	is_tonal_.fill(0);
	tonal_count_ = 0;
	gate_tonality_ = gate_tonality;
	decim_ = rate.decim;
	arm_fir_decimate_init_f32(&fir_, rate.taps, rate.decim, rate.fir, decim_state_.data(),
				  kDecimBlock);
	arm_rfft_fast_init_f32(&rfft_, dsp::kNfft);
}

void FeatureExtractor::compute_frame()
{
	static std::array<float, dsp::kNfft> buf;
	static std::array<float, dsp::kNfft> spec;
	static std::array<float, dsp::kNfft / 2 + 1> power;
	auto &row = frames_[head_];

	arm_mult_f32(hist_.data(), dsp::hann, buf.data(), dsp::kNfft);
	arm_rfft_fast_f32(&rfft_, buf.data(), spec.data(), 0);

	/* CMSIS packs Re[0] and Re[N/2] into spec[0], spec[1]. */
	power[0] = spec[0] * spec[0];
	power[dsp::kNfft / 2] = spec[1] * spec[1];
	arm_cmplx_mag_squared_f32(&spec[2], &power[1], dsp::kNfft / 2 - 1);

	for (std::size_t m = 0; m < dsp::kNumMels; m++) {
		arm_dot_prod_f32(&power[dsp::mel_start[m]], &dsp::mel_w[dsp::mel_off[m]],
				 dsp::mel_len[m], &row[m]);
	}
	arm_offset_f32(row.data(), dsp::kLogEps, row.data(), dsp::kNumMels);
	for (auto &v : row) {
		v = std::log(v);
	}

	/* Peak prominence vs. bands two away: wheezes are narrow-band (see
	 * features.tonality). Kept as a running count so the gate is O(1). */
	float ton = -INFINITY;
	for (std::size_t m = 2; m + 2 < dsp::kNumMels; m++) {
		ton = std::fmax(ton, row[m] - 0.5f * (row[m - 2] + row[m + 2]));
	}
	const uint8_t tonal = ton >= gate_tonality_;
	tonal_count_ += tonal - is_tonal_[head_];
	is_tonal_[head_] = tonal;
	tonality_[head_] = ton;

	head_ = (head_ + 1) % dsp::kNumFrames;
	frames_total_++;
}

uint32_t FeatureExtractor::push_4k(std::span<const float> x)
{
	const uint32_t before = frames_total_;

	while (!x.empty()) {
		const std::size_t take = std::min<std::size_t>(dsp::kNfft - hist_fill_, x.size());

		std::copy_n(x.begin(), take, hist_.begin() + hist_fill_);
		hist_fill_ += take;
		x = x.subspan(take);
		if (hist_fill_ == dsp::kNfft) {
			compute_frame();
			std::memmove(hist_.data(), hist_.data() + dsp::kHop,
				     (dsp::kNfft - dsp::kHop) * sizeof(float));
			hist_fill_ = dsp::kNfft - dsp::kHop;
		}
	}
	return frames_total_ - before;
}

uint32_t FeatureExtractor::push_pcm(std::span<const int16_t> pcm)
{
	static std::array<float, kDecimBlock> in;
	static std::array<float, kDecimBlock> out; /* sized for the smallest ratio */
	uint32_t frames = 0;

	/* Callers deliver whole blocks; a trailing partial block is dropped. */
	for (; pcm.size() >= kDecimBlock; pcm = pcm.subspan(kDecimBlock)) {
		arm_q15_to_float(reinterpret_cast<const q15_t *>(pcm.data()), in.data(), kDecimBlock);
		arm_fir_decimate_f32(&fir_, in.data(), out.data(), kDecimBlock);
		frames += push_4k(std::span<const float>(out.data(), kDecimBlock / decim_));
	}
	return frames;
}

void FeatureExtractor::window(std::span<float, kWindowSize> out) const
{
	/* head_ is the oldest row once the ring is full */
	float *dst = out.data();
	float mean;

	for (uint32_t i = 0; i < dsp::kNumFrames; i++) {
		const auto &row = frames_[(head_ + i) % dsp::kNumFrames];

		dst = std::copy(row.begin(), row.end(), dst);
	}
	arm_mean_f32(out.data(), kWindowSize, &mean);
	arm_offset_f32(out.data(), -mean, out.data(), kWindowSize);
}

void quantize(std::span<const float> in, std::span<int8_t> out, float scale, int32_t zp)
{
	const float inv = 1.0f / scale;

	for (std::size_t i = 0; i < in.size(); i++) {
		const auto q = static_cast<int32_t>(std::lrint(in[i] * inv)) + zp;

		out[i] = static_cast<int8_t>(std::clamp<int32_t>(q, -128, 127));
	}
}

} // namespace wheeze
