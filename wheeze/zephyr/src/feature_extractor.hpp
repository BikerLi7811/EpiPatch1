/*
 * Feature front-end: 16 kHz PCM -> 4 kHz -> log-mel frames -> int8 CNN input.
 * Mirrors training/features.py exactly. Uses the M33 FPU via CMSIS-DSP f32.
 */
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "arm_math.h"
#include "generated/dsp_tables.hpp"

namespace wheeze {

inline constexpr std::size_t kWindowSize = dsp::kNumFrames * dsp::kNumMels;

class FeatureExtractor {
public:
	/* PCM samples per decimator call; must be a multiple of every MicRate::decim. */
	static constexpr std::size_t kDecimBlock = 256;
	static_assert(kDecimBlock % 4 == 0);

	/* Explicit init instead of a constructor: avoids static-init-order issues
	 * for the global instance and keeps the object trivially placeable in .bss.
	 * Call again (e.g. after the mic was powered down) to start a fresh stream. */
	void init(const dsp::MicRate &rate, float gate_tonality);

	/* Feed mic PCM at rate.fs. Whole kDecimBlock chunks only; returns new frames. */
	uint32_t push_pcm(std::span<const int16_t> pcm);

	/* Feed already-decimated 4 kHz samples (host tests / other front-ends). */
	uint32_t push_4k(std::span<const float> x);

	bool window_ready() const { return frames_total_ >= dsp::kNumFrames; }
	uint32_t frames_total() const { return frames_total_; }

	/* Frames in the current window whose tonality >= gate_tonality (cheap gate). */
	uint32_t tonal_frames() const { return tonal_count_; }

	/* Tonality of the frame produced `age` frames ago (1 = newest). */
	float frame_tonality(uint32_t age) const
	{
		return tonality_[(head_ + dsp::kNumFrames - age) % dsp::kNumFrames];
	}

	/* Latest window (oldest -> newest), mean-normalized. */
	void window(std::span<float, kWindowSize> out) const;

	/* Row of the frame produced `age` frames ago (1 = newest). */
	std::span<const float, dsp::kNumMels> frame(uint32_t age) const
	{
		return frames_[(head_ + dsp::kNumFrames - age) % dsp::kNumFrames];
	}

private:
	void compute_frame();

	arm_fir_decimate_instance_f32 fir_;
	std::array<float, dsp::kMaxDecimTaps + kDecimBlock - 1> decim_state_;
	uint16_t decim_; /* set by init(); no initialiser keeps the global trivially constructed */
	arm_rfft_fast_instance_f32 rfft_;

	std::array<float, dsp::kNfft> hist_;     /* sliding 4 kHz analysis buffer */
	uint32_t hist_fill_;

	std::array<std::array<float, dsp::kNumMels>, dsp::kNumFrames> frames_; /* ring */
	uint32_t head_;                                                        /* oldest row */
	uint32_t frames_total_;

	std::array<float, dsp::kNumFrames> tonality_;    /* per frame, same ring index */
	std::array<uint8_t, dsp::kNumFrames> is_tonal_;
	uint32_t tonal_count_;
	float gate_tonality_;
};

/* Quantize a normalized float window to the CNN's int8 input. */
void quantize(std::span<const float> in, std::span<int8_t> out, float scale, int32_t zp);

} // namespace wheeze
