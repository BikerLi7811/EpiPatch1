// Small DSP building blocks shared by the vital-sign algorithms.
// Pure C++ + CMSIS-DSP (no Zephyr), so they run unchanged in the host tests.
// On the RISC-V FLPR coprocessor (no CMSIS-DSP) the biquad is plain C++
// (DEV_PORTABLE_DSP forces that version elsewhere, e.g. to test it).
#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#if defined(__riscv) && !defined(DEV_PORTABLE_DSP)
#define DEV_PORTABLE_DSP 1
#endif

#if !defined(DEV_PORTABLE_DSP)
#include <arm_math.h>
#endif

namespace dev::algo {

#if defined(DEV_PORTABLE_DSP)

// Biquad cascade, transposed direct form II, same coefficient layout as
// CMSIS-DSP: {b0, b1, b2, a1, a2} per stage with a1/a2 already negated.
template <std::size_t Stages>
class Biquad {
public:
	void init(const std::array<float, 5 * Stages> &coeffs)
	{
		coeffs_ = coeffs;
		state_.fill(0.0f);
	}

	// in and out must not overlap
	void process(std::span<const float> in, std::span<float> out)
	{
		for (std::size_t i = 0; i < in.size(); ++i) {
			out[i] = step(in[i]);
		}
	}

	float step(float x)
	{
		for (std::size_t s = 0; s < Stages; ++s) {
			const float *c = &coeffs_[5 * s];
			float *d = &state_[2 * s];
			const float y = c[0] * x + d[0];
			d[0] = c[1] * x + c[3] * y + d[1];
			d[1] = c[2] * x + c[4] * y;
			x = y;
		}
		return x;
	}

private:
	std::array<float, 5 * Stages> coeffs_{};
	std::array<float, 2 * Stages> state_{};
};

#else

// CMSIS-DSP biquad cascade (transposed direct form II, FPU).
template <std::size_t Stages>
class Biquad {
public:
	void init(const std::array<float, 5 * Stages> &coeffs)
	{
		state_.fill(0.0f);
		arm_biquad_cascade_df2T_init_f32(&inst_, Stages, coeffs.data(), state_.data());
	}

	// in and out must not overlap
	void process(std::span<const float> in, std::span<float> out)
	{
		arm_biquad_cascade_df2T_f32(&inst_, in.data(), out.data(), static_cast<uint32_t>(in.size()));
	}

	float step(float x)
	{
		float y;
		arm_biquad_cascade_df2T_f32(&inst_, &x, &y, 1);
		return y;
	}

private:
	arm_biquad_cascade_df2T_instance_f32 inst_{};
	std::array<float, 2 * Stages> state_{};
};

#endif

// Fixed-capacity history of the most recent N values (oldest dropped).
template <typename T, std::size_t N>
class History {
public:
	void clear() { size_ = 0; }
	void push(T v)
	{
		if (size_ == N) {
			std::copy(buf_.begin() + 1, buf_.end(), buf_.begin());
			buf_[N - 1] = v;
		} else {
			buf_[size_++] = v;
		}
	}
	std::size_t size() const { return size_; }
	T back() const { return buf_[size_ - 1]; }
	T operator[](std::size_t i) const { return buf_[i]; }

	T median() const
	{
		std::array<T, N> tmp = buf_;
		auto mid = tmp.begin() + size_ / 2;
		std::nth_element(tmp.begin(), mid, tmp.begin() + size_);
		return *mid;
	}

	T mean() const
	{
		T s{};
		for (std::size_t i = 0; i < size_; ++i) {
			s += buf_[i];
		}
		return s / static_cast<T>(size_);
	}

private:
	std::array<T, N> buf_{};
	std::size_t size_ = 0;
};

// Wrap-safe "a happened at or after b" for 32-bit millisecond/sample clocks.
constexpr bool after_eq(uint32_t a, uint32_t b) { return static_cast<int32_t>(a - b) >= 0; }

} // namespace dev::algo
