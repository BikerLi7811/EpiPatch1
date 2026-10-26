#include "wheeze_net.hpp"

#include <cmath>
#include <numeric>

#include "arm_nnfunctions.h"

namespace wheeze {

bool WheezeNet::conv(const int8_t *in, int h, int w, int cin, int32_t in_zp, const ConvLayer &l,
		     int8_t *out)
{
	const cmsis_nn_conv_params p = {
		.input_offset = -in_zp,
		.output_offset = l.out_zp,
		.stride = {1, 1},
		.padding = {1, 1},
		.dilation = {1, 1},
		.activation = {l.out_zp, 127}, /* fused ReLU: clamp at real 0 */
	};
	/* CMSIS-NN takes non-const pointers but never writes through them. */
	const cmsis_nn_per_channel_quant_params q = {const_cast<int32_t *>(l.mult),
						     const_cast<int32_t *>(l.shift)};
	const cmsis_nn_dims in_d = {1, h, w, cin};
	const cmsis_nn_dims f_d = {l.cout, 3, 3, cin};
	const cmsis_nn_dims b_d = {1, 1, 1, l.cout};
	const cmsis_nn_dims o_d = {1, h, w, l.cout};
	cmsis_nn_context ctx = {scratch_.data(),
				arm_convolve_wrapper_s8_get_buffer_size(&p, &in_d, &f_d, &o_d)};

	if (ctx.size > static_cast<int32_t>(scratch_.size())) {
		return false;
	}
	return arm_convolve_wrapper_s8(&ctx, &p, &q, &in_d, in, &f_d, l.weights, &b_d, l.bias,
				       &o_d, out) == ARM_CMSIS_NN_SUCCESS;
}

bool WheezeNet::maxpool2(const int8_t *in, int h, int w, int c, int8_t *out)
{
	const cmsis_nn_pool_params p = {.stride = {2, 2}, .padding = {0, 0}, .activation = {-128, 127}};
	const cmsis_nn_dims in_d = {1, h, w, c};
	const cmsis_nn_dims f_d = {1, 2, 2, 1};
	const cmsis_nn_dims o_d = {1, h / 2, w / 2, c};
	cmsis_nn_context ctx = {nullptr, 0};

	return arm_max_pool_s8(&ctx, &p, &in_d, in, &f_d, &o_d, out) == ARM_CMSIS_NN_SUCCESS;
}

std::optional<float> WheezeNet::run(std::span<const int8_t, kWindowSize> in)
{
	using namespace model;
	static constexpr ConvLayer c1{c1_w, c1_b, c1_mult, c1_shift, kW1, kC1Zp};
	static constexpr ConvLayer c2{c2_w, c2_b, c2_mult, c2_shift, kW2, kC2Zp};
	static constexpr ConvLayer c3{c3_w, c3_b, c3_mult, c3_shift, kW3, kC3Zp};
	alignas(4) std::array<int8_t, kW3> gap;
	int8_t logit_q;

	if (!conv(in.data(), kRows0, kCols0, 1, kInZp, c1, a_.data()) ||
	    !maxpool2(a_.data(), kRows0, kCols0, kW1, b_.data()) ||
	    !conv(b_.data(), kRows1, kCols1, kW1, kC1Zp, c2, a_.data()) ||
	    !maxpool2(a_.data(), kRows1, kCols1, kW2, b_.data()) ||
	    !conv(b_.data(), kRows2, kCols2, kW2, kC2Zp, c3, a_.data())) {
		return std::nullopt;
	}

	{ /* global average pool 10x8 -> 1x1, same quantization in/out */
		const cmsis_nn_pool_params p = {.stride = {1, 1}, .padding = {0, 0},
						.activation = {-128, 127}};
		const cmsis_nn_dims in_d = {1, kRows2, kCols2, kW3};
		const cmsis_nn_dims f_d = {1, kRows2, kCols2, 1};
		const cmsis_nn_dims o_d = {1, 1, 1, kW3};
		cmsis_nn_context ctx = {scratch_.data(), arm_avgpool_s8_get_buffer_size(1, kW3)};

		if (arm_avgpool_s8(&ctx, &p, &in_d, a_.data(), &f_d, &o_d, gap.data()) !=
		    ARM_CMSIS_NN_SUCCESS) {
			return std::nullopt;
		}
	}

	{ /* FC W3 -> 1 */
		const cmsis_nn_fc_params p = {
			.input_offset = -kC3Zp,
			.filter_offset = 0,
			.output_offset = kOutZp,
			.activation = {-128, 127},
		};
		const cmsis_nn_per_tensor_quant_params q = {kFcMult, kFcShift};
		const cmsis_nn_dims in_d = {1, 1, 1, kW3};
		const cmsis_nn_dims f_d = {kW3, 1, 1, 1}; /* n = accumulation depth, c = outputs */
		const cmsis_nn_dims b_d = {1, 1, 1, 1};
		const cmsis_nn_dims o_d = {1, 1, 1, 1};
		cmsis_nn_context ctx = {scratch_.data(), arm_fully_connected_s8_get_buffer_size(&f_d)};

		if (arm_fully_connected_s8(&ctx, &p, &q, &in_d, gap.data(), &f_d, fc_w, &b_d, fc_b,
					   &o_d, &logit_q) != ARM_CMSIS_NN_SUCCESS) {
			return std::nullopt;
		}
	}

	return 1.0f / (1.0f + std::exp(-static_cast<float>(logit_q - kOutZp) * kOutScale));
}

EventDetector::Change EventDetector::update(float p, uint32_t now_ms)
{
	hist_[idx_] = p;
	idx_ = (idx_ + 1) % cfg_.smooth;
	smoothed_ = std::accumulate(hist_.begin(), hist_.begin() + cfg_.smooth, 0.0f) / cfg_.smooth;

	Change change = Change::None;

	if (!on_) {
		hits_ = smoothed_ >= cfg_.thr_on ? hits_ + 1 : 0;
		if (hits_ == 1) {
			on_since_ = now_ms;
		}
		if (hits_ >= cfg_.confirm) {
			on_ = true;
			change = Change::Started;
		}
	} else if (smoothed_ < cfg_.thr_off) {
		const uint32_t dur = now_ms - on_since_;

		on_ = false;
		hits_ = 0;
		if (dur >= cfg_.min_event_ms) {
			events_++;
			wheeze_ms_ += dur;
			last_dur_ = dur;
			change = Change::Ended;
		}
	}
	return change;
}

} // namespace wheeze
