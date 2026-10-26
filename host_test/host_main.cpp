/*
 * Runs the unmodified firmware C++ DSP + NN + power logic on a PC against Python test vectors.
 * usage: host_test <vectors dir>
 */
#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include "duty_cycler.hpp"
#include "feature_extractor.hpp"
#include "wheeze_net.hpp"

using namespace wheeze;

template <typename T> static std::vector<T> load(const std::string &dir, const std::string &name)
{
	std::ifstream f(dir + "/" + name, std::ios::binary | std::ios::ate);
	if (!f) {
		return {};
	}
	std::vector<T> v(static_cast<std::size_t>(f.tellg()) / sizeof(T));
	f.seekg(0);
	f.read(reinterpret_cast<char *>(v.data()), v.size() * sizeof(T));
	return v;
}

/* Streams `in` through the extractor; compares every frame (and its tonality) with Python. */
template <typename T>
static bool check_frames(const std::string &what, const std::vector<float> &ref,
			 const std::vector<T> &in, const dsp::MicRate &rate,
			 const std::vector<float> &ref_ton = {})
{
	static FeatureExtractor fx;
	constexpr bool pcm = sizeof(T) == 2;
	const std::size_t chunk = pcm ? FeatureExtractor::kDecimBlock : dsp::kHop;
	const std::size_t n_ref = ref.size() / dsp::kNumMels;
	std::size_t got = 0;
	double max_err = 0, max_ton_err = 0;

	fx.init(rate, 0.0f);
	for (std::size_t i = 0; i + chunk <= in.size(); i += chunk) {
		const std::span<const T> s(in.data() + i, chunk);
		uint32_t nf;

		if constexpr (pcm) {
			nf = fx.push_pcm(s);
		} else {
			nf = fx.push_4k(s);
		}
		for (uint32_t age = nf; age > 0 && got < n_ref; age--, got++) {
			const auto row = fx.frame(age);

			for (std::size_t m = 0; m < dsp::kNumMels; m++) {
				max_err = std::fmax(max_err, std::fabs(row[m] - ref[got * dsp::kNumMels + m]));
			}
			if (got < ref_ton.size()) {
				max_ton_err = std::fmax(max_ton_err,
							std::fabs(fx.frame_tonality(age) - ref_ton[got]));
			}
		}
	}
	std::printf("%-14s: %zu/%zu frames, max |logmel_C++ - py| = %.2e, |tonality| = %.2e\n",
		    what.c_str(), got, n_ref, max_err, max_ton_err);
	return got == n_ref && max_err < 1e-3 && max_ton_err < 1e-3;
}

/* EventDetector at the exported stride: a 1 s burst must give exactly one ~1 s event. */
static bool check_detector()
{
	const uint32_t period = model::kInferStride * 16;
	EventDetector det(EventDetector::Config::for_period_ms(period));
	int started = 0, ended = 0;

	for (uint32_t t = 0; t < 8000; t += period) {
		const float p = (t >= 2000 && t < 3000) ? 0.99f : 0.01f;

		switch (det.update(p, t)) {
		case EventDetector::Change::Started: started++; break;
		case EventDetector::Change::Ended: ended++; break;
		default: break;
		}
	}
	std::printf("EventDetector : %u ms period (smooth %u, confirm %u): %d start, %d end, "
		    "%u events, last %u ms\n",
		    period, det.config().smooth, det.config().confirm, started, ended, det.events(),
		    det.last_event_ms());
	return started == 1 && ended == 1 && det.events() == 1;
}

/* DutyCycler: samples while quiet, goes continuous on a candidate, falls back after hold. */
static bool check_duty_cycler()
{
	DutyCycler dc;
	bool ok = true;

	dc.on_listen_start(0);
	ok &= !dc.should_stop(7999) && dc.should_stop(8000);  /* 8 s look, then off */
	dc.on_listen_start(30000);
	dc.on_candidate(31000);                                /* wheeze-ish sound */
	ok &= dc.continuous(31000) && !dc.should_stop(60000);  /* stays on */
	ok &= !dc.continuous(91000) && dc.should_stop(91000);  /* 60 s quiet -> sampling */
	std::printf("DutyCycler    : %s\n", ok ? "ok" : "FAIL");
	return ok;
}

int main(int argc, char **argv)
{
	const std::string dir = argc > 1 ? argv[1] : "vectors";
	bool ok = true;

	{
		const auto a = load<float>(dir, "audio_4k.f32");
		const auto r = load<float>(dir, "audio_4k_logmel.f32");

		if (!a.empty() && !r.empty()) {
			ok &= check_frames("4 kHz stream", r, a, dsp::mic_rates[0]);
		}
	}
	for (const auto &rate : dsp::mic_rates) {
		const std::string tag = "pcm_" + std::to_string(rate.fs / 1000) + "k";
		const auto pcm = load<int16_t>(dir, tag + ".s16");
		const auto r = load<float>(dir, tag + "_logmel.f32");
		const auto ton = load<float>(dir, tag + "_tonality.f32");
		const std::string what = std::to_string(rate.fs) + " Hz PCM";

		if (pcm.empty() || r.empty() || ton.empty()) {
			std::printf("%s: missing vectors\n", what.c_str());
			ok = false;
			continue;
		}
		ok &= check_frames(what, r, pcm, rate, ton);
	}
	ok &= check_detector();
	ok &= check_duty_cycler();

	/* CNN over every test window */
	if (auto w = load<float>(dir, "test_windows.f32"); !w.empty()) {
		static WheezeNet net;
		const std::size_t n = w.size() / kWindowSize;
		std::vector<float> probs(n);
		std::array<int8_t, kWindowSize> q;

		for (std::size_t i = 0; i < n; i++) {
			quantize(std::span(w).subspan(i * kWindowSize, kWindowSize), q, model::kInScale,
				 model::kInZp);
			const auto p = net.run(q);
			if (!p) {
				std::printf("nn error at %zu\n", i);
				return 2;
			}
			probs[i] = *p;
		}
		std::ofstream(dir + "/c_probs.f32", std::ios::binary)
			.write(reinterpret_cast<const char *>(probs.data()), n * sizeof(float));
		std::printf("CNN           : %zu windows -> c_probs.f32\n", n);
	}
	std::printf("%s\n", ok ? "ALL CHECKS PASSED" : "SOME CHECKS FAILED");
	return ok ? 0 : 1;
}
