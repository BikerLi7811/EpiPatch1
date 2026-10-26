// Replays simulated 1 Hz vitals through the firmware's own HistamineMonitor and
// AnaphylaxisEngine (Kconfig defaults), so the trained model is compared with
// exactly what the device would decide.
//
// in : int32 n, t, f, then float32 [n][t][f] in sim.py's COLS order
// out: uint8 [n][t], bit 0 trigger, 1 severe, 2 exposure, 3 respiratory, 4 cardiovascular
#include <cstdio>
#include <vector>

#include "algo/anaphylaxis.hpp"
#include "algo/histamine.hpp"

using namespace dev;

enum Col { HrValid, Hr, RrValid, Rr, Apnea, PpgValid, Pi, Spo2, Wheezing, Activity,
	   Collapsed, Fall, HistOk, HistNm, NCols };

int main(int argc, char **argv)
{
	if (argc != 3) {
		std::fprintf(stderr, "usage: replay <vitals.bin> <out.bin>\n");
		return 2;
	}
	FILE *in = std::fopen(argv[1], "rb");
	int hdr[3];
	if (!in || std::fread(hdr, sizeof hdr, 1, in) != 1 || hdr[2] != NCols) {
		std::fprintf(stderr, "bad input\n");
		return 1;
	}
	const int n = hdr[0], t_len = hdr[1];
	std::vector<float> rec(size_t(t_len) * NCols);
	std::vector<uint8_t> out(size_t(n) * t_len);

	// the simulator gives concentrations; the AFE adds its zero offset
	const float kOffsetMv = algo::HistamineMonitor::Config{}.offset_mv;

	for (int r = 0; r < n; ++r) {
		if (std::fread(rec.data(), sizeof(float), rec.size(), in) != rec.size()) {
			return 1;
		}
		algo::HistamineMonitor hist;
		hist.reset({});
		algo::AnaphylaxisEngine engine;
		engine.reset({});
		uint32_t last_wheeze = 0, wheeze_events = 0;

		for (int s = 0; s < t_len; ++s) {
			const float *x = &rec[size_t(s) * NCols];
			const uint32_t now = uint32_t(s + 1) * 1000u;
			if (s % 10 == 0) {
				hist.sample(x[HistOk] > 0.5f ? kOffsetMv + x[HistNm] : -1.0f);
			}
			if (x[Wheezing] > 0.5f) {
				last_wheeze = now;
				++wheeze_events;
			}
			Vitals v{};
			v.hr_valid = x[HrValid] > 0.5f;
			v.hr_bpm = x[Hr];
			v.rr_valid = x[RrValid] > 0.5f;
			v.rr_bpm = x[Rr];
			v.apnea = x[Apnea] > 0.5f;
			v.ppg_valid = x[PpgValid] > 0.5f;
			v.pi_pct = x[Pi];
			v.pulse_bpm = x[Hr];
			v.spo2_est = x[Spo2];
			v.wheezing = x[Wheezing] > 0.5f;
			v.last_wheeze_ms = last_wheeze;
			v.wheeze_events = wheeze_events;
			v.activity_g = x[Activity];
			v.collapsed = x[Collapsed] > 0.5f;
			v.fall = x[Fall] > 0.5f;
			v.hist_valid = hist.valid();
			v.hist_elevated = hist.elevated();
			v.hist_nm = hist.nm();
			v.hist_baseline_nm = hist.baseline_nm();

			const auto a = engine.update(v, now);
			out[size_t(r) * t_len + s] = uint8_t(a.trigger | a.severe << 1 | a.exposure << 2 |
							     a.respiratory << 3 | a.cardiovascular << 4);
		}
	}
	FILE *o = std::fopen(argv[2], "wb");
	std::fwrite(out.data(), 1, out.size(), o);
	std::fclose(o);
	std::printf("replayed %d recordings\n", n);
	return 0;
}
