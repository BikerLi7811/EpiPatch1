#include "burst.hpp"

#include <algorithm>
#include <array>
#include <cmath>

// Pure C++ (no Zephyr), so the host tests use it too.
#if defined(CONFIG_APP_BURST_S)
#define BURST_S CONFIG_APP_BURST_S
#else
#define BURST_S 60
#endif

namespace dev::burst {
namespace {

constexpr std::size_t kSeconds = BURST_S + 2;           // a little slack for tick jitter
constexpr std::size_t kPpg = (BURST_S + 2) * 25 + 32;   // 25 Hz + one FIFO
constexpr std::size_t kImu = (BURST_S + 2) * 13 + 64;   // 12.5 Hz + one FIFO

struct Second {
	uint32_t t_ms;
	uint32_t ppg_end; // PPG samples received by this second
	Vitals v;
};

// accelerometer samples stored as mg to halve the RAM
struct Mg3 {
	int16_t x, y, z;
};

constinit std::array<Second, kSeconds> seconds{};
constinit std::array<uint32_t, kPpg> red{}, ir{};
constinit std::array<Mg3, kImu> chest{}, thigh{};
constinit std::size_t n_sec{}, n_ppg{}, n_chest{}, n_thigh{};

int16_t to_mg(float g) { return static_cast<int16_t>(std::clamp(std::lround(g * 1000.0f), -32768L, 32767L)); }

} // namespace

void begin(uint32_t)
{
	n_sec = n_ppg = n_chest = n_thigh = 0;
}

void add_ppg(std::span<const uint32_t> r, std::span<const uint32_t> i)
{
	const std::size_t n = std::min({r.size(), i.size(), kPpg - n_ppg});
	std::copy_n(r.begin(), n, red.begin() + n_ppg);
	std::copy_n(i.begin(), n, ir.begin() + n_ppg);
	n_ppg += n;
}

void add_imu(bool is_chest, std::span<const algo::Vec3> s)
{
	auto &buf = is_chest ? chest : thigh;
	auto &cnt = is_chest ? n_chest : n_thigh;
	for (const auto &a : s) {
		if (cnt == kImu) {
			break;
		}
		buf[cnt++] = {to_mg(a.x), to_mg(a.y), to_mg(a.z)};
	}
}

void add_second(const Vitals &v, uint32_t now_ms)
{
	if (n_sec < kSeconds) {
		seconds[n_sec++] = {now_ms, static_cast<uint32_t>(n_ppg), v};
	}
}

Info info() { return {n_sec, n_ppg, n_chest, n_thigh}; }

algo::Assessment analyse(algo::AnaphylaxisEngine &engine, algo::PpgProcessor &ppg_proc)
{
	ppg_proc.reset({});
	algo::Assessment out{};
	uint16_t findings = 0;
	bool trigger = false, severe = false;
	std::size_t fed = 0;
	for (std::size_t i = 0; i < n_sec; ++i) {
		Second &s = seconds[i];
		const std::size_t end = std::min<std::size_t>(s.ppg_end, n_ppg);
		if (end > fed) {
			ppg_proc.process(std::span(red).subspan(fed, end - fed), std::span(ir).subspan(fed, end - fed));
			fed = end;
		}
		s.v.ppg_valid = ppg_proc.valid();
		s.v.pi_pct = ppg_proc.pi_pct();
		s.v.pulse_bpm = ppg_proc.pulse_bpm();
		s.v.spo2_est = ppg_proc.spo2_est();
		out = engine.update(s.v, s.t_ms);
		findings |= out.findings;
		trigger |= out.trigger;
		severe |= out.severe;
	}
	out.findings = findings;
	out.trigger = trigger;
	out.severe = severe;
	return out;
}

} // namespace dev::burst
