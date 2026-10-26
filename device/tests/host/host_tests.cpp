// Host (PC) tests of the device's pure algorithm code: the same sources that
// run on the nRF54L15, against CMSIS-DSP compiled for the PC.
//   usage: host_tests <dir from gen_vectors.py>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

#include "algo/anaphylaxis.hpp"
#include "algo/body_motion.hpp"
#include "algo/deploy_controller.hpp"
#include "algo/ecg_qrs.hpp"
#include "algo/histamine.hpp"
#include "algo/ppg.hpp"
#include "algo/resp_rate.hpp"
#include "algo/sampling_schedule.hpp"
#include "algo/amperometry.hpp"
#include "algo/tap_counter.hpp"
#include "algo/thigh_motion.hpp"
#include "algo/thigh_view.hpp"
#include "algo/vital_trigger.hpp"
#include "burst.hpp"

using namespace dev;
using namespace dev::algo;

namespace {

int g_fail = 0;
int g_pass = 0;

#define CHECK(cond, ...)                                                                            \
	do {                                                                                        \
		if (cond) {                                                                         \
			++g_pass;                                                                   \
		} else {                                                                            \
			++g_fail;                                                                   \
			std::printf("  FAIL %s:%d: ", __FILE__, __LINE__);                          \
			std::printf(__VA_ARGS__);                                                   \
			std::printf("\n");                                                          \
		}                                                                                   \
	} while (0)

std::string g_dir;

template <typename T>
std::vector<T> load(const char *name)
{
	std::ifstream f(g_dir + "/" + name, std::ios::binary | std::ios::ate);
	if (!f) {
		std::printf("missing %s\n", name);
		std::exit(2);
	}
	const auto bytes = static_cast<std::size_t>(f.tellg());
	std::vector<T> v(bytes / sizeof(T));
	f.seekg(0);
	f.read(reinterpret_cast<char *>(v.data()), static_cast<std::streamsize>(bytes));
	return v;
}

// ---------------------------------------------------------------------------

template <std::size_t N>
void filter_case(const char *name, const std::array<float, 5 * N> &c)
{
	auto x = load<float>("filt_in.f32");
	auto ref = load<float>((std::string("filt_") + name + ".f32").c_str());
	Biquad<N> bq;
	bq.init(c);
	std::vector<float> y(x.size());
	bq.process(x, y);
	float err = 0, mag = 0;
	for (std::size_t i = 0; i < x.size(); ++i) {
		err = std::max(err, std::fabs(y[i] - ref[i]));
		mag = std::max(mag, std::fabs(ref[i]));
	}
	std::printf("  %-14s max |CMSIS - scipy| = %.2e (peak %.2f)\n", name, err, mag);
	CHECK(err < 1e-4f * mag + 1e-6f, "%s filter mismatch %g", name, err);
}

void test_filters()
{
	std::printf("filters\n");
	filter_case<coeffs::kEcgBandpassStages>("kEcgBandpass", coeffs::kEcgBandpass);
	filter_case<coeffs::kRespBandpassStages>("kRespBandpass", coeffs::kRespBandpass);
	filter_case<coeffs::kPpgBandpassStages>("kPpgBandpass", coeffs::kPpgBandpass);
}

void test_ecg()
{
	std::printf("ECG heart rate (synthetic, noise + wander + mains + EMG bursts)\n");
	const auto x = load<float>("ecg_uv.f32");
	const auto beats = load<float>("ecg_beats_s.f32");
	QrsDetector q;
	q.reset();
	struct Cp {
		float t, bpm;
	};
	const Cp cps[] = {{55, 60}, {115, 100}, {175, 150}, {235, 72}};
	std::size_t k = 0;
	for (std::size_t i = 0; i + 25 <= x.size(); i += 25) {
		q.process(std::span(x).subspan(i, 25), false);
		const float t = (i + 25) / 125.0f;
		if (k < std::size(cps) && t >= cps[k].t) {
			std::printf("  t=%3.0fs  HR %6.1f (truth %3.0f) valid %d\n", t, q.hr_bpm(), cps[k].bpm,
				    q.hr_valid());
			CHECK(q.hr_valid() && std::fabs(q.hr_bpm() - cps[k].bpm) <= 3.0f, "HR at %.0f s", t);
			++k;
		}
	}
	const int miss = static_cast<int>(beats.size()) - static_cast<int>(q.beats());
	std::printf("  beats detected %u / %zu\n", q.beats(), beats.size());
	CHECK(std::abs(miss) <= 4, "beat count off by %d", miss);

	q.process(std::span(x).subspan(0, 25), true);
	CHECK(!q.hr_valid(), "HR must be invalid with leads off");
}

void test_resp()
{
	std::printf("Bio-Z breathing rate / apnea (synthetic)\n");
	const auto x = load<float>("bioz.f32");
	RespRate r;
	r.reset({.apnea_ms = 20000});
	struct Cp {
		float t;
		float bpm; // 0 = expect apnea
	};
	const Cp cps[] = {{85, 14}, {175, 32}, {215, 0}, {235, 0}, {320, 12}};
	std::size_t k = 0;
	for (std::size_t i = 0; i + 25 <= x.size(); i += 25) {
		r.process(std::span(x).subspan(i, 25), false);
		const float t = (i + 25) / 125.0f;
		if (k < std::size(cps) && t >= cps[k].t) {
			std::printf("  t=%3.0fs  RR %5.1f valid %d apnea %d (truth %s%.0f)\n", t, r.rate_bpm(),
				    r.valid(), r.apnea(), cps[k].bpm ? "" : "apnea ", cps[k].bpm);
			if (cps[k].bpm > 0) {
				CHECK(r.valid() && !r.apnea() && std::fabs(r.rate_bpm() - cps[k].bpm) <= 2.0f,
				      "RR at %.0f s", t);
			} else {
				CHECK(r.apnea() && !r.valid(), "apnea at %.0f s", t);
			}
			++k;
		}
	}
}

void test_ppg()
{
	std::printf("PPG perfusion index / pulse / SpO2 estimate (synthetic)\n");
	const auto red = load<uint32_t>("ppg_red.u32");
	const auto ir = load<uint32_t>("ppg_ir.u32");
	PpgProcessor p;
	p.reset({});
	struct Cp {
		float t, pi, bpm;
	};
	const Cp cps[] = {{115, 2.0f, 75}, {235, 0.6f, 110}};
	std::size_t k = 0;
	for (std::size_t i = 0; i + 25 <= ir.size(); i += 25) {
		p.process(std::span(red).subspan(i, 25), std::span(ir).subspan(i, 25));
		const float t = (i + 25) / 25.0f;
		if (k < std::size(cps) && t >= cps[k].t) {
			std::printf("  t=%3.0fs  PI %.2f%% (truth %.1f)  pulse %.1f (truth %.0f)  SpO2~ %.1f\n", t,
				    p.pi_pct(), cps[k].pi, p.pulse_bpm(), cps[k].bpm, p.spo2_est());
			CHECK(p.valid(), "PPG valid at %.0f s", t);
			CHECK(std::fabs(p.pi_pct() - cps[k].pi) <= 0.15f * cps[k].pi, "PI at %.0f s", t);
			CHECK(std::fabs(p.pulse_bpm() - cps[k].bpm) <= 3.0f, "pulse at %.0f s", t);
			CHECK(std::fabs(p.spo2_est() - 95.0f) <= 2.5f, "SpO2 estimate at %.0f s", t);
			++k;
		}
	}
	// sensor off the skin
	std::vector<uint32_t> off(100, 500);
	p.process(off, off);
	CHECK(!p.contact() && !p.valid(), "no contact must be invalid");
}

// ---------------------------------------------------------------------------
// IMU scenarios: 12.5 Hz streams built from a body-angle timeline.

struct ImuSim {
	BodyMotion m;
	uint32_t t = 0;          // ms
	unsigned seed = 1;

	float noise(float a)
	{
		seed = seed * 1103515245u + 12345u;
		return a * ((seed >> 8) % 2001 / 1000.0f - 1.0f);
	}

	// chest angle from vertical (deg), thigh angle (deg), dynamic noise (g), seconds
	void run(float chest0, float chest1, float thigh0, float thigh1, float act, float secs)
	{
		const uint32_t n = static_cast<uint32_t>(secs * 12.5f);
		std::vector<Vec3> c, th;
		for (uint32_t i = 0; i < n; ++i) {
			const float f = n > 1 ? static_cast<float>(i) / (n - 1) : 1.0f;
			const float a = (chest0 + (chest1 - chest0) * f) / 57.2958f;
			const float b = (thigh0 + (thigh1 - thigh0) * f) / 57.2958f;
			c.push_back({std::cos(a) + noise(act), noise(act), std::sin(a) + noise(act)});
			th.push_back({std::cos(b) + noise(act), noise(act), std::sin(b) + noise(act)});
			t += 80;
			if (t % 1000 < 80) {
				m.chest(c, t, 80);
				m.thigh(th, t, 80);
				c.clear();
				th.clear();
				m.update(t);
			}
		}
		if (!c.empty()) {
			m.chest(c, t, 80);
			m.thigh(th, t, 80);
			m.update(t);
		}
	}
};

void test_motion()
{
	std::printf("IMU posture / fall / collapse scenarios\n");
	{
		ImuSim s;
		s.m.reset({});
		s.run(0, 0, 0, 0, 0.01f, 10);
		CHECK(s.m.posture() == Posture::Standing, "standing");
		s.run(0, 0, 90, 90, 0.01f, 10);
		CHECK(s.m.posture() == Posture::Sitting, "sitting");
	}
	{
		ImuSim s;
		s.m.reset({});
		s.run(0, 0, 0, 0, 0.15f, 20);          // walking
		s.run(0, 90, 0, 90, 0.4f, 1.0f);       // goes down in 1 s
		s.m.impact(s.t);                       // hits the floor
		const uint32_t t_impact = s.t;
		s.run(90, 90, 90, 90, 0.01f, 5);
		CHECK(!s.m.fall(), "fall not confirmed too early");
		s.run(90, 90, 90, 90, 0.01f, 8);
		CHECK(s.m.fall(), "fall confirmed ~10 s after impact (t+%u ms)", s.t - t_impact);
		CHECK(s.m.posture() == Posture::Lying, "lying after fall");
		s.run(90, 90, 90, 90, 0.01f, 10);
		CHECK(s.m.collapsed(), "collapsed after 15 s motionless");
		std::printf("  fall + collapse detected, posture lying\n");
		s.run(90, 0, 90, 0, 0.2f, 2);
		s.run(0, 0, 0, 0, 0.05f, 7);
		CHECK(!s.m.fall() && !s.m.collapsed(), "cleared after standing up");
	}
	{
		ImuSim s;
		s.m.reset({});
		s.run(0, 0, 0, 0, 0.01f, 10);
		s.run(0, 90, 0, 90, 0.05f, 6);         // lies down slowly
		s.run(90, 90, 90, 90, 0.01f, 40);
		CHECK(!s.m.collapsed() && !s.m.fall(), "slow lie-down is not a collapse");
	}
	{
		ImuSim s;
		s.m.reset({});
		s.run(0, 0, 0, 0, 0.1f, 10);
		s.m.impact(s.t);                       // bumps into something, stays up
		s.run(0, 0, 0, 0, 0.1f, 20);
		CHECK(!s.m.fall(), "impact while upright is not a fall");
	}
}

void test_taps()
{
	std::printf("tap gesture\n");
	TapCounter tc;
	tc.reset({});
	CHECK(!tc.on_tap(1000) && !tc.on_tap(1300) && tc.on_tap(1600), "3 taps in 0.6 s");
	CHECK(!tc.on_tap(5000) && !tc.on_tap(5300), "2 taps only");
	CHECK(!tc.on_tap(8000), "the 2 old taps expired");
	CHECK(!tc.on_tap(10000) && !tc.on_tap(11000) && !tc.on_tap(12000), "3 taps over 2 s");
	CHECK(!tc.on_tap(20000) && !tc.on_tap(20030) && !tc.on_tap(20300), "ringing tap not double-counted");
	CHECK(tc.on_tap(20600), "...third real tap completes");
}

void test_histamine()
{
	std::printf("histamine baseline / elevation\n");
	HistamineMonitor h;
	h.reset({.offset_mv = 0.0f, .warmup_ms = 60000});
	for (int i = 0; i < 60; ++i) {
		h.sample(100.0f);
	}
	CHECK(h.valid() && !h.elevated() && std::fabs(h.baseline_nm() - 100.0f) < 1.0f, "baseline 100 nM");
	h.sample(200.0f);
	CHECK(!h.elevated(), "one high sample is not enough");
	h.sample(200.0f);
	CHECK(h.elevated(), "elevated after 2 high samples");
	CHECK(h.baseline_nm() < 101.0f, "baseline frozen while elevated");
	h.sample(102.0f);
	h.sample(102.0f);
	CHECK(!h.elevated(), "clears when back to baseline");
	h.sample(0.0f);
	CHECK(!h.valid() && !h.elevated(), "open sensor is invalid, never elevated");

	HistamineMonitor d; // Kconfig-equivalent defaults
	d.reset({});
	d.sample(500.0f);
	CHECK(d.valid() && d.nm() < 0.01f, "default offset: zero histamine is a valid reading");
	d.sample(510.0f);
	CHECK(d.valid() && std::fabs(d.nm() - 10.0f) < 0.01f, "default offset: 10 nM reads as 10 nM");
}

// ---------------------------------------------------------------------------
// decision engine + deploy state machine

Vitals calm()
{
	Vitals v{};
	v.hr_valid = true;
	v.hr_bpm = 70;
	v.rr_valid = true;
	v.rr_bpm = 14;
	v.ppg_valid = true;
	v.pi_pct = 2.0f;
	v.pulse_bpm = 70;
	v.posture = Posture::Sitting;
	v.activity_g = 0.01f;
	v.hist_valid = true;
	v.hist_nm = 100;
	return v;
}

struct Sim {
	AnaphylaxisEngine e;
	DeployController d;
	uint32_t t = 1000;
	int alerts = 0, fires = 0, cancels = 0;
	Assessment last{};

	explicit Sim(uint8_t done = 0)
	{
		e.reset({});
		d.reset({}, done);
	}
	void step(const Vitals &v, int secs = 1)
	{
		for (int i = 0; i < secs; ++i) {
			last = e.update(v, t);
			const auto a = d.update(last, t);
			alerts += a == DeployController::Action::StartAlert;
			fires += a == DeployController::Action::Fire;
			if (a == DeployController::Action::Fire) {
				d.motor_done(true, t + 3000);
			}
			t += 1000;
		}
	}
	void warm() { step(calm(), 300); }
};

void test_decision()
{
	std::printf("decision engine + deploy state machine\n");
	{
		Sim s;
		s.warm();
		CHECK(s.e.hr_baseline_valid() && std::fabs(s.e.hr_baseline() - 70) < 1, "HR baseline learned");
		Vitals v = calm();
		v.hist_elevated = true;
		v.wheezing = true;
		s.step(v, 2);
		CHECK(s.alerts == 0, "two seconds of wheeze are not yet a finding");
		s.step(v);
		CHECK(s.last.trigger && s.alerts == 1 && s.d.state() == DeployState::Alert,
		      "histamine + 3 s of wheeze -> alert");
		s.step(v, 19);
		CHECK(s.fires == 0, "no firing during the 20 s countdown");
		s.step(v, 1);
		CHECK(s.fires == 1, "fires when the countdown ends");
		CHECK(s.d.state() == DeployState::Deployed, "deployed");
		s.step(v, 900);
		CHECK(s.fires == 1 && s.alerts == 1, "single-dose device never re-fires");
	}
	{
		Sim s;
		s.warm();
		Vitals v = calm();
		v.hist_elevated = true;
		v.wheezing = true;
		s.step(v, 5);
		CHECK(s.d.cancel(s.t) == DeployController::Action::Cancelled, "triple tap cancels");
		s.step(v, 120);
		CHECK(s.fires == 0 && s.alerts == 1, "cooldown suppresses the same trigger");
		v.collapsed = true; // now on the floor, still wheezing
		s.step(v, 1);
		CHECK(s.last.severe && s.alerts == 2, "collapse + wheeze re-alerts despite cooldown");
		s.step(v, 25);
		CHECK(s.fires == 1, "...and fires if not cancelled again");
	}
	{
		Sim s;
		s.warm();
		Vitals v = calm();
		v.activity_g = 0.3f; // running
		v.hr_bpm = 160;
		v.rr_bpm = 36;
		v.hist_elevated = true;
		s.step(v, 120);
		CHECK(s.alerts == 0, "exercise HR/RR are not findings (histamine alone never triggers)");
	}
	{
		Sim s;
		s.warm();
		Vitals v = calm();
		v.hist_elevated = true;
		v.hr_bpm = 135;
		s.step(v, 59);
		CHECK(s.alerts == 0, "tachycardia needs 60 s");
		s.step(v, 2);
		CHECK(s.alerts == 1 && (s.last.findings & kTachycardia), "histamine + 60 s tachycardia -> alert");
	}
	{
		Sim s;
		s.warm();
		Vitals v = calm();
		v.hist_elevated = true;
		v.pi_pct = 0.8f;
		s.step(v, 31);
		CHECK(s.alerts == 1 && (s.last.findings & kPerfusionDrop), "histamine + perfusion drop -> alert");
	}
	{
		Sim s;
		s.warm();
		Vitals v = calm();
		v.hist_elevated = true;
		v.hr_valid = false; // leads off: garbage HR must not count
		v.hr_bpm = 220;
		v.ppg_valid = false;
		v.pi_pct = 0.0f;
		s.step(v, 120);
		CHECK(s.alerts == 0, "invalid sensors never contribute findings");
	}
	{
		Sim s;
		s.warm();
		Vitals v = calm();
		v.wheezing = true;
		v.fall = true;
		s.step(v, 3);
		CHECK(s.alerts == 1, "wheeze + fall (no histamine) -> alert (rule B)");
	}
	{
		Sim s(1); // rebooted after a deployment
		s.warm();
		Vitals v = calm();
		v.hist_elevated = true;
		v.wheezing = true;
		s.step(v, 60);
		CHECK(s.d.state() == DeployState::Deployed && s.alerts == 0, "latched after reboot");
	}
	{
		Sim s;
		s.warm();
		Vitals v = calm();
		v.hist_elevated = true;
		v.wheeze_events = 1; // one stray detection, then quiet
		s.step(v);
		s.step(v, 120);
		CHECK(s.alerts == 0 && !(s.last.findings & kWheeze), "histamine + one stray wheeze -> no alert");
		v.wheeze_events = 2;
		s.step(v);
		s.step(v, 60);
		v.wheeze_events = 3;
		s.step(v);
		CHECK(s.alerts == 0, "detections more than 60 s apart do not add up");
		v.wheeze_events = 4;
		s.step(v, 10);
		v.wheeze_events = 5;
		s.step(v);
		CHECK(s.alerts == 1, "three detections within 60 s -> wheeze finding");
	}
	{
		Sim s;
		s.warm();
		Vitals v = calm();
		v.hist_elevated = true;
		s.step(v, 300); // histamine peaks with no symptoms yet
		v.hist_elevated = false;
		s.step(v, 1500); // back to baseline 25 min before the circulation goes
		CHECK(s.alerts == 0 && (s.last.findings & kHistamine), "exposure held after histamine clears");
		v.hr_bpm = 135;
		s.step(v, 61);
		CHECK(s.alerts == 1, "late tachycardia after cleared histamine -> alert (rule A)");
	}
	{
		Sim s;
		s.warm();
		Vitals v = calm();
		v.hist_elevated = true;
		s.step(v, 60);
		v.hist_elevated = false;
		s.step(v, 3601);
		CHECK(!(s.last.findings & kHistamine), "exposure expires after an hour");
		v.hr_bpm = 135;
		s.step(v, 61);
		CHECK(s.alerts == 0, "tachycardia after exposure expired -> no alert");
	}
	{
		// histamine read minutes apart (idle schedule), then every 10 s (burst)
		HistamineMonitor h;
		h.reset({.offset_mv = 0.0f, .warmup_ms = 300000});
		uint32_t t = 0;
		for (int i = 0; i < 4; ++i, t += 120000) {
			h.sample(100.0f, t);
		}
		CHECK(h.valid() && !h.elevated() && std::fabs(h.baseline_nm() - 100.0f) < 1.0f,
		      "2-minute histamine readings: warm, baseline 100 nM");
		h.sample(200.0f, t);
		h.sample(200.0f, t + 10000);
		CHECK(h.elevated(), "two high readings 10 s apart -> elevated");
	}
}

// ---------------------------------------------------------------------------
// ECG/Bio-Z triggers, sampling schedule, burst analysis

VitalTrigger::Input afe(float hr, float rr, bool apnea = false)
{
	return {.hr_valid = true, .hr_bpm = hr, .rr_valid = true, .rr_bpm = rr, .apnea = apnea, .leads_off = false};
}

void test_trigger()
{
	std::printf("ECG/Bio-Z triggers\n");
	VitalTrigger tr;
	tr.reset({});
	uint32_t t = 0;
	uint8_t got = 0;
	for (int i = 0; i < 30; ++i, t += 1000) {
		got |= tr.update(afe(100, 16), t);
	}
	CHECK(got == 0, "normal HR/RR: no trigger");
	for (int i = 0; i < 5; ++i, t += 1000) {
		got |= tr.update(afe(115, 16), t);
	}
	CHECK(got == 0, "HR 115 for 5 readings (4 s): not yet");
	got = tr.update(afe(115, 16), t);
	t += 1000;
	CHECK(got == VitalTrigger::kHrHigh, "HR 115 held 5 s -> heart-rate trigger");
	got = 0;
	for (int i = 0; i < 60; ++i, t += 1000) {
		got |= tr.update(afe(118, 16), t);
	}
	CHECK(got == 0 && tr.heart_latched(), "fires once, then latched until re-armed");
	tr.rearm();
	CHECK(tr.update(afe(118, 16), t) == VitalTrigger::kHrHigh, "re-armed while still high -> fires again at once");
	t += 1000;
	tr.rearm();
	got = 0;
	for (int i = 0; i < 10; ++i, t += 1000) {
		got |= tr.update(afe(80, 16), t);
	}
	CHECK(got == 0, "re-armed after recovery -> quiet");

	got = 0;
	for (int i = 0; i < 6; ++i, t += 1000) {
		got |= tr.update(afe(80, 30), t);
	}
	CHECK(got == VitalTrigger::kRrHigh, "breathing 30/min -> breathing trigger");
	tr.reset({});
	got = 0;
	for (int i = 0; i < 6; ++i, t += 1000) {
		got |= tr.update(afe(36, 16), t);
	}
	CHECK(got == VitalTrigger::kHrLow, "HR 36 -> heart-rate (low) trigger");
	got = 0;
	for (int i = 0; i < 6; ++i, t += 1000) {
		got |= tr.update(afe(70, 0, true), t);
	}
	CHECK(got & VitalTrigger::kApnea, "apnea -> breathing trigger");
	tr.reset({});
	got = 0;
	for (int i = 0; i < 10; ++i, t += 1000) {
		VitalTrigger::Input in = afe(200, 50);
		in.leads_off = true;
		got |= tr.update(in, t);
	}
	CHECK(got == 0, "leads off -> no trigger from garbage rates");
}

void test_schedule()
{
	std::printf("sampling schedule\n");
	using A = SamplingSchedule::Action;
	using M = SamplingSchedule::Mode;
	SamplingSchedule s;
	s.reset({}, 0);
	CHECK(s.update(0, false) == A::StartSpot && s.awake(), "first read at boot");
	CHECK(s.update(14999, false) == A::None, "spot read lasts 15 s");
	CHECK(s.update(15000, false) == A::EndSpot && !s.awake(), "...then sensors off");
	CHECK(s.idle_wait_ms(15000) == 105000, "idle until 2 min after the last read");
	CHECK(s.update(119999, false) == A::None && s.update(120000, false) == A::StartSpot,
	      "next read every 2 min");
	CHECK(s.trigger(125000) == A::StartBurst && s.mode() == M::Burst, "trigger during a spot read -> burst");
	CHECK(s.trigger(126000) == A::None, "second trigger during a burst is absorbed");
	CHECK(s.update(184999, false) == A::None && s.update(185000, false) == A::EndBurst,
	      "burst records for 1 min, then analyse");
	CHECK(s.update(304999, false) == A::None && s.update(305000, false) == A::StartSpot,
	      "burst counts as a read: next spot 2 min after it");
	CHECK(s.update(306000, true) == A::StartActive && s.mode() == M::Active, "alert -> everything on");
	CHECK(s.trigger(307000) == A::None, "trigger during an alert is absorbed");
	CHECK(s.update(400000, true) == A::None, "stays on while alerting");
	CHECK(s.update(401000, false) == A::EndActive && !s.awake(), "alert over -> idle");
	CHECK(s.update(520999, false) == A::None && s.update(521000, false) == A::StartSpot,
	      "next spot 2 min after the alert");
	CHECK(s.trigger(530000) == A::StartBurst, "trigger while idle-or-spot -> burst");
	CHECK(s.update(531000, true) == A::None && s.mode() == M::Burst,
	      "an alert cannot start mid-burst (nothing is decided until the minute is analysed)");
}

void test_burst()
{
	std::printf("burst recording + analysis\n");
	AnaphylaxisEngine e;
	e.reset({});
	PpgProcessor p;
	// warm baselines with calm, resting seconds
	uint32_t t = 1000;
	for (int i = 0; i < 300; ++i, t += 1000) {
		e.update(calm(), t);
	}
	const auto record = [&](bool reaction) {
		burst::begin(t);
		std::array<uint32_t, 25> red, ir;
		std::array<Vec3, 13> acc;
		acc.fill({0.0f, 0.0f, 1.0f});
		for (int s = 0; s < 60; ++s, t += 1000) {
			for (int k = 0; k < 25; ++k) {
				const double ph = 2 * 3.14159265358979 * 1.2 * (s * 25 + k) / 25.0;
				red[k] = static_cast<uint32_t>(100000 + 1000 * std::sin(ph));
				ir[k] = static_cast<uint32_t>(120000 + 1500 * std::sin(ph));
			}
			burst::add_ppg(red, ir);
			burst::add_imu(true, acc);
			Vitals v = calm();
			v.ppg_valid = false; // must come from the raw PPG, not from here
			if (reaction) {
				v.hist_elevated = true;
				v.wheezing = true;
			}
			burst::add_second(v, t);
		}
		return burst::analyse(e, p);
	};
	Assessment a = record(false);
	const burst::Info info = burst::info();
	CHECK(info.seconds == 60 && info.ppg_samples == 1500 && info.chest_samples == 780,
	      "a minute of samples kept (60 s, 1500 PPG, 780 IMU)");
	CHECK(!a.trigger, "calm minute -> no trigger");
	CHECK(p.valid(), "PPG analysed from the raw samples");
	a = record(true);
	CHECK(a.trigger && (a.findings & kWheeze) && (a.findings & kHistamine),
	      "histamine + wheeze minute -> trigger after analysis");
}

// ---------------------------------------------------------------------------
// two patches: thigh fall detection, the chest's view of the thigh, AD5941 maths

void test_thigh()
{
	std::printf("thigh patch: fall, chest-side fusion, AD5941 conversions\n");
	const Vec3 standing{1.0f, 0.0f, 0.0f}; // +X toward the hip
	const Vec3 lying{0.0f, 0.0f, 1.0f};
	std::array<Vec3, 13> up, flat;
	up.fill(standing);
	flat.fill(lying);
	const auto feed = [](ThighMotion &m, const std::array<Vec3, 13> &s, uint32_t &t, int seconds) {
		for (int i = 0; i < seconds; ++i, t += 1000) {
			m.samples(s, t, 80);
		}
	};
	{
		ThighMotion m;
		m.reset({});
		uint32_t t = 1000;
		feed(m, up, t, 10);
		m.impact(t);
		feed(m, flat, t, 15);
		CHECK(m.fall(), "impact, then thigh horizontal and still -> fall");
		feed(m, up, t, 3);
		CHECK(m.fall(), "fall holds until upright for 5 s");
		feed(m, up, t, 4);
		CHECK(!m.fall(), "upright 5 s -> fall cleared");
	}
	{
		ThighMotion m;
		m.reset({});
		uint32_t t = 1000;
		feed(m, up, t, 10);
		feed(m, flat, t, 30);
		CHECK(!m.fall(), "sitting / lying down without an impact is not a fall");
		ThighMotion n;
		n.reset({});
		t = 1000;
		feed(n, up, t, 10);
		n.impact(t);
		feed(n, up, t, 30);
		CHECK(!n.fall(), "a bump while standing is not a fall");
	}
	{
		// the chest merges the thigh's report into its own vital signs
		ThighView tv;
		tv.reset({.fresh_ms = 300000});
		Vitals v = calm();
		v.hist_valid = true;
		v.hist_elevated = true;
		tv.merge(v, 1000);
		CHECK(!v.hist_valid && !v.hist_elevated, "no thigh report yet -> no histamine on the chest");
		patch_msg::Report r{};
		r.type = patch_msg::Type::Report;
		r.flags = patch_msg::kHistValid | patch_msg::kHistElevated | patch_msg::kFall | patch_msg::kImuValid;
		r.hist_nm = 42.0f;
		r.activity_g = 0.3f;
		r.deploy_state = static_cast<uint8_t>(DeployState::Alert);
		tv.on_report(r, 10000);
		v = calm();
		tv.merge(v, 20000);
		CHECK(v.hist_valid && v.hist_elevated && v.hist_nm == 42.0f && v.fall && v.activity_g >= 0.3f,
		      "fresh report -> histamine, thigh fall and activity merged");
		CHECK(tv.alerting(20000), "thigh countdown running -> chest stays fully on");
		v = calm();
		tv.merge(v, 10000 + 300001);
		CHECK(!v.hist_valid && !v.fall && !tv.alerting(10000 + 300001),
		      "report older than 5 min -> thigh data treated as missing");

		AnaphylaxisEngine e;
		e.reset({});
		uint32_t t = 1000;
		for (int i = 0; i < 300; ++i, t += 1000) {
			e.update(calm(), t);
		}
		tv.on_report(r, t);
		Assessment a{};
		for (int i = 0; i < 3; ++i, t += 1000) {
			Vitals w = calm();
			w.wheezing = true;
			tv.merge(w, t);
			a = e.update(w, t);
		}
		CHECK(a.trigger && (a.findings & kHistamine) && (a.findings & kFall),
		      "thigh histamine + thigh fall + chest wheeze -> chest decides to trigger");
	}
	{
		using namespace amperometry;
		const DacCodes c = lpdac_codes(1100.0f, 500.0f); // ADI's example values
		CHECK(c.code6 == 26 && c.code12 == 2593, "LPDAC codes match ADI's example formula (26, 2593)");
		CHECK(std::fabs(bias_mv(c) - 499.5f) < 1.0f, "bias reproduces 500 mV within one LSB");
		CHECK(adc_volts(32768, 1) == 0.0f, "mid-scale ADC code = 0 V");
		const float na = current_na(32768 - 1000, 1, rtia_ohm(24));
		CHECK(na > 190.0f && na < 200.0f, "1000 codes below mid-scale across 196 k = +195 nA (oxidation)");
		CHECK(rtia_ohm(24) == 196e3f && rtia_ohm(26) == 512e3f, "RTIA table");
	}
}

} // namespace

int main(int argc, char **argv)
{
	if (argc < 2) {
		std::printf("usage: %s <vector dir>\n", argv[0]);
		return 2;
	}
	g_dir = argv[1];
	test_filters();
	test_ecg();
	test_resp();
	test_ppg();
	test_motion();
	test_taps();
	test_histamine();
	test_decision();
	test_trigger();
	test_schedule();
	test_burst();
	test_thigh();
	std::printf("\n%d checks passed, %d failed\n%s\n", g_pass, g_fail,
		    g_fail ? "SOME CHECKS FAILED" : "ALL CHECKS PASSED");
	return g_fail ? 1 : 0;
}
