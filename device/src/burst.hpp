// One-minute burst recording, started by an ECG/Bio-Z trigger.
//
// While it runs, every PPG and accelerometer sample is kept here, plus a
// snapshot of everything else each second (heart rate and breathing from the
// AFE, wheeze, posture/collapse/fall, histamine). The raw ECG/Bio-Z minute is
// kept where the ADS1292R is read (see afe::recording()). Nothing is decided
// during the minute; analyse() then replays it second by second through the
// PPG processor and the decision engine.
#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

#include "algo/anaphylaxis.hpp"
#include "algo/body_motion.hpp"
#include "algo/ppg.hpp"
#include "algo/vitals.hpp"

namespace dev::burst {

void begin(uint32_t now_ms);

// raw samples as they arrive
void add_ppg(std::span<const uint32_t> red, std::span<const uint32_t> ir);
void add_imu(bool chest, std::span<const algo::Vec3> samples);

// once a second; the PPG fields of v are ignored (they come from the raw PPG)
void add_second(const Vitals &v, uint32_t now_ms);

struct Info {
	std::size_t seconds;
	std::size_t ppg_samples;
	std::size_t chest_samples, thigh_samples;
};
Info info();

// Replays the minute through ppg_proc (reset first) and engine. Returns the
// last second's assessment with trigger/severe/findings OR-ed over the minute.
algo::Assessment analyse(algo::AnaphylaxisEngine &engine, algo::PpgProcessor &ppg_proc);

} // namespace dev::burst
