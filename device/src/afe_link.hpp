// The application core's view of the ADS1292R (ECG heart rate, Bio-Z
// breathing) and its two vital-sign triggers, wherever it is read:
//   CONFIG_APP_AFE_ON_FLPR=y  the FLPR coprocessor reads it (flpr/); this file
//                             talks to it over IPC
//   otherwise                 an afe thread on this core reads it
// Either way the trigger callback runs outside the main thread: post an event.
#pragma once

#include <cstdint>

#include "afe_msgs.hpp"
#include "algo/vital_trigger.hpp"

namespace dev::afe {

struct Reading {
	bool hr_valid;
	float hr_bpm;
	bool leads_off;
	bool rr_valid;
	float rr_bpm;
	bool apnea;
	uint32_t updated_ms; // uptime of the reading; 0 = none yet
};

// A completed raw minute: 6 bytes per sample (ECG, Bio-Z; 24-bit big-endian).
struct Recording {
	const uint8_t *data;
	uint32_t samples;
	uint32_t sps;
};

using TriggerFn = void (*)(uint8_t bits); // algo::VitalTrigger::Bits

int start(const algo::VitalTrigger::Config &trig, uint32_t apnea_ms, TriggerFn on_trigger);

// Off: triggers only. Status: a Reading every second. Record: also keep every
// ECG/Bio-Z sample of the next CONFIG_APP_BURST_S seconds.
void stream(afe_msg::StreamMode mode);

// The burst a trigger started has been analysed: let the triggers fire again.
void rearm();

Reading latest();

// Most recent completed recording; samples == 0 if none.
Recording recording();

} // namespace dev::afe
