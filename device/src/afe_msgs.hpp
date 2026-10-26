// Messages between the application core and the FLPR coprocessor that owns
// the ADS1292R (one IPC endpoint, "afe"). Both cores are little-endian 32-bit
// and these structs only hold naturally aligned scalars, so they are sent as
// they are.
//
//   app  -> flpr   Config   trigger thresholds and apnea time (sent at boot)
//                  Stream   Off: triggers only / Status: one Status a second /
//                           Record: Status a second + keep every ECG and
//                           Bio-Z sample of the next minute
//                  Rearm    the burst a trigger started has been analysed
//   flpr -> app    Trigger  a heart-rate or breathing trigger fired
//                  Status   heart rate, breathing rate, apnea, leads-off
//                  Recording where the recorded minute is, once it is complete
#pragma once

#include <cstdint>

namespace dev::afe_msg {

enum class Type : uint8_t { Config = 1, Stream, Rearm, Trigger, Status, Recording };
enum class StreamMode : uint8_t { Off = 0, Status, Record };

enum StatusFlags : uint8_t { kHrValid = 1u << 0, kRrValid = 1u << 1, kApnea = 1u << 2, kLeadsOff = 1u << 3 };

struct Config {
	Type type;
	uint8_t trig_apnea; // bool
	uint8_t reserved[2];
	float hr_high, hr_low, rr_high, rr_low;
	uint32_t confirm_ms;
	uint32_t apnea_ms;
};

struct Stream {
	Type type;
	StreamMode mode;
};

struct Rearm {
	Type type;
};

struct Trigger {
	Type type;
	uint8_t bits; // algo::VitalTrigger::Bits
	uint8_t reserved[2];
	float hr_bpm, rr_bpm;
};

struct Status {
	Type type;
	uint8_t flags; // StatusFlags
	uint8_t reserved[2];
	float hr_bpm, rr_bpm;
};

// The recorded minute stays in the coprocessor's RAM (readable by the
// application core) until the next Record request overwrites it.
// Each sample is 6 bytes: ECG then Bio-Z, 24-bit two's complement, big-endian
// as the ADS1292R delivers them.
struct Recording {
	Type type;
	uint8_t reserved[3];
	uint32_t addr;
	uint32_t samples;
	uint32_t sps;
};

static_assert(sizeof(Config) == 28 && sizeof(Trigger) == 12 && sizeof(Status) == 12 &&
	      sizeof(Recording) == 16);

} // namespace dev::afe_msg
