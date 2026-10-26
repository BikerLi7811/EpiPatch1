// Messages between the chest patch and the thigh patch over their encrypted
// Bluetooth LE link (one GATT service on the thigh patch; see patch_link_*.cpp).
// Both patches are nRF54L15s (little-endian, same compiler), and these structs
// only hold naturally aligned scalars, so they are sent as they are.
//
//   chest -> thigh   Decision    the chest's verdict after each analysis:
//                                trigger / severe (rule A or B), findings
//                    Cancel      three taps on the chest
//                    ReportNow   the chest is starting a reading; send fresh data
//   thigh -> chest   Report      histamine, thigh fall/activity, countdown state
//
// The chest makes every decision. The thigh runs the countdown, the tap
// cancel and the release itself, so a countdown that has started finishes
// (or is cancelled) even if the link drops. With no link the thigh does
// nothing new.
#pragma once

#include <cstdint>

namespace dev::patch_msg {

enum class Type : uint8_t { Decision = 1, Cancel, ReportNow, Report };

struct Decision {
	Type type;
	uint8_t trigger; // bool: rule A or B met
	uint8_t severe;  // bool: respiratory + collapse/fall (re-alerts after a cancel)
	uint8_t reserved;
	uint16_t findings; // algo::Finding bits
	uint16_t reserved2;
	uint32_t seq;
};

struct Cancel {
	Type type;
	uint8_t reserved[3];
	uint32_t seq;
};

struct ReportNow {
	Type type;
	uint8_t reserved[3];
};

enum ReportFlags : uint8_t {
	kHistValid = 1u << 0,
	kHistElevated = 1u << 1,
	kFall = 1u << 2,
	kImuValid = 1u << 3,
};

// algo::DeployState as a byte
struct Report {
	Type type;
	uint8_t flags;       // ReportFlags
	uint8_t deploy_state;
	uint8_t deployments;
	float hist_nm;
	float hist_baseline_nm;
	float activity_g;
	uint32_t seq;
};

static_assert(sizeof(Decision) == 12 && sizeof(Cancel) == 8 && sizeof(ReportNow) == 4 &&
	      sizeof(Report) == 20);

} // namespace dev::patch_msg
