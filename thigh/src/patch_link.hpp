// Thigh side of the Bluetooth LE link to the chest patch (peripheral, GATT
// server). Receives the chest's decisions, cancels and report requests over
// an encrypted link, and notifies Reports back.
// Callbacks run on the Bluetooth thread: post an event and return.
#pragma once

#include "patch_msgs.hpp"

namespace dev::link {

struct Callbacks {
	void (*decision)(const patch_msg::Decision &d);
	void (*cancel)();
	void (*report_now)();
};

int start(const Callbacks &cb);

// the chest is connected over an encrypted link and listening
bool ready();

void send_report(const patch_msg::Report &r);

} // namespace dev::link
