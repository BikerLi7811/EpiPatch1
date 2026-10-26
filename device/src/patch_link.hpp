// Chest side of the Bluetooth LE link to the thigh patch (central, GATT
// client). Connects to the thigh, encrypts the link, subscribes to its
// Reports, and sends the chest's decisions and cancels.
// Callbacks run on the Bluetooth thread: post an event and return.
#pragma once

#include <cstdint>

#include "algo/anaphylaxis.hpp"
#include "patch_msgs.hpp"

namespace dev::link {

using ReportFn = void (*)(const patch_msg::Report &r);

int start(ReportFn on_report);

// encrypted, discovered and subscribed
bool ready();

void send_decision(const algo::Assessment &a);
void send_cancel();
void request_report();

} // namespace dev::link
