// Deployment motor behind a logic-level MOSFET (gate on a GPIO).
//
// Safety layers:
//   - the gate pin is forced inactive by a SYS_INIT hook right after the GPIO
//     driver starts, before any application code runs (keep a gate pull-down
//     resistor on the board so it is also off during reset / before boot)
//   - fire() only works while armed (armed when the alert starts, disarmed on cancel)
//   - one shot per arm; a k_timer cuts the gate after run_ms even if the
//     application thread is stuck; an optional end-stop cuts it earlier
//   - the hardware watchdog resets the chip (pins -> inputs -> gate pulled off)
#pragma once

#include <atomic>
#include <cstdint>

#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

#include "drivers/irq_pin.hpp"

namespace dev::drv {

class Motor {
public:
	// ok = ran the full stroke (end-stop hit, or timer when no end-stop is required)
	using DoneFn = void (*)(bool ok, void *ctx);

	constexpr Motor(const gpio_dt_spec &gate, const gpio_dt_spec &endstop) : gate_(gate), endstop_(endstop) {}

	int init(DoneFn done, void *ctx);
	void arm() { armed_ = true; }
	void disarm() { armed_ = false; }
	bool armed() const { return armed_; }
	bool running() const { return running_; }

	// Turns the gate on for at most run_ms. Calls done() from ISR context.
	int fire(uint32_t run_ms, bool require_endstop);

private:
	static void on_timer(k_timer *t);
	static void on_endstop(void *self);
	void finish(bool endstop_hit);

	gpio_dt_spec gate_, endstop_;
	k_timer timer_{};
	IrqPin endstop_irq_{};
	DoneFn done_ = nullptr;
	void *ctx_ = nullptr;
	bool require_endstop_ = false;
	std::atomic<bool> armed_{false};
	std::atomic<bool> running_{false};
};

} // namespace dev::drv
