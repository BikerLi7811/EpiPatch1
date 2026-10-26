// Buzzer / vibration motor + status LED patterns, stepped by a 100 ms k_timer.
#pragma once

#include <cstdint>

#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

namespace dev::drv {

class Alert {
public:
	// each bit = 100 ms on/off, LSB first
	struct Pattern {
		uint32_t bits;
		uint8_t len;
		bool repeat;
	};
	static constexpr Pattern kOff{0, 1, false};
	static constexpr Pattern kCountdown{0b0011'0011'0011'0011, 16, true};    // fast beeping
	static constexpr Pattern kCountdownEnd{0b1111'1111'0111'1111, 16, true}; // last seconds
	static constexpr Pattern kCancelAck{0b10101, 5, false};                 // three blips
	static constexpr Pattern kDeployed{0b1111, 20, true};                   // 0.4 s beep / 2 s
	static constexpr Pattern kFault{0b1111'1111'1111'1111, 20, true};       // long beeps

	constexpr Alert(const gpio_dt_spec &buzzer, const gpio_dt_spec &led) : buzzer_(buzzer), led_(led) {}

	int init();
	void play(const Pattern &p, bool sound);
	const Pattern *current() const { return cur_; }

private:
	static void on_tick(k_timer *t);
	void set(bool on);

	gpio_dt_spec buzzer_, led_;
	k_timer timer_{};
	const Pattern *cur_ = &kOff;
	bool sound_ = false;
	uint8_t pos_ = 0;
};

} // namespace dev::drv
