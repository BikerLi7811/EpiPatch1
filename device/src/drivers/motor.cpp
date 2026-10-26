#include "drivers/motor.hpp"

#include <zephyr/init.h>

#include "board.hpp"

namespace dev::drv {

namespace {

// Gate off as early as possible: POST_KERNEL, just after the GPIO drivers
// (CONFIG_GPIO_INIT_PRIORITY, 40 by default).
int gate_safe_init()
{
	if (!gpio_is_ready_dt(&board::kMotorGate)) {
		return -ENODEV;
	}
	return gpio_pin_configure_dt(&board::kMotorGate, GPIO_OUTPUT_INACTIVE);
}

} // namespace

SYS_INIT(gate_safe_init, POST_KERNEL, 41);

int Motor::init(DoneFn done, void *ctx)
{
	if (!gpio_is_ready_dt(&gate_)) {
		return -ENODEV;
	}
	done_ = done;
	ctx_ = ctx;
	int err = gpio_pin_configure_dt(&gate_, GPIO_OUTPUT_INACTIVE);
	if (err) {
		return err;
	}
	k_timer_init(&timer_, &Motor::on_timer, nullptr);
	k_timer_user_data_set(&timer_, this);
	if (endstop_.port) {
		err = endstop_irq_.attach(endstop_, &Motor::on_endstop, this);
	}
	return err;
}

int Motor::fire(uint32_t run_ms, bool require_endstop)
{
	bool expected = true;
	// consume the arm: exactly one stroke per arm()
	if (!armed_.compare_exchange_strong(expected, false)) {
		return -EPERM;
	}
	if (require_endstop && !endstop_.port) {
		return -ENOTSUP;
	}
	require_endstop_ = require_endstop;
	running_ = true;
	k_timer_start(&timer_, K_MSEC(run_ms), K_NO_WAIT);
	const int err = gpio_pin_set_dt(&gate_, 1);
	if (err) {
		finish(false);
	}
	return err;
}

void Motor::finish(bool endstop_hit)
{
	// both the timer and the end-stop may race here; first one wins
	bool expected = true;
	if (!running_.compare_exchange_strong(expected, false)) {
		return;
	}
	gpio_pin_set_dt(&gate_, 0);
	k_timer_stop(&timer_);
	const bool ok = endstop_hit || !require_endstop_;
	if (done_) {
		done_(ok, ctx_);
	}
}

void Motor::on_timer(k_timer *t) { static_cast<Motor *>(k_timer_user_data_get(t))->finish(false); }

void Motor::on_endstop(void *self) { static_cast<Motor *>(self)->finish(true); }

} // namespace dev::drv
