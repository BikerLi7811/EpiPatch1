#include "drivers/alert.hpp"

namespace dev::drv {

int Alert::init()
{
	if (buzzer_.port) {
		gpio_pin_configure_dt(&buzzer_, GPIO_OUTPUT_INACTIVE);
	}
	if (led_.port) {
		gpio_pin_configure_dt(&led_, GPIO_OUTPUT_INACTIVE);
	}
	k_timer_init(&timer_, &Alert::on_tick, nullptr);
	k_timer_user_data_set(&timer_, this);
	return 0;
}

void Alert::play(const Pattern &p, bool sound)
{
	k_timer_stop(&timer_);
	cur_ = &p;
	sound_ = sound;
	pos_ = 0;
	set(false);
	if (p.bits != 0) {
		k_timer_start(&timer_, K_NO_WAIT, K_MSEC(100));
	}
}

void Alert::set(bool on)
{
	if (buzzer_.port) {
		gpio_pin_set_dt(&buzzer_, on && sound_);
	}
	if (led_.port) {
		gpio_pin_set_dt(&led_, on);
	}
}

void Alert::on_tick(k_timer *t)
{
	auto *self = static_cast<Alert *>(k_timer_user_data_get(t));
	const Pattern &p = *self->cur_;
	self->set((p.bits >> self->pos_) & 1u);
	if (++self->pos_ >= p.len) {
		self->pos_ = 0;
		if (!p.repeat) {
			self->set(false);
			k_timer_stop(t);
		}
	}
}

} // namespace dev::drv
