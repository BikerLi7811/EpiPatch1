// A GPIO interrupt input that calls a plain function (with a context) on its
// active edge. Runs in ISR context: post a semaphore/event and return.
#pragma once

#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/util.h>

namespace dev::drv {

struct IrqPin {
	gpio_callback cb;
	void (*fn)(void *);
	void *ctx;

	int attach(const gpio_dt_spec &pin, void (*f)(void *), void *c)
	{
		if (!gpio_is_ready_dt(&pin)) {
			return -ENODEV;
		}
		fn = f;
		ctx = c;
		int err = gpio_pin_configure_dt(&pin, GPIO_INPUT);
		if (err) {
			return err;
		}
		gpio_init_callback(&cb, &IrqPin::trampoline, BIT(pin.pin));
		err = gpio_add_callback_dt(&pin, &cb);
		if (err) {
			return err;
		}
		return gpio_pin_interrupt_configure_dt(&pin, GPIO_INT_EDGE_TO_ACTIVE);
	}

	static void trampoline(const device *, gpio_callback *c, uint32_t)
	{
		IrqPin *self = CONTAINER_OF(c, IrqPin, cb);
		self->fn(self->ctx);
	}
};

} // namespace dev::drv
