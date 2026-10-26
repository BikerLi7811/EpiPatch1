// Maxim MAX30101/MAX30102 PPG in SpO2 mode (red + IR).
// 50 sps with 2-sample averaging -> 25 Hz into the 32-deep FIFO; the
// almost-full interrupt fires about once a second, so the MCU sleeps between.
#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>

#include "drivers/irq_pin.hpp"

namespace dev::drv {

class Max30101 {
public:
	static constexpr std::size_t kFifoDepth = 32;
	static constexpr uint32_t kFs = 25;

	constexpr Max30101(const i2c_dt_spec &i2c, const gpio_dt_spec &irq) : i2c_(i2c), irq_pin_(irq) {}

	// led currents in 0.2 mA steps (0..255 = 0..51 mA)
	int init(uint8_t red_pa, uint8_t ir_pa, void (*on_irq)(void *), void *ctx);
	// Drains the FIFO into red/ir (each at least kFifoDepth long). Returns count or <0.
	int read(std::span<uint32_t> red, std::span<uint32_t> ir);
	int shutdown();
	// Back from shutdown(): FIFO emptied, sampling again.
	int wake();

private:
	i2c_dt_spec i2c_;
	gpio_dt_spec irq_pin_;
	IrqPin irq_{};
};

} // namespace dev::drv
