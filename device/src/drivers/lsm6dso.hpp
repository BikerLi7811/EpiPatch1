// ST LSM6DSO / LSM6DSOX / LSM6DSR accelerometer (gyro stays off).
//
// Uses the chip's own embedded functions so the MCU only wakes on events:
//   single tap   -> INT1 (only on the IMU configured for taps)
//   wake-up      -> INT1, threshold set high = impact detector
//   free-fall    -> INT1
//   FIFO watermark -> INT1 (optional), so posture and collapse keep working
//                     while the application core sleeps between scheduled reads
// and batches accelerometer samples at 12.5 Hz in its FIFO for posture.
#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>

#include "algo/body_motion.hpp"
#include "drivers/irq_pin.hpp"

namespace dev::drv {

class Lsm6dso {
public:
	enum Event : uint8_t { kTap = 1u << 0, kImpact = 1u << 1, kFreeFall = 1u << 2, kFifo = 1u << 3 };

	struct Config {
		uint16_t odr_hz;        // 26 / 52 / 104 / 208 (low-power capable) or 417 / 833
		bool low_power;         // only honoured at <= 208 Hz
		bool tap;               // enable single-tap detection
		uint16_t tap_mg;        // tap threshold
		uint16_t impact_mg;     // wake-up (impact) threshold, <= 7.8 g
		uint16_t freefall_mg;   // 156..500 mg
		uint16_t freefall_ms;
		uint8_t fifo_wtm;       // FIFO watermark interrupt at this many samples, 0 = off
	};

	static constexpr uint32_t kFifoDtMs = 80; // 12.5 Hz batching
	static constexpr std::size_t kMaxFifo = 64;

	constexpr Lsm6dso(const i2c_dt_spec &i2c, const gpio_dt_spec &int1) : i2c_(i2c), int1_(int1) {}

	int init(const Config &cfg, void (*on_irq)(void *), void *ctx);
	// Reads and clears latched events (bitmask of Event), <0 on error.
	int events();
	bool irq_active() const { return gpio_pin_get_dt(&int1_) > 0; }
	// Drains up to out.size() accelerometer samples [g]; returns count or <0.
	int read_fifo(std::span<algo::Vec3> out);

private:
	int write(uint8_t reg, uint8_t val) { return i2c_reg_write_byte_dt(&i2c_, reg, val); }

	i2c_dt_spec i2c_;
	gpio_dt_spec int1_;
	IrqPin irq_{};
	float g_per_lsb_ = 0.000244f;
};

} // namespace dev::drv
