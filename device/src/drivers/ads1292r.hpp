// TI ADS1292R: channel 1 = respiration (Bio-Z, internal 32 kHz modulation),
// channel 2 = ECG, DC lead-off detection on the ECG electrodes.
// One 9-byte frame per DRDY at 125 SPS (RDATAC mode).
#pragma once

#include <cstdint>

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>

#include "drivers/irq_pin.hpp"

namespace dev::drv {

class Ads1292r {
public:
	struct Sample {
		int32_t resp;  // channel 1, counts
		int32_t ecg;   // channel 2, counts
		uint8_t loff;  // LOFF_STAT[4:0]
	};

	static constexpr uint32_t kSps = 125;
	// ECG: 2 * VREF(2.42 V) / gain(12) / (2^23 - 1)
	static constexpr float kEcgUvPerLsb = 2.0f * 2.42e6f / 12.0f / 8388607.0f;
	static constexpr uint8_t kLoffEcg = 0x0C;  // IN2N_OFF | IN2P_OFF
	static constexpr uint8_t kLoffResp = 0x03; // IN1N_OFF | IN1P_OFF

	constexpr Ads1292r(const spi_dt_spec &spi, const gpio_dt_spec &drdy, const gpio_dt_spec &pwdn,
			   const gpio_dt_spec &start)
		: spi_(spi), drdy_(drdy), pwdn_(pwdn), start_(start)
	{
	}

	int init();                         // reset, verify ID, configure, start streaming
	int wait(k_timeout_t timeout) { return k_sem_take(&sem_, timeout); }
	int read(Sample &out);              // after wait() returned 0
	int standby();                      // stop conversions, lowest power

private:
	int transfer(const uint8_t *tx, uint8_t *rx, size_t len);
	int command(uint8_t op);
	int write_reg(uint8_t reg, uint8_t val);
	int read_reg(uint8_t reg, uint8_t &val);
	static void on_drdy(void *self);

	spi_dt_spec spi_;
	gpio_dt_spec drdy_, pwdn_, start_;
	k_sem sem_{};
	IrqPin irq_{};
};

} // namespace dev::drv
