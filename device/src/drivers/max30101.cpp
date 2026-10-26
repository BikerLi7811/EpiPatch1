#include "drivers/max30101.hpp"

#include <algorithm>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(device, CONFIG_APP_LOG_LEVEL);

namespace dev::drv {

namespace {

constexpr uint8_t kIntStatus1 = 0x00;
constexpr uint8_t kIntStatus2 = 0x01;
constexpr uint8_t kIntEnable1 = 0x02;
constexpr uint8_t kIntEnable2 = 0x03;
constexpr uint8_t kFifoWrPtr = 0x04;
constexpr uint8_t kOvfCounter = 0x05;
constexpr uint8_t kFifoRdPtr = 0x06;
constexpr uint8_t kFifoData = 0x07;
constexpr uint8_t kFifoConfig = 0x08;
constexpr uint8_t kModeConfig = 0x09;
constexpr uint8_t kSpo2Config = 0x0A;
constexpr uint8_t kLed1Pa = 0x0C; // red
constexpr uint8_t kLed2Pa = 0x0D; // IR
constexpr uint8_t kPartId = 0xFF;

constexpr uint8_t kPartIdMax3010x = 0x15;
constexpr uint8_t kModeReset = 0x40;
constexpr uint8_t kModeShutdown = 0x80;
constexpr uint8_t kModeSpo2 = 0x03;

// SMP_AVE = 2 (001), FIFO rollover on, almost-full when 25 samples are waiting
constexpr uint8_t kFifoCfg = (1u << 5) | (1u << 4) | (Max30101::kFifoDepth - 25);
// ADC range 4096 nA (01), 50 sps (000), 411 us pulses / 18 bit (11)
constexpr uint8_t kSpo2Cfg = (1u << 5) | (0u << 2) | 3u;
constexpr uint8_t kIntAFull = 0x80;

} // namespace

int Max30101::init(uint8_t red_pa, uint8_t ir_pa, void (*on_irq)(void *), void *ctx)
{
	if (!i2c_is_ready_dt(&i2c_)) {
		return -ENODEV;
	}
	uint8_t id = 0;
	if (i2c_reg_read_byte_dt(&i2c_, kPartId, &id) || id != kPartIdMax3010x) {
		LOG_ERR("MAX30101: no part (id 0x%02x)", id);
		return -ENODEV;
	}
	i2c_reg_write_byte_dt(&i2c_, kModeConfig, kModeReset);
	uint8_t mode = kModeReset;
	for (int i = 0; i < 10 && (mode & kModeReset); ++i) {
		k_msleep(1);
		i2c_reg_read_byte_dt(&i2c_, kModeConfig, &mode);
	}
	uint8_t junk;
	i2c_reg_read_byte_dt(&i2c_, kIntStatus1, &junk); // clear power-ready
	i2c_reg_read_byte_dt(&i2c_, kIntStatus2, &junk);

	int err = 0;
	err |= i2c_reg_write_byte_dt(&i2c_, kFifoConfig, kFifoCfg);
	err |= i2c_reg_write_byte_dt(&i2c_, kSpo2Config, kSpo2Cfg);
	err |= i2c_reg_write_byte_dt(&i2c_, kLed1Pa, red_pa);
	err |= i2c_reg_write_byte_dt(&i2c_, kLed2Pa, ir_pa);
	err |= i2c_reg_write_byte_dt(&i2c_, kFifoWrPtr, 0);
	err |= i2c_reg_write_byte_dt(&i2c_, kOvfCounter, 0);
	err |= i2c_reg_write_byte_dt(&i2c_, kFifoRdPtr, 0);
	err |= i2c_reg_write_byte_dt(&i2c_, kIntEnable1, kIntAFull);
	err |= i2c_reg_write_byte_dt(&i2c_, kIntEnable2, 0);
	if (err) {
		return -EIO;
	}
	if (irq_pin_.port) {
		err = irq_.attach(irq_pin_, on_irq, ctx);
		if (err) {
			return err;
		}
	}
	return i2c_reg_write_byte_dt(&i2c_, kModeConfig, kModeSpo2);
}

int Max30101::read(std::span<uint32_t> red, std::span<uint32_t> ir)
{
	uint8_t st;
	i2c_reg_read_byte_dt(&i2c_, kIntStatus1, &st); // releases the INT pin

	uint8_t ptr[3]; // WR_PTR, OVF_COUNTER, RD_PTR
	if (i2c_burst_read_dt(&i2c_, kFifoWrPtr, ptr, sizeof(ptr))) {
		return -EIO;
	}
	std::size_t n = (ptr[0] - ptr[2]) & (kFifoDepth - 1);
	if (ptr[1] != 0) {
		n = kFifoDepth; // overflowed: the whole FIFO is valid (oldest lost)
	}
	n = std::min({n, red.size(), ir.size()});
	if (n == 0) {
		return 0;
	}
	uint8_t raw[kFifoDepth * 6];
	// FIFO_DATA does not auto-increment: a burst read pops successive samples
	if (i2c_burst_read_dt(&i2c_, kFifoData, raw, n * 6)) {
		return -EIO;
	}
	for (std::size_t i = 0; i < n; ++i) {
		const uint8_t *p = &raw[i * 6];
		red[i] = ((uint32_t(p[0]) << 16) | (uint32_t(p[1]) << 8) | p[2]) & 0x3FFFF;
		ir[i] = ((uint32_t(p[3]) << 16) | (uint32_t(p[4]) << 8) | p[5]) & 0x3FFFF;
	}
	return static_cast<int>(n);
}

int Max30101::shutdown() { return i2c_reg_write_byte_dt(&i2c_, kModeConfig, kModeShutdown); }

int Max30101::wake()
{
	int err = 0;
	err |= i2c_reg_write_byte_dt(&i2c_, kFifoWrPtr, 0);
	err |= i2c_reg_write_byte_dt(&i2c_, kOvfCounter, 0);
	err |= i2c_reg_write_byte_dt(&i2c_, kFifoRdPtr, 0);
	err |= i2c_reg_write_byte_dt(&i2c_, kModeConfig, kModeSpo2);
	return err ? -EIO : 0;
}

} // namespace dev::drv
