#include "drivers/ads1292r.hpp"

#include <array>

#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(device, CONFIG_APP_LOG_LEVEL);

namespace dev::drv {

namespace {

// commands
constexpr uint8_t kStandby = 0x04;
constexpr uint8_t kReset = 0x06;
constexpr uint8_t kStart = 0x08;
constexpr uint8_t kStop = 0x0A;
constexpr uint8_t kRdatac = 0x10;
constexpr uint8_t kSdatac = 0x11;
constexpr uint8_t kRreg = 0x20;
constexpr uint8_t kWreg = 0x40;

// registers
constexpr uint8_t kId = 0x00;
constexpr uint8_t kConfig1 = 0x01;
constexpr uint8_t kConfig2 = 0x02;
constexpr uint8_t kLoff = 0x03;
constexpr uint8_t kCh1Set = 0x04;
constexpr uint8_t kCh2Set = 0x05;
constexpr uint8_t kRldSens = 0x06;
constexpr uint8_t kLoffSens = 0x07;
constexpr uint8_t kLoffStat = 0x08;
constexpr uint8_t kResp1 = 0x09;
constexpr uint8_t kResp2 = 0x0A;
constexpr uint8_t kGpio = 0x0B;

constexpr uint8_t kIdAds1292r = 0x73;

struct RegVal {
	uint8_t reg, val;
};

constexpr std::array<RegVal, 11> kConfig = {{
	{kConfig2, 0xE0},  // lead-off comparators on, reference buffer on, VREF 2.42 V
	{kConfig1, 0x00},  // continuous, 125 SPS
	{kLoff, 0x10},     // DC lead-off, 95 % threshold, 6 nA
	{kCh1Set, 0x40},   // CH1 (respiration): gain 4, normal input
	{kCh2Set, 0x60},   // CH2 (ECG): gain 12, normal input
	{kRldSens, 0x2C},  // right-leg drive on, derived from CH2
	{kLoffSens, 0x0C}, // lead-off sensing on CH2 P/N
	{kLoffStat, 0x00}, // fMOD = fCLK / 4 (512 kHz clock)
	{kResp1, 0xEA},    // respiration demod + mod on, phase 112.5 deg, internal clock
	{kResp2, 0x03},    // 32 kHz modulation, RLDREF internal
	{kGpio, 0x0C},     // GPIOs as inputs (unused)
}};

int32_t s24(const uint8_t *p)
{
	int32_t v = (int32_t(p[0]) << 16) | (int32_t(p[1]) << 8) | p[2];
	return (v & 0x800000) ? v - 0x1000000 : v;
}

} // namespace

int Ads1292r::transfer(const uint8_t *tx, uint8_t *rx, size_t len)
{
	const spi_buf txb = {.buf = const_cast<uint8_t *>(tx), .len = len};
	const spi_buf_set txs = {.buffers = &txb, .count = 1};
	const spi_buf rxb = {.buf = rx, .len = len};
	const spi_buf_set rxs = {.buffers = &rxb, .count = 1};
	// At 500 kHz SCLK one byte takes 16 us > 4 tCLK, so multi-byte commands
	// need no extra gaps between bytes.
	return spi_transceive_dt(&spi_, &txs, rx ? &rxs : nullptr);
}

int Ads1292r::command(uint8_t op)
{
	const int err = transfer(&op, nullptr, 1);
	k_busy_wait(10);
	return err;
}

int Ads1292r::write_reg(uint8_t reg, uint8_t val)
{
	const uint8_t tx[3] = {uint8_t(kWreg | reg), 0x00, val};
	const int err = transfer(tx, nullptr, sizeof(tx));
	k_busy_wait(10);
	return err;
}

int Ads1292r::read_reg(uint8_t reg, uint8_t &val)
{
	const uint8_t tx[3] = {uint8_t(kRreg | reg), 0x00, 0x00};
	uint8_t rx[3];
	const int err = transfer(tx, rx, sizeof(tx));
	val = rx[2];
	k_busy_wait(10);
	return err;
}

void Ads1292r::on_drdy(void *self) { k_sem_give(&static_cast<Ads1292r *>(self)->sem_); }

int Ads1292r::init()
{
	if (!spi_is_ready_dt(&spi_)) {
		return -ENODEV;
	}
	k_sem_init(&sem_, 0, 1);

	if (pwdn_.port) {
		// PWDN/RESET is active low: pulse it, then hold the chip running
		gpio_pin_configure_dt(&pwdn_, GPIO_OUTPUT_ACTIVE);
		k_msleep(1);
		gpio_pin_set_dt(&pwdn_, 0);
	}
	if (start_.port) {
		gpio_pin_configure_dt(&start_, GPIO_OUTPUT_INACTIVE);
	}
	k_msleep(150); // power-on reset + oscillator start-up

	int err = command(kReset);
	k_msleep(1);
	err |= command(kSdatac);
	if (err) {
		return -EIO;
	}

	uint8_t id = 0;
	read_reg(kId, id);
	if (id != kIdAds1292r) {
		LOG_ERR("ADS1292R: unexpected ID 0x%02x", id);
		return -ENODEV;
	}

	for (const auto &rv : kConfig) {
		err |= write_reg(rv.reg, rv.val);
		if (rv.reg == kConfig2) {
			k_msleep(10); // reference settling
		}
	}
	// read back the two registers that matter most
	uint8_t c1 = 0xFF, ch2 = 0;
	read_reg(kConfig1, c1);
	read_reg(kCh2Set, ch2);
	if (err || c1 != 0x00 || ch2 != 0x60) {
		LOG_ERR("ADS1292R: config readback failed (%02x %02x)", c1, ch2);
		return -EIO;
	}

	err = irq_.attach(drdy_, &Ads1292r::on_drdy, this);
	if (err) {
		return err;
	}
	if (start_.port) {
		gpio_pin_set_dt(&start_, 1);
	} else {
		command(kStart);
	}
	return command(kRdatac);
}

int Ads1292r::read(Sample &out)
{
	const uint8_t tx[9] = {}; // in RAM: EasyDMA cannot read from RRAM
	uint8_t rx[9];
	const int err = transfer(tx, rx, sizeof(rx));
	if (err) {
		return err;
	}
	// status word: 1100 | LOFF_STAT[4:0] | GPIO[1:0] | 0...
	const uint32_t status = (uint32_t(rx[0]) << 16) | (uint32_t(rx[1]) << 8) | rx[2];
	if ((status >> 20) != 0xC) {
		return -EAGAIN; // lost frame sync
	}
	out.loff = (status >> 15) & 0x1F;
	out.resp = s24(&rx[3]);
	out.ecg = s24(&rx[6]);
	return 0;
}

int Ads1292r::standby()
{
	int err = command(kSdatac);
	if (start_.port) {
		gpio_pin_set_dt(&start_, 0);
	} else {
		err |= command(kStop);
	}
	err |= command(kStandby);
	return err ? -EIO : 0;
}

} // namespace dev::drv
