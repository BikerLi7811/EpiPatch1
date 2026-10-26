#include "drivers/lsm6dso.hpp"

#include <algorithm>
#include <array>
#include <cmath>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(device, CONFIG_APP_LOG_LEVEL);

namespace dev::drv {

namespace {

constexpr uint8_t kFifoCtrl1 = 0x07; // WTM[7:0]
constexpr uint8_t kFifoCtrl2 = 0x08; // bit 0 = WTM8
constexpr uint8_t kFifoCtrl3 = 0x09;
constexpr uint8_t kFifoCtrl4 = 0x0A;
constexpr uint8_t kWhoAmI = 0x0F;
constexpr uint8_t kInt1Ctrl = 0x0D;  // bit 3 = INT1_FIFO_TH
constexpr uint8_t kCtrl1Xl = 0x10;
constexpr uint8_t kCtrl2G = 0x11;
constexpr uint8_t kCtrl3C = 0x12;
constexpr uint8_t kCtrl6C = 0x15;
constexpr uint8_t kWakeUpSrc = 0x1B; // followed by TAP_SRC 0x1C
constexpr uint8_t kFifoStatus1 = 0x3A; // followed by FIFO_STATUS2, bit 7 = FIFO_WTM_IA
constexpr uint8_t kTapCfg0 = 0x56;
constexpr uint8_t kTapCfg1 = 0x57;
constexpr uint8_t kTapCfg2 = 0x58;
constexpr uint8_t kTapThs6d = 0x59;
constexpr uint8_t kIntDur2 = 0x5A;
constexpr uint8_t kWakeUpThs = 0x5B;
constexpr uint8_t kWakeUpDur = 0x5C;
constexpr uint8_t kRegFreeFall = 0x5D; // (Lsm6dso::kFreeFall is the event bit)
constexpr uint8_t kMd1Cfg = 0x5E;
constexpr uint8_t kFifoDataOutTag = 0x78;

constexpr uint8_t kFs8g = 0x0C;          // CTRL1_XL FS_XL = 11
constexpr float kGPerLsb8g = 0.000244f;  // 0.244 mg/LSB
constexpr uint16_t kTapLsbMg = 8000 / 32; // tap threshold LSB = FS / 32
constexpr uint16_t kWakeLsbMg = 8000 / 64; // wake-up threshold LSB = FS / 64
constexpr uint8_t kTagAccel = 0x02;

uint8_t odr_code(uint16_t hz)
{
	constexpr std::array<uint16_t, 7> odr = {12, 26, 52, 104, 208, 417, 833};
	for (uint8_t i = 0; i < odr.size(); ++i) {
		if (hz <= odr[i]) {
			return i + 1;
		}
	}
	return 7;
}

uint16_t odr_hz(uint8_t code)
{
	constexpr std::array<uint16_t, 8> hz = {0, 13, 26, 52, 104, 208, 417, 833};
	return hz[code & 7];
}

uint8_t ff_code(uint16_t mg)
{
	constexpr std::array<uint16_t, 8> ths = {156, 219, 250, 312, 344, 406, 469, 500};
	uint8_t best = 0;
	for (uint8_t i = 0; i < ths.size(); ++i) {
		if (ths[i] <= mg) {
			best = i;
		}
	}
	return best;
}

template <typename T>
T clampi(long v, long lo, long hi)
{
	return static_cast<T>(std::clamp(v, lo, hi));
}

} // namespace

int Lsm6dso::init(const Config &cfg, void (*on_irq)(void *), void *ctx)
{
	if (!i2c_is_ready_dt(&i2c_)) {
		return -ENODEV;
	}
	uint8_t who = 0;
	if (i2c_reg_read_byte_dt(&i2c_, kWhoAmI, &who) || (who != 0x6C && who != 0x6B)) {
		LOG_ERR("LSM6DSO@0x%02x: WHO_AM_I 0x%02x", i2c_.addr, who);
		return -ENODEV;
	}
	write(kCtrl3C, 0x01); // SW_RESET
	uint8_t c3 = 0x01;
	for (int i = 0; i < 20 && (c3 & 0x01); ++i) {
		k_msleep(1);
		i2c_reg_read_byte_dt(&i2c_, kCtrl3C, &c3);
	}

	const uint8_t odr = odr_code(cfg.odr_hz);
	const uint16_t hz = odr_hz(odr);
	const bool lp = cfg.low_power && hz <= 208;
	g_per_lsb_ = kGPerLsb8g;

	const auto tap_ths = clampi<uint8_t>((cfg.tap_mg + kTapLsbMg / 2) / kTapLsbMg, 1, 31);
	const auto wk_ths = clampi<uint8_t>((cfg.impact_mg + kWakeLsbMg / 2) / kWakeLsbMg, 1, 63);
	const auto ff_dur = clampi<uint8_t>(long(cfg.freefall_ms) * hz / 1000, 1, 63);
	// tap shock window ~40 ms (8/ODR per LSB), quiet ~20 ms (4/ODR per LSB)
	const auto shock = clampi<uint8_t>(lround(0.040 * hz / 8), 1, 3);
	const auto quiet = clampi<uint8_t>(lround(0.020 * hz / 4), 1, 3);

	int err = 0;
	err |= write(kCtrl3C, 0x44);           // BDU, register auto-increment
	err |= write(kCtrl2G, 0x00);           // gyro off
	err |= write(kCtrl6C, lp ? 0x10 : 0);  // XL_HM_MODE: 1 = low-power/normal
	err |= write(kCtrl1Xl, uint8_t(odr << 4) | kFs8g);
	err |= write(kFifoCtrl3, 0x01);        // batch accel at 12.5 Hz, gyro not batched
	err |= write(kFifoCtrl1, cfg.fifo_wtm);
	err |= write(kFifoCtrl2, 0x00);
	err |= write(kFifoCtrl4, 0x06);        // continuous FIFO
	err |= write(kInt1Ctrl, cfg.fifo_wtm ? 0x08 : 0x00);
	// latched interrupts, cleared by reading the source registers
	err |= write(kTapCfg0, uint8_t(0x41 | (cfg.tap ? 0x0E : 0x00)));
	err |= write(kTapCfg1, tap_ths);       // X threshold, priority X>Y>Z
	err |= write(kTapCfg2, uint8_t(0x80 | tap_ths)); // enable embedded functions, Y threshold
	err |= write(kTapThs6d, tap_ths);      // Z threshold
	err |= write(kIntDur2, uint8_t((quiet << 2) | shock));
	err |= write(kWakeUpThs, wk_ths);      // single-tap only (SINGLE_DOUBLE_TAP = 0)
	err |= write(kWakeUpDur, uint8_t((ff_dur & 0x20) << 2));
	err |= write(kRegFreeFall, uint8_t(((ff_dur & 0x1F) << 3) | ff_code(cfg.freefall_mg)));
	err |= write(kMd1Cfg, uint8_t((cfg.tap ? 0x40 : 0x00) | 0x20 | 0x10)); // tap, WU, FF -> INT1
	if (err) {
		return -EIO;
	}
	events(); // clear anything latched during setup
	return irq_.attach(int1_, on_irq, ctx);
}

int Lsm6dso::events()
{
	uint8_t src[2]; // WAKE_UP_SRC, TAP_SRC
	if (i2c_burst_read_dt(&i2c_, kWakeUpSrc, src, sizeof(src))) {
		return -EIO;
	}
	int ev = 0;
	if (src[0] & 0x08) {
		ev |= kImpact;   // WU_IA
	}
	if (src[0] & 0x20) {
		ev |= kFreeFall; // FF_IA
	}
	if (src[1] & 0x20) {
		ev |= kTap;      // SINGLE_TAP
	}
	uint8_t st[2];
	if (i2c_burst_read_dt(&i2c_, kFifoStatus1, st, sizeof(st)) == 0 && (st[1] & 0x80)) {
		ev |= kFifo;     // FIFO_WTM_IA (stays set until the FIFO is read below the watermark)
	}
	return ev;
}

int Lsm6dso::read_fifo(std::span<algo::Vec3> out)
{
	uint8_t st[2];
	if (i2c_burst_read_dt(&i2c_, kFifoStatus1, st, sizeof(st))) {
		return -EIO;
	}
	const std::size_t level = ((st[1] & 0x03u) << 8) | st[0];
	const std::size_t n = std::min(level, out.size());
	std::size_t got = 0;
	for (std::size_t i = 0; i < n; ++i) {
		uint8_t w[7]; // tag + X/Y/Z little-endian
		if (i2c_burst_read_dt(&i2c_, kFifoDataOutTag, w, sizeof(w))) {
			return -EIO;
		}
		if ((w[0] >> 3) != kTagAccel) {
			continue;
		}
		const auto axis = [&](int k) {
			return static_cast<int16_t>(w[1 + 2 * k] | (w[2 + 2 * k] << 8)) * g_per_lsb_;
		};
		out[got++] = {axis(0), axis(1), axis(2)};
	}
	return static_cast<int>(got);
}

} // namespace dev::drv
