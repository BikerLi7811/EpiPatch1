#include "drivers/ad5941.hpp"

#include <array>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ad5941, CONFIG_APP_LOG_LEVEL);

namespace dev::drv {
namespace {

// SPI commands
constexpr uint8_t kSetAddr = 0x20;
constexpr uint8_t kWriteReg = 0x2D;
constexpr uint8_t kReadReg = 0x6D;

// registers (ad5940.h)
constexpr uint16_t kAdiId = 0x0400;
constexpr uint16_t kChipId = 0x0404;
constexpr uint16_t kAfeCon = 0x2000;
constexpr uint16_t kAdcFilterCon = 0x2044;
constexpr uint16_t kLpRefBufCon = 0x2050;
constexpr uint16_t kSinc2Dat = 0x2080;
constexpr uint16_t kLpTiaSw0 = 0x20E4;
constexpr uint16_t kLpTiaCon0 = 0x20EC;
constexpr uint16_t kLpDacDat0 = 0x2120;
constexpr uint16_t kLpDacSw0 = 0x2124;
constexpr uint16_t kLpDacCon0 = 0x2128;
constexpr uint16_t kBufSenCon = 0x2180;
constexpr uint16_t kAdcCon = 0x21A8;
constexpr uint16_t kPmbw = 0x22F0;
constexpr uint16_t kIntcClr = 0x3004;
constexpr uint16_t kIntcSel0 = 0x3008;
constexpr uint16_t kIntcFlag0 = 0x3010;

// AFECON bits
constexpr uint32_t kHpRefDis = 1u << 5;
constexpr uint32_t kAdcEn = 1u << 7;
constexpr uint32_t kAdcConvEn = 1u << 8;
constexpr uint32_t kSinc2En = 1u << 16;
// interrupt source: SINC2/notch result ready
constexpr uint32_t kSinc2Rdy = 1u << 2;

constexpr uint32_t kAdiIdValue = 0x4144;

// registers 0x1000..0x3014 are 32-bit, the rest 16-bit
constexpr bool is_long(uint16_t reg) { return reg >= 0x1000 && reg <= 0x3014; }

// Start-up table that ADI's library writes after every reset (AD5940_Initialize)
struct RegVal {
	uint16_t reg;
	uint32_t val;
};
constexpr std::array<RegVal, 13> kInitTable{{
	{0x0908, 0x02C9},
	{0x0C08, 0x206C},
	{0x21F0, 0x0010},
	{0x0410, 0x02C9},
	{0x0A28, 0x0009},
	{0x238C, 0x0104},
	{0x0A04, 0x4859},
	{0x0A04, 0xF27B},
	{0x0A00, 0x8009},
	{0x22F0, 0x0000},
	{0x2230, 0xDE87A5AF},
	{0x2250, 0x103F},
	{0x22B0, 0x203C},
}};
constexpr RegVal kInitLast{0x2230, 0xDE87A5A0};

} // namespace

int Ad5941::set_addr(uint16_t reg)
{
	uint8_t tx[3] = {kSetAddr, static_cast<uint8_t>(reg >> 8), static_cast<uint8_t>(reg)};
	const spi_buf b{.buf = tx, .len = sizeof(tx)};
	const spi_buf_set s{.buffers = &b, .count = 1};
	return spi_write_dt(&spi_, &s);
}

int Ad5941::write(uint16_t reg, uint32_t val)
{
	int err = set_addr(reg);
	if (err) {
		return err;
	}
	uint8_t tx[5] = {kWriteReg};
	std::size_t n;
	if (is_long(reg)) {
		tx[1] = static_cast<uint8_t>(val >> 24);
		tx[2] = static_cast<uint8_t>(val >> 16);
		tx[3] = static_cast<uint8_t>(val >> 8);
		tx[4] = static_cast<uint8_t>(val);
		n = 5;
	} else {
		tx[1] = static_cast<uint8_t>(val >> 8);
		tx[2] = static_cast<uint8_t>(val);
		n = 3;
	}
	const spi_buf b{.buf = tx, .len = n};
	const spi_buf_set s{.buffers = &b, .count = 1};
	return spi_write_dt(&spi_, &s);
}

int Ad5941::read(uint16_t reg, uint32_t &val)
{
	int err = set_addr(reg);
	if (err) {
		return err;
	}
	// command, one dummy byte, then the data
	const std::size_t n = is_long(reg) ? 6 : 4;
	uint8_t tx[6] = {kReadReg};
	uint8_t rx[6] = {};
	const spi_buf tb{.buf = tx, .len = n};
	const spi_buf rb{.buf = rx, .len = n};
	const spi_buf_set ts{.buffers = &tb, .count = 1};
	const spi_buf_set rs{.buffers = &rb, .count = 1};
	err = spi_transceive_dt(&spi_, &ts, &rs);
	if (err) {
		return err;
	}
	val = 0;
	for (std::size_t i = 2; i < n; ++i) {
		val = (val << 8) | rx[i];
	}
	return 0;
}

int Ad5941::modify(uint16_t reg, uint32_t clear, uint32_t set)
{
	uint32_t v;
	int err = read(reg, v);
	if (err) {
		return err;
	}
	return write(reg, (v & ~clear) | set);
}

int Ad5941::init(const Config &cfg)
{
	cfg_ = cfg;
	if (!spi_is_ready_dt(&spi_)) {
		return -ENODEV;
	}
	if (reset_.port) {
		gpio_pin_configure_dt(&reset_, GPIO_OUTPUT_ACTIVE); // in reset
		k_msleep(1);
		gpio_pin_set_dt(&reset_, 0);
	}
	k_msleep(5);

	uint32_t id = 0;
	if (read(kAdiId, id) || id != kAdiIdValue) {
		LOG_ERR("AD5941: ADIID 0x%04x", id);
		return -ENODEV;
	}
	int err = 0;
	for (const auto &rv : kInitTable) {
		err |= write(rv.reg, rv.val);
	}
	err |= write(kInitLast.reg, kInitLast.val);
	uint32_t chip = 0;
	err |= read(kChipId, chip);
	if (err || (chip & 0xFF00) != 0x5500) {
		LOG_ERR("AD5941: CHIPID 0x%04x", chip);
		return -ENODEV;
	}

	const auto codes = algo::amperometry::lpdac_codes(cfg.vzero_mv, cfg.bias_mv);
	err |= write(kPmbw, 0u | (1u << 2));                  // low-power mode, 50 kHz bandwidth
	err |= modify(kAfeCon, kHpRefDis, 0);                 // high-power bandgap on (ADC reference)
	err |= modify(kBufSenCon, 0, 0x01 | 0x04 | 0x10 | 0x20); // 1.8 V and 1.1 V HP/LP buffers
	err |= write(kLpRefBufCon, 0);                        // LP bandgap + 2.5 V buffer on
	// LPDAC0: data from register, Vzero = 6-bit, Vbias = 12-bit, 2.5 V ref, powered, writable
	err |= write(kLpDacCon0, 0x01);
	err |= write(kLpDacDat0, (static_cast<uint32_t>(codes.code6) << 12) | codes.code12);
	// Vbias -> PA and pin, Vzero -> TIA and pin; switches under register control
	err |= write(kLpDacSw0, 0x1E | 0x20);
	// LPTIA0 + PA powered, normal power, RTIA, 100 ohm Rload, 1 Mohm LPF
	err |= write(kLpTiaCon0, (static_cast<uint32_t>(cfg.rtia) << 5) | (4u << 10) | (7u << 13));
	err |= write(kLpTiaSw0, (1u << 2) | (1u << 4) | (1u << 5) | (1u << 12) | (1u << 13));
	// ADC across RTIA: + = LPTIA0 output, - = LPTIA0 inverting input
	err |= write(kAdcCon, 0x21u | (0x02u << 8) | (static_cast<uint32_t>(cfg.pga) << 16));
	// 800 kHz ADC clock, SINC3 OSR 4, SINC2 OSR 22, notch on
	err |= write(kAdcFilterCon, 0x1u | (0u << 8) | (1u << 12));
	err |= modify(kIntcSel0, 0, kSinc2Rdy);
	if (err) {
		return -EIO;
	}
	LOG_INF("AD5941 ready: chip 0x%04x, bias %d mV", chip,
		static_cast<int>(algo::amperometry::bias_mv(codes)));
	return 0;
}

int Ad5941::read_na(float &na)
{
	constexpr int kDiscard = 4; // notch filter settling after the ADC starts
	int err = modify(kAfeCon, 0, kAdcEn | kSinc2En);
	k_usleep(500);
	err |= write(kIntcClr, kSinc2Rdy);
	err |= modify(kAfeCon, 0, kAdcConvEn);
	uint64_t sum = 0;
	int got = 0;
	for (int i = 0; !err && i < kDiscard + cfg_.averages; ++i) {
		uint32_t flag = 0;
		int tries = 0;
		while (!(err |= read(kIntcFlag0, flag)) && !(flag & kSinc2Rdy) && ++tries < 200) {
			k_busy_wait(100);
		}
		if (!(flag & kSinc2Rdy)) {
			err = -ETIMEDOUT;
			break;
		}
		uint32_t code = 0;
		err |= read(kSinc2Dat, code);
		err |= write(kIntcClr, kSinc2Rdy);
		if (i >= kDiscard) {
			sum += code & 0xFFFF;
			++got;
		}
	}
	modify(kAfeCon, kAdcConvEn | kAdcEn | kSinc2En, 0); // ADC off, bias stays on
	if (err || got == 0) {
		return err ? err : -EIO;
	}
	const auto mean = static_cast<uint32_t>(sum / static_cast<uint64_t>(got));
	na = algo::amperometry::current_na(mean, cfg_.pga, algo::amperometry::rtia_ohm(cfg_.rtia));
	return 0;
}

} // namespace dev::drv
