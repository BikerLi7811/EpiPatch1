// Conversions for constant-potential amperometry on the AD5941 low-power loop
// (values from ADI's AD594x library and its amperometric example):
//   LPDAC 6-bit output sets Vzero (sense electrode),  1 LSB = 34.375 mV, 200 mV offset
//   LPDAC 12-bit output sets Vbias (reference electrode), 1 LSB = 0.537 mV, 200 mV offset
//   sensor bias = Vbias - Vzero
//   ADC (SINC2 result) code -> volts across RTIA -> current
#pragma once

#include <cstdint>

namespace dev::algo::amperometry {

struct DacCodes {
	uint8_t code6;   // Vzero
	uint16_t code12; // Vbias
};

constexpr float kDac6LsbMv = 34.375f;
constexpr float kDac12LsbMv = 2200.0f / 4095.0f; // 0.537 mV

// vzero_mv: 200..2366 mV; bias_mv: Vbias - Vzero (may be negative)
constexpr DacCodes lpdac_codes(float vzero_mv, float bias_mv)
{
	float c6 = (vzero_mv - 200.0f) / kDac6LsbMv;
	c6 = c6 < 0.0f ? 0.0f : (c6 > 63.0f ? 63.0f : c6);
	const auto code6 = static_cast<uint8_t>(c6);
	float c12 = bias_mv / kDac12LsbMv + code6 * 64.0f;
	c12 = c12 < 0.0f ? 0.0f : (c12 > 4095.0f ? 4095.0f : c12);
	auto code12 = static_cast<uint16_t>(c12);
	// as in ADI's example: round toward Vzero when above it
	if (code12 > code6 * 64) {
		--code12;
	}
	return {code6, code12};
}

// Actual bias the codes produce [mV]
constexpr float bias_mv(DacCodes c) { return (c.code12 - c.code6 * 64) * kDac12LsbMv; }

// PGA gain for ADCPGA_1 / 1P5 / 2 / 4 / 9
constexpr float pga_gain(uint8_t pga)
{
	switch (pga) {
	case 0: return 1.0f;
	case 1: return 1.5f;
	case 2: return 2.0f;
	case 3: return 4.0f;
	case 4: return 9.0f;
	default: return 1.0f;
	}
}

// 16-bit SINC2 code -> volts at the ADC input
constexpr float adc_volts(uint32_t code, uint8_t pga, float vref = 1.82f)
{
	constexpr float kFactor = 1.835f / 1.82f; // ADI's gain correction
	const auto tmp = static_cast<int32_t>(code & 0xFFFF) - 32768;
	return static_cast<float>(tmp) / 32768.0f * (vref / pga_gain(pga)) * kFactor;
}

// Sensor current [nA], oxidation positive (ADI's sign convention)
constexpr float current_na(uint32_t code, uint8_t pga, float rtia_ohm, float vref = 1.82f)
{
	return -adc_volts(code, pga, vref) / rtia_ohm * 1e9f;
}

// Nominal ohms for the LPTIA RTIA settings (LPTIARTIA_200R .. LPTIARTIA_512K)
constexpr float rtia_ohm(uint8_t sel)
{
	constexpr float kOhm[] = {0,     200,   1e3,   2e3,   3e3,    4e3,    6e3,    8e3,    10e3,
				  12e3,  16e3,  20e3,  24e3,  30e3,   32e3,   40e3,   48e3,   64e3,
				  85e3,  96e3,  100e3, 120e3, 128e3,  160e3,  196e3,  256e3,  512e3};
	return sel < sizeof(kOhm) / sizeof(kOhm[0]) ? kOhm[sel] : 0.0f;
}

} // namespace dev::algo::amperometry
