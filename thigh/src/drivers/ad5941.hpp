// Analog Devices AD5941: constant-potential amperometry on its low-power loop.
//
// The low-power DAC holds the sensor bias between the reference and sense
// electrodes all the time (electrochemical sensors need a steady bias). The
// low-power TIA turns the cell current into a voltage across RTIA. A reading
// powers the ADC, averages a few SINC2+notch results across RTIA, and powers
// the ADC down again.
//
// Register addresses and values follow ADI's AD594x library (ad5940.c/.h) and
// its amperometric example.
#pragma once

#include <cstdint>

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>

#include "algo/amperometry.hpp"

namespace dev::drv {

class Ad5941 {
public:
	struct Config {
		float vzero_mv;   // sense-electrode potential, 200..2366 mV
		float bias_mv;    // reference minus sense electrode potential
		uint8_t rtia;     // LPTIARTIA_* code (see algo::amperometry::rtia_ohm)
		uint8_t pga;      // ADCPGA_* code
		uint8_t averages; // SINC2 results averaged per reading
	};

	constexpr Ad5941(const spi_dt_spec &spi, const gpio_dt_spec &reset) : spi_(spi), reset_(reset) {}

	// Checks the chip ID, applies ADI's start-up register table, sets up the
	// references and the low-power loop and leaves the bias applied.
	int init(const Config &cfg);
	// One reading: cell current [nA]
	int read_na(float &na);

private:
	int write(uint16_t reg, uint32_t val);
	int read(uint16_t reg, uint32_t &val);
	int set_addr(uint16_t reg);
	int modify(uint16_t reg, uint32_t clear, uint32_t set);

	spi_dt_spec spi_;
	gpio_dt_spec reset_;
	Config cfg_{};
};

} // namespace dev::drv
