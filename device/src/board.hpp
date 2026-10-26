// Every devicetree-described part of the chest patch in one place.
// See boards/*.overlay for the wiring; dts/bindings for the properties.
// (The thigh patch has its own: ../../thigh/src/board.hpp.)
#pragma once

#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/spi.h>

namespace dev::board {

#define DEV_ADS_NODE DT_NODELABEL(ads1292r)
#define DEV_PPG_NODE DT_NODELABEL(ppg)
#define DEV_IMU_CHEST_NODE DT_NODELABEL(imu_chest)
#define DEV_USER_NODE DT_PATH(zephyr_user)

#if !DT_NODE_HAS_STATUS(DEV_PPG_NODE, okay) || !DT_NODE_HAS_STATUS(DEV_IMU_CHEST_NODE, okay)
#error "Devicetree needs nodes labelled ppg and imu_chest (see boards/)"
#endif
// The ADS1292R is on this core only when the FLPR coprocessor isn't reading it
#if !defined(CONFIG_APP_AFE_ON_FLPR) && !DT_NODE_HAS_STATUS(DEV_ADS_NODE, okay)
#error "Devicetree needs a node labelled ads1292r (see boards/)"
#endif

#if !defined(CONFIG_APP_AFE_ON_FLPR)
// ADS1292R on SPI mode 1 (CPOL 0, CPHA 1). Two-argument form: Zephyr >= 4.2
// takes CS delays from the devicetree.
inline constexpr spi_dt_spec kAdsSpi =
	SPI_DT_SPEC_GET(DEV_ADS_NODE, SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_MODE_CPHA);
inline constexpr gpio_dt_spec kAdsDrdy = GPIO_DT_SPEC_GET(DEV_ADS_NODE, drdy_gpios);
inline constexpr gpio_dt_spec kAdsPwdn = GPIO_DT_SPEC_GET_OR(DEV_ADS_NODE, pwdn_gpios, {});
inline constexpr gpio_dt_spec kAdsStart = GPIO_DT_SPEC_GET_OR(DEV_ADS_NODE, start_gpios, {});
#endif

inline constexpr i2c_dt_spec kPpgI2c = I2C_DT_SPEC_GET(DEV_PPG_NODE);
inline constexpr gpio_dt_spec kPpgInt = GPIO_DT_SPEC_GET_OR(DEV_PPG_NODE, int_gpios, {});

inline constexpr i2c_dt_spec kChestI2c = I2C_DT_SPEC_GET(DEV_IMU_CHEST_NODE);
inline constexpr gpio_dt_spec kChestInt = GPIO_DT_SPEC_GET(DEV_IMU_CHEST_NODE, int1_gpios);

// buzzer (mirrors the thigh's countdown) and status LED: both optional
inline constexpr gpio_dt_spec kAlert = GPIO_DT_SPEC_GET_OR(DEV_USER_NODE, alert_gpios, {});
inline constexpr gpio_dt_spec kStatusLed = GPIO_DT_SPEC_GET_OR(DEV_USER_NODE, status_led_gpios, {});

} // namespace dev::board
