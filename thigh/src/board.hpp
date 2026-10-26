// Every devicetree-described part of the thigh patch in one place.
// See boards/*.overlay for the wiring; ../device/dts/bindings for the properties.
#pragma once

#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/spi.h>

namespace dev::board {

#define DEV_AD5941_NODE DT_NODELABEL(ad5941)
#define DEV_IMU_THIGH_NODE DT_NODELABEL(imu_thigh)
#define DEV_USER_NODE DT_PATH(zephyr_user)

#if !DT_NODE_HAS_STATUS(DEV_AD5941_NODE, okay) || !DT_NODE_HAS_STATUS(DEV_IMU_THIGH_NODE, okay)
#error "Devicetree needs nodes labelled ad5941 and imu_thigh (see boards/)"
#endif
#if !DT_NODE_HAS_PROP(DEV_USER_NODE, motor_gpios) || !DT_NODE_HAS_PROP(DEV_USER_NODE, alert_gpios)
#error "zephyr,user needs motor-gpios (release motor MOSFET) and alert-gpios (vibration motor)"
#endif

// AD5941 on SPI mode 0, MSB first
inline constexpr spi_dt_spec kAd5941Spi = SPI_DT_SPEC_GET(DEV_AD5941_NODE, SPI_WORD_SET(8) | SPI_TRANSFER_MSB);
inline constexpr gpio_dt_spec kAd5941Reset = GPIO_DT_SPEC_GET_OR(DEV_AD5941_NODE, reset_gpios, {});

inline constexpr i2c_dt_spec kThighI2c = I2C_DT_SPEC_GET(DEV_IMU_THIGH_NODE);
inline constexpr gpio_dt_spec kThighInt = GPIO_DT_SPEC_GET(DEV_IMU_THIGH_NODE, int1_gpios);

// release motor: MOSFET gate (active = motor on), optional end-of-travel switch
inline constexpr gpio_dt_spec kMotorGate = GPIO_DT_SPEC_GET(DEV_USER_NODE, motor_gpios);
inline constexpr gpio_dt_spec kMotorEndstop = GPIO_DT_SPEC_GET_OR(DEV_USER_NODE, endstop_gpios, {});

// vibration motor (its own GPIO), status LED, service button
inline constexpr gpio_dt_spec kAlert = GPIO_DT_SPEC_GET(DEV_USER_NODE, alert_gpios);
inline constexpr gpio_dt_spec kStatusLed = GPIO_DT_SPEC_GET_OR(DEV_USER_NODE, status_led_gpios, {});
inline constexpr gpio_dt_spec kService = GPIO_DT_SPEC_GET_OR(DEV_USER_NODE, service_gpios, {});

} // namespace dev::board
