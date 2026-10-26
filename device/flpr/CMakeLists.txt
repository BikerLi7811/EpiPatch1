# Firmware for the nRF54L15's FLPR RISC-V coprocessor: ADS1292R reading,
# heart rate, breathing rate and the two vital-sign triggers. Built by sysbuild
# from ../sysbuild.cmake; it shares its algorithm and driver code with ../src.
cmake_minimum_required(VERSION 3.20.0)

# the wearable,ads1292r binding lives in ../dts/bindings
list(APPEND DTS_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/..)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(afe_flpr LANGUAGES C CXX)

set(DEV_SRC ${CMAKE_CURRENT_SOURCE_DIR}/../src)
target_include_directories(app PRIVATE ${DEV_SRC})
target_sources(app PRIVATE
  src/main.cpp
  ${DEV_SRC}/algo/ecg_qrs.cpp
  ${DEV_SRC}/algo/resp_rate.cpp
  ${DEV_SRC}/drivers/ads1292r.cpp
)
