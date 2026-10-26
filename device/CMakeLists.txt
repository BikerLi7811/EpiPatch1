cmake_minimum_required(VERSION 3.20.0)

list(APPEND ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_SOURCE_DIR}/../wheeze)

# Where the ADS1292R is read: "flpr" (default) = the RISC-V coprocessor, built
# by sysbuild from flpr/; "app" = an afe thread on this core (no sysbuild needed).
set(AFE_CORE flpr CACHE STRING "Core that reads the ADS1292R: flpr or app")
if(AFE_CORE STREQUAL "flpr")
  list(APPEND EXTRA_DTC_OVERLAY_FILE ${CMAKE_CURRENT_SOURCE_DIR}/boards/afe_flpr.overlay)
elseif(NOT AFE_CORE STREQUAL "app")
  message(FATAL_ERROR "AFE_CORE must be flpr or app")
endif()

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(anaphylaxis_monitor LANGUAGES C CXX)

target_include_directories(app PRIVATE src)
target_sources(app PRIVATE
  src/main.cpp
  src/afe_link.cpp
  src/burst.cpp
  src/patch_link.cpp
  src/algo/anaphylaxis.cpp
  src/algo/body_motion.cpp
  src/algo/ecg_qrs.cpp
  src/algo/ppg.cpp
  src/algo/resp_rate.cpp
  src/drivers/ads1292r.cpp
  src/drivers/alert.cpp
  src/drivers/lsm6dso.cpp
  src/drivers/max30101.cpp
)
