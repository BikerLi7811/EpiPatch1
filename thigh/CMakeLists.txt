# EpiPatch thigh patch: AD5941 histamine, thigh IMU (fall + tap cancel),
# vibration countdown and the epinephrine release motor. Shares its algorithm
# and driver code with the chest patch in ../device/src.
cmake_minimum_required(VERSION 3.20.0)

# the wearable,* bindings live in ../device/dts/bindings
list(APPEND DTS_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../device)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(epipatch_thigh LANGUAGES C CXX)

set(DEV_SRC ${CMAKE_CURRENT_SOURCE_DIR}/../device/src)
# thigh/src first, so the shared drivers pick up this patch's board.hpp
target_include_directories(app PRIVATE src ${DEV_SRC})
# no biquad filters here, so no CMSIS-DSP: the plain C++ DSP helpers are enough
target_compile_definitions(app PRIVATE DEV_PORTABLE_DSP)
target_sources(app PRIVATE
  src/main.cpp
  src/patch_link.cpp
  src/drivers/ad5941.cpp
  ${DEV_SRC}/persist.cpp
  ${DEV_SRC}/drivers/alert.cpp
  ${DEV_SRC}/drivers/lsm6dso.cpp
  ${DEV_SRC}/drivers/motor.cpp
)
