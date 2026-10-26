cmake_minimum_required(VERSION 3.20.0)

# Pull in the wheeze module. In your own project either do the same with the
# path to wheeze/, or add the module to your west.yml (see README).
list(APPEND ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_SOURCE_DIR}/../wheeze)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(wheeze_app LANGUAGES C CXX)

target_sources(app PRIVATE src/main.c)
