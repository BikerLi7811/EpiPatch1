# Adds the FLPR coprocessor image (flpr/) when the application reads the
# ADS1292R through it (the default; see AFE_CORE in CMakeLists.txt).
if(NOT DEFINED ${DEFAULT_IMAGE}_AFE_CORE OR "${${DEFAULT_IMAGE}_AFE_CORE}" STREQUAL "flpr")
  string(REPLACE "/cpuapp" "/cpuflpr" FLPR_BOARD "${BOARD}${BOARD_QUALIFIERS}")
  ExternalZephyrProject_Add(
    APPLICATION afe_flpr
    SOURCE_DIR  ${APP_DIR}/flpr
    BOARD       ${FLPR_BOARD}
  )
endif()
