# Wheeze detector Zephyr module.
if(CONFIG_WHEEZE_DETECTOR)
  zephyr_include_directories(include)

  zephyr_library_named(wheeze)
  zephyr_library_sources(
    src/wheeze_service.cpp
    src/feature_extractor.cpp
    src/wheeze_net.cpp
    src/generated/dsp_tables.cpp
    src/generated/model_data.cpp
  )
  zephyr_library_include_directories(src)
  # Hot DSP/NN path: optimise for speed (race to idle) even in size-optimised apps.
  zephyr_library_compile_options(-O3)
endif()
