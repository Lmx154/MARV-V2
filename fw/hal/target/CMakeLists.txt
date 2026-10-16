# The FC3v2 Rev-C target (decision 0027, M1 packet P2): marv_hal_target implements the HAL interface (fw/hal/include) on the
# Pico SDK, and marv_fc3v2 is the linked image. Added from fw/hal/CMakeLists.txt only when MARV_TARGET is m33. Only this
# directory links Pico SDK targets: the library takes the SDK *_headers targets PRIVATE (they add include directories to
# this library alone), the image links the full SDK libraries it needs.
add_library(marv_hal_target STATIC src/hal_target.cpp src/lsm6dsv.cpp src/motor_output.cpp src/motor_test.cpp
  src/manual_control.cpp src/bench_stats.cpp)
target_include_directories(marv_hal_target PUBLIC include)
target_link_libraries(marv_hal_target
  PUBLIC marv_hal marv_types marv_dshot
  PRIVATE marv_lsm6dsv_decode
          pico_base_headers pico_time_headers pico_sync_headers hardware_timer_headers hardware_sync_headers
          hardware_gpio_headers hardware_irq_headers hardware_structs_headers hardware_spi_headers
          hardware_pio_headers hardware_clocks_headers)
# hal_panic polls the TinyUSB device (the image's pico_stdio_usb brings the TinyUSB sources and the tusb_config.h). Only
# the include directories and compile definitions of TinyUSB's own target are taken, not its sources: those are compiled
# once, into the image.
target_include_directories(marv_hal_target SYSTEM PRIVATE
  $<TARGET_PROPERTY:tinyusb_common_base,INTERFACE_INCLUDE_DIRECTORIES>
  $<TARGET_PROPERTY:pico_stdio_usb_headers,INTERFACE_INCLUDE_DIRECTORIES>)
target_compile_definitions(marv_hal_target PRIVATE $<TARGET_PROPERTY:tinyusb_common_base,INTERFACE_COMPILE_DEFINITIONS>)
marv_apply_flags(marv_hal_target FIRMWARE)

# The version in the banner is the git description at configure time; "unknown" outside a git checkout.
execute_process(COMMAND git describe --always --dirty
  WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
  OUTPUT_VARIABLE MARV_FC3V2_VERSION OUTPUT_STRIP_TRAILING_WHITESPACE
  RESULT_VARIABLE marv_describe_result ERROR_QUIET)
if(NOT marv_describe_result EQUAL 0 OR MARV_FC3V2_VERSION STREQUAL "")
  set(MARV_FC3V2_VERSION unknown)
endif()

# The image links the quad flight composition's library and ticks it from the IMU interrupt (M1 P9).
# MARV_FC3V2_COUNT_ONLY (bench only, default OFF): the tick counts and stamps the samples without calling the
# composition, so the driver runs alone; the default image ticks quad_fc.
option(MARV_FC3V2_COUNT_ONLY "FC3v2 bench image whose IMU tick does not call the composition" OFF)
# MARV_FC3V2_BENCH_STATS (bench only, default OFF, M1 P12a): the image records the DWT cycle counts of the IMU
# interrupt's stages, the USBCTRL handler chain and the interrupt-entry spacing, and the still-bench IMU sums, and prints
# them once per second (marv/hal_target/bench_stats.hpp). MARV_FC3V2_BENCH_ACCEPT_SPACING (bench only, default OFF): the
# image raises rate_period_tolerance_us through the parameter table so the rate law's accepted path runs and is timed
# (main.cpp). The default image has neither.
option(MARV_FC3V2_BENCH_STATS "FC3v2 bench image that records stage cycle counts and still-bench IMU sums" OFF)
option(MARV_FC3V2_BENCH_ACCEPT_SPACING "FC3v2 bench image whose rate loop accepts the board's sample spacing" OFF)
target_compile_definitions(marv_hal_target PUBLIC $<$<BOOL:${MARV_FC3V2_BENCH_STATS}>:MARV_FC3V2_BENCH_STATS>)
add_executable(marv_fc3v2 src/main.cpp)
# Programmed to flash; the boot copies the whole image to SRAM and runs it there (core §4.1 EMB-1). Measured with code
# executing from XIP flash the IMU interrupt failed EMB-3 (M1 P12a, tests/regression/quad/L09/results/p12_bench/).
pico_set_binary_type(marv_fc3v2 copy_to_ram)
target_compile_definitions(marv_fc3v2 PRIVATE MARV_FC3V2_VERSION="${MARV_FC3V2_VERSION}"
  $<$<BOOL:${MARV_FC3V2_COUNT_ONLY}>:MARV_FC3V2_COUNT_ONLY>
  $<$<BOOL:${MARV_FC3V2_BENCH_ACCEPT_SPACING}>:MARV_FC3V2_BENCH_ACCEPT_SPACING>)
target_link_libraries(marv_fc3v2 PRIVATE
  marv_hal_target marv_lsm6dsv_decode marv_console_decode marv_composition_quad_fc marv_params_fc3v2_generated marv_params_fc3v2_runtime
  pico_stdlib hardware_spi hardware_pio)
pico_enable_stdio_usb(marv_fc3v2 1)
pico_enable_stdio_uart(marv_fc3v2 0)
marv_apply_flags(marv_fc3v2 FIRMWARE)
