# The FC3v2 Rev-C target (decision 0027, M1 packet P2): marv_hal_target implements the HAL interface (fw/hal/include) on the
# Pico SDK, and marv_fc3v2 is the linked image. Added from fw/hal/CMakeLists.txt only when MARV_TARGET is m33. Only this
# directory links Pico SDK targets: the library takes the SDK *_headers targets PRIVATE (they add include directories to
# this library alone), the image links the full SDK libraries it needs.
add_library(marv_hal_target STATIC src/hal_target.cpp src/lsm6dsv.cpp src/motor_output.cpp src/motor_test.cpp
  src/manual_control.cpp src/receiver.cpp src/bench_stats.cpp src/storage_stub.cpp)
target_include_directories(marv_hal_target PUBLIC include)
target_link_libraries(marv_hal_target
  PUBLIC marv_hal marv_types marv_dshot marv_crsf_decode
  PRIVATE marv_lsm6dsv_decode marv_hal_storage
          pico_base_headers pico_time_headers pico_sync_headers hardware_timer_headers hardware_sync_headers
          hardware_gpio_headers hardware_irq_headers hardware_structs_headers hardware_spi_headers
          hardware_pio_headers hardware_clocks_headers hardware_uart_headers)
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
# MARV_FC3V2_IMU_LOG (bench only, default OFF, M3 P3a): the image copies every sample the composition's tick receives into
# an SRAM ring and writes the ring to the USB CDC port as framed binary records (marv/hal_target/imu_log.hpp), with a
# status record once per second in place of the text status line; tools/target/imu_log.py captures and checks it. The
# default image does not compile or link the log. Mutually exclusive with MARV_FC3V2_COUNT_ONLY (the log is of the samples
# the composition receives) and with MARV_FC3V2_BENCH_STATS (its text lines would interleave with the records).
option(MARV_FC3V2_IMU_LOG "FC3v2 bench image that logs every IMU sample the estimator consumes to the host" OFF)
if(MARV_FC3V2_IMU_LOG AND (MARV_FC3V2_COUNT_ONLY OR MARV_FC3V2_BENCH_STATS))
  message(FATAL_ERROR "MARV_FC3V2_IMU_LOG excludes MARV_FC3V2_COUNT_ONLY and MARV_FC3V2_BENCH_STATS")
endif()
if(MARV_FC3V2_IMU_LOG)
  target_sources(marv_hal_target PRIVATE src/imu_log.cpp)
  target_compile_definitions(marv_hal_target PUBLIC MARV_FC3V2_IMU_LOG)
  target_include_directories(marv_hal_target SYSTEM PRIVATE
    $<TARGET_PROPERTY:pico_stdio_headers,INTERFACE_INCLUDE_DIRECTORIES>)
endif()
# MARV_FC3V2_HIL_LINK (bench only, default OFF, HIL stage S1): the image is the USB CDC link measurement alone
# (src/hil_link_main.cpp, hil_link.cpp, hil_link_format.hpp; tools/target/hil_link.py is the host side). It starts no IMU,
# composition tick, receiver or motor output: the DShot PIO program is never loaded and the motor pads are never driven,
# not even with stop frames (decision 0046). MARV_FC3V2_HIL_LINK_MAX_BATCH is the largest downlink record the image accepts,
# in samples: the largest batch size the S1 runs use (a scenario value, not a flight parameter). Mutually exclusive with
# every other bench variant.
option(MARV_FC3V2_HIL_LINK "FC3v2 bench image that measures the USB CDC link in both directions; no flight code runs" OFF)
set(MARV_FC3V2_HIL_LINK_MAX_BATCH 16 CACHE STRING "Largest downlink record of the HIL link image, in samples (scenario value: the largest S1 batch)")
if(MARV_FC3V2_HIL_LINK AND (MARV_FC3V2_COUNT_ONLY OR MARV_FC3V2_BENCH_STATS OR MARV_FC3V2_IMU_LOG OR MARV_FC3V2_BENCH_ACCEPT_SPACING))
  message(FATAL_ERROR "MARV_FC3V2_HIL_LINK excludes MARV_FC3V2_COUNT_ONLY, MARV_FC3V2_BENCH_STATS, MARV_FC3V2_IMU_LOG and MARV_FC3V2_BENCH_ACCEPT_SPACING")
endif()
# MARV_FC3V2_HIL (bench only, default OFF, HIL stage S3a, decision 0046 items 3-5): the image runs quad_fc with the
# generated fc3v2 table unchanged on the IMU samples and ManualControl frames a host sends over USB, and returns its DShot
# values over USB (src/hil_main.cpp, hil.cpp, hil_format.hpp; tools/target/hil_fc.py drives it). It starts no IMU, receiver
# (except with MARV_FC3V2_HIL_RECEIVER, below) or motor output: lsm6dsv.cpp is not compiled, the DShot PIO program is never loaded and the motor pads are never driven,
# not even with stop frames (decision 0046 item 5). Its sample records hold at most MARV_FC3V2_HIL_LINK_MAX_BATCH samples,
# the S1 limit. Mutually exclusive with every other bench variant.
option(MARV_FC3V2_HIL "FC3v2 bench HIL image: quad_fc ticked from USB samples, DShot values returned over USB; no motor pad driven" OFF)
if(MARV_FC3V2_HIL AND (MARV_FC3V2_COUNT_ONLY OR MARV_FC3V2_BENCH_STATS OR MARV_FC3V2_IMU_LOG OR MARV_FC3V2_BENCH_ACCEPT_SPACING OR MARV_FC3V2_HIL_LINK))
  message(FATAL_ERROR "MARV_FC3V2_HIL excludes MARV_FC3V2_COUNT_ONLY, MARV_FC3V2_BENCH_STATS, MARV_FC3V2_IMU_LOG, MARV_FC3V2_BENCH_ACCEPT_SPACING and MARV_FC3V2_HIL_LINK")
endif()
# MARV_FC3V2_HIL_RECEIVER (bench only, default OFF, HIL stage S4, decision 0046 item 4; needs MARV_FC3V2_HIL): the HIL image
# starts the flight image's CRSF receiver (receiver.cpp, its flight interrupt priority) and takes the ManualControl from it,
# not from the host; the host's manual-control records are refused, and the receiver's frames and counters go to the host
# (hil_format.hpp receiver records). Everything else is the HIL image above, motor pads included (none driven).
option(MARV_FC3V2_HIL_RECEIVER "FC3v2 bench HIL image whose ManualControl comes from the board's receiver (HIL S4)" OFF)
if(MARV_FC3V2_HIL_RECEIVER AND NOT MARV_FC3V2_HIL)
  message(FATAL_ERROR "MARV_FC3V2_HIL_RECEIVER needs MARV_FC3V2_HIL")
endif()
if(MARV_FC3V2_HIL)
  target_sources(marv_hal_target PRIVATE src/hil.cpp)
  set_source_files_properties(src/lsm6dsv.cpp PROPERTIES HEADER_FILE_ONLY ON)
  target_compile_definitions(marv_hal_target PRIVATE MARV_FC3V2_HIL MARV_FC3V2_HIL_LINK_MAX_BATCH=${MARV_FC3V2_HIL_LINK_MAX_BATCH})
  target_include_directories(marv_hal_target SYSTEM PRIVATE
    $<TARGET_PROPERTY:pico_stdio_headers,INTERFACE_INCLUDE_DIRECTORIES>)
  add_executable(marv_fc3v2 src/hil_main.cpp)
elseif(MARV_FC3V2_HIL_LINK)
  target_sources(marv_hal_target PRIVATE src/hil_link.cpp)
  target_compile_definitions(marv_hal_target PRIVATE MARV_FC3V2_HIL_LINK MARV_FC3V2_HIL_LINK_MAX_BATCH=${MARV_FC3V2_HIL_LINK_MAX_BATCH})
  target_include_directories(marv_hal_target SYSTEM PRIVATE
    $<TARGET_PROPERTY:pico_stdio_headers,INTERFACE_INCLUDE_DIRECTORIES>)
  add_executable(marv_fc3v2 src/hil_link_main.cpp)
else()
  add_executable(marv_fc3v2 src/main.cpp)
endif()
# Programmed to flash; the boot copies the whole image to SRAM and runs it there (core §4.1 EMB-1). Measured with code
# executing from XIP flash the IMU interrupt failed EMB-3 (M1 P12a, tests/regression/quad/L09/results/p12_bench/).
pico_set_binary_type(marv_fc3v2 copy_to_ram)
target_compile_definitions(marv_fc3v2 PRIVATE MARV_FC3V2_VERSION="${MARV_FC3V2_VERSION}"
  $<$<BOOL:${MARV_FC3V2_COUNT_ONLY}>:MARV_FC3V2_COUNT_ONLY>
  $<$<BOOL:${MARV_FC3V2_HIL_RECEIVER}>:MARV_FC3V2_HIL_RECEIVER>
  $<$<BOOL:${MARV_FC3V2_BENCH_ACCEPT_SPACING}>:MARV_FC3V2_BENCH_ACCEPT_SPACING>)
if(MARV_FC3V2_HIL_LINK)
  target_link_libraries(marv_fc3v2 PRIVATE marv_hal_target marv_lsm6dsv_decode pico_stdlib hardware_spi hardware_pio hardware_uart)
else()
  target_link_libraries(marv_fc3v2 PRIVATE
    marv_composition_quad_fc marv_hal_target marv_lsm6dsv_decode marv_console_decode marv_params_fc3v2_generated marv_params_fc3v2_runtime
    pico_stdlib hardware_spi hardware_pio hardware_uart)
  # The image holds exactly the FC3v2 set's parameter table, or the build fails and the image is removed.
  marv_check_param_table(marv_fc3v2 marv_params_fc3v2)
endif()
pico_enable_stdio_usb(marv_fc3v2 1)
pico_enable_stdio_uart(marv_fc3v2 0)
marv_apply_flags(marv_fc3v2 FIRMWARE)
