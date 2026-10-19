# The FC3v2 Rev-C target (decision 0027, M1 packet P2): marv_hal_target implements the HAL interface (fw/hal/include) on the
# Pico SDK, and marv_fc3v2 is the linked image. Added from fw/hal/CMakeLists.txt only when MARV_TARGET is m33. Only this
# directory links Pico SDK targets: the library takes the SDK *_headers targets PRIVATE (they add include directories to
# this library alone), the image links the full SDK libraries it needs.
add_library(marv_hal_target STATIC src/hal_target.cpp src/lsm6dsv.cpp src/motor_output.cpp src/motor_test.cpp
  src/manual_control.cpp src/receiver.cpp src/bench_stats.cpp src/storage_flash.cpp src/flash_image.cpp src/storage_sd.cpp
  src/sd_spi.cpp)
target_include_directories(marv_hal_target PUBLIC include)
target_link_libraries(marv_hal_target
  PUBLIC marv_hal marv_types marv_dshot marv_crsf_decode
  PRIVATE marv_lsm6dsv_decode marv_hal_storage
          pico_base_headers pico_time_headers pico_sync_headers hardware_timer_headers hardware_sync_headers
          hardware_gpio_headers hardware_irq_headers hardware_structs_headers hardware_spi_headers
          hardware_pio_headers hardware_clocks_headers hardware_uart_headers
          pico_usb_reset_headers pico_bootrom_headers pico_unique_id_headers
          hardware_flash_headers hardware_watchdog_headers pico_sha256_headers
          marv_fatfs)
# FatFs (third_party/fatfs, ground-segment unit 13): the pending file on the SD card (storage_sd.cpp, sd_spi.cpp).
add_subdirectory("${PROJECT_SOURCE_DIR}/third_party/fatfs" "${CMAKE_BINARY_DIR}/third_party/fatfs")
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
# MARV_FC3V2_SD_BENCH (bench only, default OFF, ground-segment unit 13): the background loop mounts the SD card, prints its type,
# size and file system, and runs MARV_FC3V2_SD_BENCH_CYCLES write/verify cycles of a MARV_FC3V2_SD_BENCH_FILE_BYTES buffer
# through hal_config_pending_write / _read and through FatFs directly, timing every operation (src/sd_bench.cpp). Build it
# with MARV_FC3V2_BENCH_STATS on to see the IMU interrupt-entry spacing during the traffic. The values are scenario values,
# not flight parameters: the cycle count is the unit-13 "done when" (1000); the file size is the size of
# marv_params_fc3v2.mcfg at unit 4b (40256 bytes, fw/hal/CMakeLists.txt); the idle baseline is the seconds the bench waits
# after the console host connects, so the spacing before the traffic is on record. The image runs no link service and gives the
# config store no working memory (src/main.cpp): with them its buffers do not fit the SRAM (ground-segment unit 14).
option(MARV_FC3V2_SD_BENCH "FC3v2 bench image that runs the SD write/verify bench in the background loop" OFF)
set(MARV_FC3V2_SD_BENCH_CYCLES 1000 CACHE STRING "SD bench: write/verify cycles (scenario value, unit 13 done-when)")
set(MARV_FC3V2_SD_BENCH_FILE_BYTES 40256 CACHE STRING "SD bench: bytes per file (scenario value: marv_params_fc3v2.mcfg at unit 4b)")
set(MARV_FC3V2_SD_BENCH_IDLE_S 10 CACHE STRING "SD bench: idle seconds after the console host connects (scenario value)")
if(MARV_FC3V2_SD_BENCH AND (MARV_FC3V2_HIL OR MARV_FC3V2_HIL_LINK OR MARV_FC3V2_IMU_LOG OR MARV_FC3V2_COUNT_ONLY))
  message(FATAL_ERROR "MARV_FC3V2_SD_BENCH excludes MARV_FC3V2_HIL, MARV_FC3V2_HIL_LINK, MARV_FC3V2_IMU_LOG and MARV_FC3V2_COUNT_ONLY")
endif()
if(MARV_FC3V2_SD_BENCH)
  target_sources(marv_hal_target PRIVATE src/sd_bench.cpp)
  target_compile_definitions(marv_hal_target PRIVATE MARV_FC3V2_SD_BENCH_CYCLES=${MARV_FC3V2_SD_BENCH_CYCLES}
    MARV_FC3V2_SD_BENCH_FILE_BYTES=${MARV_FC3V2_SD_BENCH_FILE_BYTES} MARV_FC3V2_SD_BENCH_IDLE_S=${MARV_FC3V2_SD_BENCH_IDLE_S})
  target_include_directories(marv_hal_target SYSTEM PRIVATE
    $<TARGET_PROPERTY:pico_stdio_headers,INTERFACE_INCLUDE_DIRECTORIES>)
endif()
# MARV_FC3V2_LOG_BENCH (bench only, default OFF; the onboard SD logger's unit U2, serena memory
# quad-m4/architect-onboard-logger-2026-10-18 items 1-5): the flight image with the onboard logger. The tick pushes every sample,
# rate-tick write and new receiver frame into a ring (src/log_tick.cpp, from the sample sink, hal_actuators_write and
# hal_manual_control; the composition is not changed), and the background loop runs the U1 logger (fw/log) on it with the SD
# card as its storage (src/log_storage_sd.cpp: a contiguous preallocated file per session, data written straight into it).
# The image uses the parameter set marv_params_fc3v2_log (the FC3v2 set plus the log_* entries; the composition and the
# config store are compiled against it: marv_composition_quad_fc_log, marv_store_log), runs no link service and gives the
# config store no working memory, which frees the SRAM for the ring. Its console has the bench commands of
# marv/hal_target/log_bench.hpp, among them the bench trigger `log start`, which logs without arming. The ring and staging
# sizes are the ruling's design values (item 3: a dedicated 256 KiB ring; a 16 KiB staging chunk); the post-link check
# tools/ci/check_log_ring.py fails the build unless the ring holds the fast stream for sd_log_write_stall_max_s (invariant L-5,
# core EMB-6). MARV_FC3V2_LOG_BENCH_FILE_BYTES, when set, replaces log_file_bytes through the parameter table (a scenario value:
# a long file for the soak, a short one for the card-full check). Combines with MARV_FC3V2_BENCH_STATS (EMB-3 with the hooks);
# excludes every other bench variant.
option(MARV_FC3V2_LOG_BENCH "FC3v2 bench image that runs the onboard SD logger (no link service)" OFF)
set(MARV_FC3V2_LOG_RING_BYTES 262144 CACHE STRING
    "Log bench: the ring's bytes (design value, architect ruling 2026-10-18 item 3: a dedicated 256 KiB ring)")
set(MARV_FC3V2_LOG_STAGING_BYTES 16384 CACHE STRING
    "Log bench: the staging chunk's bytes, a whole number of 512-byte sectors (design value, 16 KiB)")
set(MARV_FC3V2_LOG_BENCH_FILE_BYTES "" CACHE STRING
    "Log bench: log_file_bytes through the parameter table (scenario value; empty: the generated value)")
if(MARV_FC3V2_LOG_BENCH AND (MARV_FC3V2_HIL OR MARV_FC3V2_HIL_LINK OR MARV_FC3V2_IMU_LOG OR MARV_FC3V2_COUNT_ONLY OR
                             MARV_FC3V2_SD_BENCH OR MARV_FC3V2_USB_BENCH OR MARV_FC3V2_BENCH_ACCEPT_SPACING))
  message(FATAL_ERROR "MARV_FC3V2_LOG_BENCH excludes MARV_FC3V2_HIL, MARV_FC3V2_HIL_LINK, MARV_FC3V2_IMU_LOG, MARV_FC3V2_COUNT_ONLY, MARV_FC3V2_SD_BENCH, MARV_FC3V2_USB_BENCH and MARV_FC3V2_BENCH_ACCEPT_SPACING")
endif()
if(NOT MARV_FC3V2_LOG_BENCH AND NOT MARV_FC3V2_LOG_BENCH_FILE_BYTES STREQUAL "")
  message(FATAL_ERROR "MARV_FC3V2_LOG_BENCH_FILE_BYTES needs MARV_FC3V2_LOG_BENCH")
endif()
if(MARV_FC3V2_LOG_BENCH)
  target_sources(marv_hal_target PRIVATE src/log_tick.cpp src/log_storage_sd.cpp src/log_bench.cpp)
  target_compile_definitions(marv_hal_target PUBLIC MARV_FC3V2_LOG_BENCH
    PRIVATE MARV_FC3V2_LOG_RING_BYTES=${MARV_FC3V2_LOG_RING_BYTES} MARV_FC3V2_LOG_STAGING_BYTES=${MARV_FC3V2_LOG_STAGING_BYTES})
  target_link_libraries(marv_hal_target PUBLIC marv_log)
endif()
# The composite USB device (ground-segment unit 12a, decision 0053; src/usb.cpp, marv/hal_target/usb.hpp): every image but the
# two HIL ones keeps the SDK's pico_stdio_usb for the console (CDC 0, interface 0) and supplies its own descriptors (console
# CDC, link CDC at interface 2, the Raspberry Pi reset interface at interface 4), a tusb_config.h with two CDC instances
# (usb_config/, ahead of the SDK's in the include path, which it includes), and the reset gate. The SDK's default
# descriptors, its default registration of the reset interface's driver and its 1200-baud reset are switched off:
# usb.cpp provides the first, a gated driver around the SDK's reset handler, and a gated line-coding callback.
# PICO_USB_RESET_MS_OS_20_DESCRIPTOR_ITF is the reset interface's number (kItfReset in usb.cpp, which asserts it). The HIL
# images keep the SDK's single-CDC device and its descriptors unchanged.
if(NOT MARV_FC3V2_HIL AND NOT MARV_FC3V2_HIL_LINK)
  target_sources(marv_hal_target PRIVATE src/usb.cpp)
  target_include_directories(marv_hal_target BEFORE PUBLIC usb_config)
  target_compile_definitions(marv_hal_target PUBLIC
    PICO_STDIO_USB_USE_DEFAULT_DESCRIPTORS=0
    PICO_USB_RESET_INCLUDE_DEFAULT_APP_DRIVER_CB=0
    PICO_ENABLE_USB_RESET_VIA_BAUD_RATE=0
    PICO_USB_RESET_MS_OS_20_DESCRIPTOR_ITF=4)
  # The ground link's transport (ground-segment unit 12b; src/link_port.cpp, marv/hal_target/link_port.hpp): the link service
  # (fw/link) runs on the link CDC from main.cpp's background loop. MARV_FC3V2_LINK_QUEUE_FRAMES is a PLACEHOLDER: the frames of
  # the longest length (marv/link/messages.hpp kFrameBytesMax) its send queue holds, room for one listing step (two frames),
  # the heartbeat and the answers to the few requests one loop pass reads; ground-segment unit 11 replaces it with the link
  # budget (decision 0053 Q4: the total rate capped by a design-budget entry from the measured link throughput).
  set(MARV_FC3V2_LINK_QUEUE_FRAMES 16 CACHE STRING
      "Frames of the longest length the board's link send queue holds (PLACEHOLDER until the link budget of ground-segment unit 11)")
  target_sources(marv_hal_target PRIVATE src/link_port.cpp)
  target_link_libraries(marv_hal_target PRIVATE marv_link)
  target_compile_definitions(marv_hal_target PRIVATE MARV_FC3V2_LINK_QUEUE_FRAMES=${MARV_FC3V2_LINK_QUEUE_FRAMES})
endif()
# MARV_FC3V2_REBOOT_DELAY_MS is a PLACEHOLDER (ground-segment unit 14): how long hal_reboot (src/storage_flash.cpp) lets the
# background loop run after an accepted reboot before the watchdog resets the chip, so the acknowledgement already queued
# (of the config or reboot command) reaches the host. Chosen well above the time to drain a full send queue at the measured link
# rate (16 frames of at most 280 bytes at about 400 kB/s, about 11 ms; L10/results/link_board); replaced by a measured drain
# time with the link budget of ground-segment unit 11.
set(MARV_FC3V2_REBOOT_DELAY_MS 100 CACHE STRING
    "Milliseconds between an accepted reboot and the watchdog reset (PLACEHOLDER until the link budget of ground-segment unit 11)")
target_compile_definitions(marv_hal_target PRIVATE MARV_FC3V2_REBOOT_DELAY_MS=${MARV_FC3V2_REBOOT_DELAY_MS})
# MARV_FC3V2_USB_BENCH (bench only, default OFF, ground-segment unit 12a): the background loop echoes every byte received on the
# link port back on it (usb_bench_echo_step), for the bench check of the transport; that image does not run the link service.
option(MARV_FC3V2_USB_BENCH "FC3v2 bench image that echoes the USB link port" OFF)
if(MARV_FC3V2_USB_BENCH AND (MARV_FC3V2_HIL OR MARV_FC3V2_HIL_LINK))
  message(FATAL_ERROR "MARV_FC3V2_USB_BENCH excludes MARV_FC3V2_HIL and MARV_FC3V2_HIL_LINK (they have no link port)")
endif()
target_compile_definitions(marv_hal_target PUBLIC $<$<BOOL:${MARV_FC3V2_USB_BENCH}>:MARV_FC3V2_USB_BENCH>)
# Programmed to flash; the boot copies the whole image to SRAM and runs it there (core §4.1 EMB-1). Measured with code
# executing from XIP flash the IMU interrupt failed EMB-3 (M1 P12a, tests/regression/quad/L09/results/p12_bench/).
pico_set_binary_type(marv_fc3v2 copy_to_ram)
target_compile_definitions(marv_fc3v2 PRIVATE MARV_FC3V2_VERSION="${MARV_FC3V2_VERSION}"
  $<$<BOOL:${MARV_FC3V2_COUNT_ONLY}>:MARV_FC3V2_COUNT_ONLY>
  $<$<BOOL:${MARV_FC3V2_SD_BENCH}>:MARV_FC3V2_SD_BENCH>
  $<$<BOOL:${MARV_FC3V2_HIL_RECEIVER}>:MARV_FC3V2_HIL_RECEIVER>
  $<$<BOOL:${MARV_FC3V2_BENCH_ACCEPT_SPACING}>:MARV_FC3V2_BENCH_ACCEPT_SPACING>)
if(MARV_FC3V2_HIL_LINK)
  target_link_libraries(marv_fc3v2 PRIVATE marv_hal_target marv_lsm6dsv_decode pico_stdlib hardware_spi hardware_pio hardware_uart)
elseif(MARV_FC3V2_LOG_BENCH)
  target_link_libraries(marv_fc3v2 PRIVATE
    marv_composition_quad_fc_log marv_store_log marv_hal_target marv_lsm6dsv_decode marv_console_decode marv_log
    marv_params_fc3v2_log_generated marv_params_fc3v2_log_runtime pico_stdlib hardware_spi hardware_pio hardware_uart
    hardware_flash hardware_watchdog pico_sha256 pico_rand)
  if(NOT MARV_FC3V2_LOG_BENCH_FILE_BYTES STREQUAL "")
    target_compile_definitions(marv_fc3v2 PRIVATE MARV_FC3V2_LOG_BENCH_FILE_BYTES=${MARV_FC3V2_LOG_BENCH_FILE_BYTES})
  endif()
  marv_check_param_table(marv_fc3v2 marv_params_fc3v2_log)
  # Invariant L-5 (core EMB-6) on the linked ring and the set's parameters, or the build fails and the image is removed.
  set(marv_log_python "${MARV_PYTHON}")
  if(marv_log_python STREQUAL "")
    set(marv_log_python "${PROJECT_SOURCE_DIR}/.venv/bin/python")
  endif()
  add_custom_command(TARGET marv_fc3v2 POST_BUILD
    COMMAND "${marv_log_python}" "${PROJECT_SOURCE_DIR}/tools/ci/check_log_ring.py"
            --manifest "$<TARGET_PROPERTY:marv_params_fc3v2_log_generated,MARV_PARAM_MANIFEST>"
            --elf "$<TARGET_FILE:marv_fc3v2>" --nm "${CMAKE_NM}" --delete-on-failure
    COMMENT "Checking the log ring of marv_fc3v2 against the fast stream x the worst SD stall (L-5)"
    VERBATIM)
else()
  target_link_libraries(marv_fc3v2 PRIVATE
    marv_composition_quad_fc marv_store marv_hal_target marv_lsm6dsv_decode marv_console_decode marv_params_fc3v2_generated
    marv_params_fc3v2_runtime pico_stdlib hardware_spi hardware_pio hardware_uart hardware_flash hardware_watchdog pico_sha256)
  if(NOT MARV_FC3V2_HIL)
    target_link_libraries(marv_fc3v2 PRIVATE marv_link_service)
  endif()
  # The image holds exactly the FC3v2 set's parameter table, or the build fails and the image is removed.
  marv_check_param_table(marv_fc3v2 marv_params_fc3v2)
endif()
# Every image ends below the config slots at the top of the flash, or the build fails and the image is removed.
marv_check_flash_layout(marv_fc3v2)
pico_enable_stdio_usb(marv_fc3v2 1)
pico_enable_stdio_uart(marv_fc3v2 0)
marv_apply_flags(marv_fc3v2 FIRMWARE)

# Invariant L-6 (unit U3 BENCH-PROFILE): no flight image contains the bench motor-step profile. After every link,
# tools/ci/check_bench_profile.py lists the image's symbols (nm -C): FORBID fails the build and removes the image if a symbol
# names bench_profile; REQUIRE fails it unless one does (the negative control: the same check finds the profile in the image
# that has it). MARV_PYTHON as in cmake/flash_layout.cmake (this directory is configured before fw/params on a first configure).
function(marv_check_bench_profile target mode)
  set(python "${MARV_PYTHON}")
  if(python STREQUAL "")
    set(python "${PROJECT_SOURCE_DIR}/.venv/bin/python")
  endif()
  if(mode STREQUAL "FORBID")
    set(check_args --forbid "$<TARGET_FILE:${target}>" --delete-on-failure)
  elseif(mode STREQUAL "REQUIRE")
    set(check_args --require "$<TARGET_FILE:${target}>")
  else()
    message(FATAL_ERROR "marv_check_bench_profile(${target} ${mode}): the mode is FORBID or REQUIRE")
  endif()
  add_custom_command(TARGET ${target} POST_BUILD
    COMMAND "${python}" "${PROJECT_SOURCE_DIR}/tools/ci/check_bench_profile.py" --nm "${CMAKE_NM}" ${check_args}
    COMMENT "Checking the bench profile's symbols in ${target} (invariant L-6, ${mode})"
    VERBATIM)
endfunction()
marv_check_bench_profile(marv_fc3v2 FORBID)

# The bench motor-step profile image (unit U3 BENCH-PROFILE, architect ruling 2026-10-18 item 8; the props-off runbook is
# tests/regression/quad/L10/results/bench_profile/README.md): main.cpp built with MARV_FC3V2_BENCH_PROFILE, linking the
# composition variant marv_composition_quad_fc_bench_profile (the profile in place of the mixer's output while Armed, run
# and killed from the radio, no USB needed) with the FC3v2 set plus the bench-profile register, and the console. It has no
# config store and no link service: both are built against marv_params_fc3v2, whose table this set's is not, so the image
# boots the compiled defaults, and a bench value changes only with a new image (a new hash). The onboard logger joins it
# with unit U2. Built in the default configuration only: every bench option above changes marv_hal_target, which it shares.
if(NOT (MARV_FC3V2_COUNT_ONLY OR MARV_FC3V2_BENCH_STATS OR MARV_FC3V2_BENCH_ACCEPT_SPACING OR MARV_FC3V2_IMU_LOG OR
        MARV_FC3V2_HIL_LINK OR MARV_FC3V2_HIL OR MARV_FC3V2_SD_BENCH OR MARV_FC3V2_USB_BENCH OR MARV_FC3V2_LOG_BENCH))
  add_executable(marv_fc3v2_bench_profile src/main.cpp)
  pico_set_binary_type(marv_fc3v2_bench_profile copy_to_ram)
  target_compile_definitions(marv_fc3v2_bench_profile PRIVATE MARV_FC3V2_VERSION="${MARV_FC3V2_VERSION}"
    MARV_FC3V2_BENCH_PROFILE)
  target_link_libraries(marv_fc3v2_bench_profile PRIVATE
    marv_composition_quad_fc_bench_profile marv_hal_target marv_lsm6dsv_decode marv_console_decode
    marv_params_fc3v2_bench_profile_generated marv_params_fc3v2_bench_profile_runtime
    pico_stdlib hardware_spi hardware_pio hardware_uart hardware_flash hardware_watchdog pico_sha256)
  marv_check_param_table(marv_fc3v2_bench_profile marv_params_fc3v2_bench_profile)
  marv_check_flash_layout(marv_fc3v2_bench_profile)
  marv_check_bench_profile(marv_fc3v2_bench_profile REQUIRE)
  pico_enable_stdio_usb(marv_fc3v2_bench_profile 1)
  pico_enable_stdio_uart(marv_fc3v2_bench_profile 0)
  marv_apply_flags(marv_fc3v2_bench_profile FIRMWARE)
endif()
