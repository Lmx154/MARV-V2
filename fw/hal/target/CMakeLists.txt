# The FC3v2 Rev-C target (decision 0027, M1 packet P2): marv_hal_target implements the HAL interface (fw/hal/include) on the
# Pico SDK, and marv_fc3v2 is the linked image. Added from fw/hal/CMakeLists.txt only when MARV_TARGET is m33. Only this
# directory links Pico SDK targets: the library takes the SDK *_headers targets PRIVATE (they add include directories to
# this library alone), the image links the full SDK libraries it needs.
add_library(marv_hal_target STATIC src/hal_target.cpp src/lsm6dsv.cpp src/motor_output.cpp)
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
add_executable(marv_fc3v2 src/main.cpp)
target_compile_definitions(marv_fc3v2 PRIVATE MARV_FC3V2_VERSION="${MARV_FC3V2_VERSION}"
  $<$<BOOL:${MARV_FC3V2_COUNT_ONLY}>:MARV_FC3V2_COUNT_ONLY>)
target_link_libraries(marv_fc3v2 PRIVATE
  marv_hal_target marv_lsm6dsv_decode marv_composition_quad_fc marv_params_fc3v2_generated marv_params_fc3v2_runtime
  pico_stdlib hardware_spi hardware_pio)
pico_enable_stdio_usb(marv_fc3v2 1)
pico_enable_stdio_uart(marv_fc3v2 0)
marv_apply_flags(marv_fc3v2 FIRMWARE)
