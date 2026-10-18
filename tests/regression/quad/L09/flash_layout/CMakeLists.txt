# The post-link flash-layout check's negative control (cmake/flash_layout.cmake; ci/run_ci.sh flash_layout_control; decision
# 0053, ground-segment unit 14), m33 only (added from the top-level CMakeLists.txt). Built only by its CI step: an SDK image
# (the default flash binary type) whose read-only data alone fills the flash up to the config slots, so its code and data end
# inside slot A. The build must fail in the check and leave no image.
math(EXPR marv_planted_flash_bytes "(${PICO_FLASH_SIZE_BYTES}) - ${MARV_CONFIG_SLOTS} * ${MARV_CONFIG_SLOT_BYTES}")
add_executable(flash_layout_planted EXCLUDE_FROM_ALL flash_layout_planted.cpp)
target_compile_definitions(flash_layout_planted PRIVATE MARV_PLANTED_FLASH_BYTES=${marv_planted_flash_bytes})
target_link_libraries(flash_layout_planted PRIVATE pico_runtime)
marv_apply_flags(flash_layout_planted FIRMWARE)
marv_check_flash_layout(flash_layout_planted)
