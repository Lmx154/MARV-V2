# The post-link flash-layout check of the m33 images (decision 0053 Q3, ground-segment unit 14).
#
# marv_check_flash_layout(<target>)
#   After every link of the m33 <target>, tools/ci/check_flash_layout.py reads the linked ELF and fails the build, removing
#   the file, if the image's flash bytes reach the config slots: kConfigSlots (A and B, marv/hal/storage.hpp) regions of
#   MARV_CONFIG_SLOT_BYTES (fw/hal/CMakeLists.txt, the design-budget entry config_slot_bytes) at the top of the
#   PICO_FLASH_SIZE_BYTES flash (the board header, boards/madflight_fc3v2_revc.h). The board's storage
#   (fw/hal/target/src/storage_flash.cpp) puts the slots there from the same two values. Negative control:
#   tests/regression/quad/L09/flash_layout (flash_layout_planted), ci/run_ci.sh flash_layout_control.
set(MARV_CONFIG_SLOTS 2)  # slots A and B (marv/hal/storage.hpp ConfigSlot)

function(marv_check_flash_layout target)
  if(NOT DEFINED PICO_FLASH_SIZE_BYTES)
    message(FATAL_ERROR "marv_check_flash_layout(${target}): PICO_FLASH_SIZE_BYTES is not set by the board header")
  endif()
  math(EXPR flash_bytes "${PICO_FLASH_SIZE_BYTES}")
  # MARV_PYTHON is the uv environment's Python (fw/params/CMakeLists.txt). The image's directory (fw/hal/target) is configured
  # before fw/params on a first configure, so the same default is taken while the cache does not hold it yet.
  set(python "${MARV_PYTHON}")
  if(python STREQUAL "")
    set(python "${PROJECT_SOURCE_DIR}/.venv/bin/python")
  endif()
  add_custom_command(TARGET ${target} POST_BUILD
    COMMAND "${python}" "${PROJECT_SOURCE_DIR}/tools/ci/check_flash_layout.py" --elf "$<TARGET_FILE:${target}>"
            --flash-bytes ${flash_bytes} --slot-bytes ${MARV_CONFIG_SLOT_BYTES} --slots ${MARV_CONFIG_SLOTS}
            --delete-on-failure
    COMMENT "Checking that ${target} ends below the config slots"
    VERBATIM)
endfunction()
