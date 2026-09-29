# L0 T2 replay: the fixed L0 composition through the SIL entry, hashed against a committed golden.
add_executable(replay_l00 l0_trace_test.cpp)
target_link_libraries(replay_l00 PRIVATE marv_null_plant GTest::gtest_main)
target_compile_definitions(replay_l00 PRIVATE
  MARV_L0_RUNNER="$<TARGET_FILE:marv_null_plant_run>"
  MARV_L0_GOLDEN="${CMAKE_CURRENT_SOURCE_DIR}/golden_l0_trace.txt")
add_dependencies(replay_l00 marv_null_plant_run)
marv_apply_flags(replay_l00)
marv_frozen_suite(replay_l00)

# Golden regeneration. ONLY run inside the marv-ci image (core C-8: goldens are regenerated only there); CI never
# regenerates, and the target exists only when configured with -DMARV_ALLOW_GOLDEN_REGEN=ON:
#   cmake --preset host-release -DMARV_ALLOW_GOLDEN_REGEN=ON && cmake --build --preset host-release --target regen_l0_golden
# then commit tests/regression/quad/L00/replay/golden_l0_trace.txt.
option(MARV_ALLOW_GOLDEN_REGEN "Define the regen_l0_golden target (only inside the marv-ci image)" OFF)
if(MARV_ALLOW_GOLDEN_REGEN)
  set(_l0_note "fixtures:")
  get_target_property(_l0_card_files marv_params_l0_generated MARV_PARAM_CARD_FILES)
  get_target_property(_l0_register_files marv_params_l0_generated MARV_PARAM_REGISTER_FILES)
  foreach(_f IN LISTS _l0_card_files _l0_register_files)
    file(SHA256 "${_f}" _sha)
    cmake_path(GET _f FILENAME _name)
    string(APPEND _l0_note " ${_name} sha256=${_sha}")
  endforeach()
  add_custom_target(regen_l0_golden
    COMMAND marv_null_plant_run --write-golden "${CMAKE_CURRENT_SOURCE_DIR}/golden_l0_trace.txt"
            --note "${_l0_note}"
    DEPENDS marv_null_plant_run
    COMMENT "Regenerating tests/regression/quad/L00/replay/golden_l0_trace.txt (only inside the marv-ci image)"
    VERBATIM)
endif()
