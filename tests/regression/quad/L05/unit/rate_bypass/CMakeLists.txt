# The L4 rate-loop bypass (I-A1 to I-A4, decision 0006 "A"). The fixture and the acro golden live in reference/.
add_executable(unit_l5_rate_bypass rate_bypass_test.cpp)
target_link_libraries(unit_l5_rate_bypass PRIVATE
  marv_rate marv_mixer marv_types marv_params_runtime marv_hal_sim GTest::gtest_main)
target_compile_definitions(unit_l5_rate_bypass PRIVATE
  MARV_L5_RATE_BYPASS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/reference")
target_include_directories(unit_l5_rate_bypass PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
marv_apply_flags(unit_l5_rate_bypass)
marv_frozen_suite(unit_l5_rate_bypass)
