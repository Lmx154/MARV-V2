# Invariant L-10 of the log file's life (onboard logger unit U4-5; serena memory quad-m4/architect-logger-u4-2026-10-20), host only
# (added from the top-level CMakeLists.txt): the logger and marv::log::LifecycleStorage (marv/log/file_life.hpp) on an in-memory
# card with a directory. It reads the parameter set marv_params_fc3v2_log and the shared helpers of the logger tests
# (../unit/log_test_util.hpp, included, not changed); link order as unit_log_logger.
add_executable(unit_log_file_life file_life_test.cpp)
target_include_directories(unit_log_file_life PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../unit")
target_link_libraries(unit_log_file_life PRIVATE
  marv_log marv_params_fc3v2_log_runtime marv_params_fc3v2_log_generated marv_types marv_hal_sim GTest::gtest_main)
marv_apply_flags(unit_log_file_life)
marv_frozen_suite(unit_log_file_life)
