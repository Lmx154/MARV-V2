# The T2 bench-log replay of the attitude estimator (M3 P3b, decision 0042 item 2): replay.hpp parses a bench IMU log
# (fw/hal/target imu_log_format.hpp), replays it through the firmware's Mahony filter with quad_fc's flag rule and runs the
# still-segment check (a) and the gyro-path check (c'), each with its control; replay_test.cpp proves them on a synthetic
# known-answer log and on the rejected logs. The gains, the health bounds, the tick period, the stamp tolerance and
# attitude_tilt_error_max are read from the FC3v2 parameter set (marv_params_fc3v2), as the composition reads them.
# marv_params_fc3v2_runtime is listed first so its param_get is the one linked (see L09/unit). A committed real log is one
# more entry of kRealLogs in replay_test.cpp, found under MARV_L07_RESULTS_DIR.
add_executable(unit_l07_replay replay_test.cpp)
target_include_directories(unit_l07_replay PRIVATE "${PROJECT_SOURCE_DIR}/fw/hal/target/include")
target_compile_definitions(unit_l07_replay PRIVATE
  MARV_L07_RESULTS_DIR="${PROJECT_SOURCE_DIR}/tests/regression/quad/L07/results")
target_link_libraries(unit_l07_replay PRIVATE
  marv_params_fc3v2_runtime marv_params_fc3v2_generated marv_estimator marv_types marv_prim marv_hal_sim GTest::gtest_main)
marv_apply_flags(unit_l07_replay)
marv_frozen_suite(unit_l07_replay)
