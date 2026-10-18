# The T2 bench-log replay of the attitude estimator (M3 P3b, decision 0042 item 2): replay.hpp parses a bench IMU log
# (fw/hal/target imu_log_format.hpp), replays it through the firmware's Mahony filter with quad_fc's flag rule and runs the
# still-segment check (a) and the gyro-path check (c'), each with its control; replay_test.cpp proves them on a synthetic
# known-answer log and on the rejected logs. The gains, the health bounds, the tick period, the stamp tolerance and
# attitude_tilt_error_max are read from the FC3v2 parameter set (marv_params_fc3v2), as the composition reads them.
# marv_params_fc3v2_runtime is listed first so its param_get is the one linked (see L09/unit). A committed real log is one
# more entry of kRealLogs in replay_test.cpp, found under MARV_L07_RESULTS_DIR.
# The committed logs are <name>.tar.xz in tests/regression/quad/L07/results/ (see its README.md); at configure time each is
# extracted into the build tree with file(ARCHIVE_EXTRACT) (no dependency beyond CMake) and checked against the sha256 in
# results/SHA256SUMS, which is the sha256 of the log as captured.
set(L07_RESULTS_SRC "${PROJECT_SOURCE_DIR}/tests/regression/quad/L07/results")
set(L07_RESULTS_DIR "${CMAKE_CURRENT_BINARY_DIR}/results")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${L07_RESULTS_SRC}/SHA256SUMS")
file(STRINGS "${L07_RESULTS_SRC}/SHA256SUMS" L07_LOG_SUMS)
foreach(line IN LISTS L07_LOG_SUMS)
  string(REGEX MATCH "^([0-9a-f]+)  (.+)$" matched "${line}")
  if(NOT matched)
    message(FATAL_ERROR "L07 results/SHA256SUMS: malformed line '${line}'")
  endif()
  set(want "${CMAKE_MATCH_1}")
  set(log "${CMAKE_MATCH_2}")
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${L07_RESULTS_SRC}/${log}.tar.xz")
  set(have "")
  if(EXISTS "${L07_RESULTS_DIR}/${log}")
    file(SHA256 "${L07_RESULTS_DIR}/${log}" have)
  endif()
  if(NOT have STREQUAL want)
    file(REMOVE "${L07_RESULTS_DIR}/${log}")
    file(ARCHIVE_EXTRACT INPUT "${L07_RESULTS_SRC}/${log}.tar.xz" DESTINATION "${L07_RESULTS_DIR}")
    file(SHA256 "${L07_RESULTS_DIR}/${log}" have)
  endif()
  if(NOT have STREQUAL want)
    message(FATAL_ERROR "L07 bench log ${log}: sha256 ${have} differs from results/SHA256SUMS (${want})")
  endif()
endforeach()
add_executable(unit_l07_replay replay_test.cpp)
target_include_directories(unit_l07_replay PRIVATE "${PROJECT_SOURCE_DIR}/fw/hal/target/include")
target_compile_definitions(unit_l07_replay PRIVATE
  MARV_L07_RESULTS_DIR="${L07_RESULTS_DIR}")
target_link_libraries(unit_l07_replay PRIVATE
  marv_params_fc3v2_runtime marv_params_fc3v2_generated marv_estimator marv_store marv_types marv_prim marv_hal_sim GTest::gtest_main)
marv_apply_flags(unit_l07_replay)
marv_frozen_suite(unit_l07_replay)
# Ground-segment unit 9 (decision 0053 G7 check 3): the committed-log predicate test and its two controls, labelled
# config-validation; tools/validate/validate_config.py runs it with MARV_VALIDATE_CONFIG naming the candidate config (replay_test.cpp
# replay_table), and without the variable it is the frozen test above.
add_test(NAME l07_replay_config_validation COMMAND unit_l07_replay --gtest_filter=L07Replay.CommittedBenchLogs)
set_tests_properties(l07_replay_config_validation PROPERTIES LABELS config-validation)
