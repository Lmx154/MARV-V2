# The truth-gyro command source (sim/gz, marv::truth): the sample handed to the SIL is the float cast of the truth body
# rate. marv_sil_init succeeds once per process, so the SIL case runs in a forked child (truth_gyro_test.cpp).
add_executable(unit_truth_gyro truth_gyro_test.cpp)
target_link_libraries(unit_truth_gyro PRIVATE
  marv_gz_adapter marv_plant marv_sil_l2_open_loop marv_types
  marv_params_l2_open_loop_generated GTest::gtest_main)
marv_apply_flags(unit_truth_gyro)
marv_frozen_suite(unit_truth_gyro)
