# The L5 T3 suite (quad spec 4 L5, decision 0006 F "T3 step" and "T3 envelopes"): angle mode, the attitude law and the rate
# loop in bypass, configured from the committed input fixture (reference/attitude_t3_inputs.txt), closed with the design
# plant in double at tick resolution, against the committed oracle.
add_executable(t3_l5_attitude t3_test.cpp)
target_link_libraries(t3_l5_attitude PRIVATE
  marv_attitude marv_rate marv_mixer marv_types marv_params_runtime marv_hal_sim GTest::gtest_main)
target_compile_definitions(t3_l5_attitude PRIVATE MARV_L5_T3_REFERENCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/reference")
marv_apply_flags(t3_l5_attitude)
marv_frozen_suite(t3_l5_attitude)
