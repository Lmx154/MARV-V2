# The L5 T3 suite (quad spec 4 L5, decision 0006 F "T3 step" and "T3 envelopes"; L6 stage (c), decision 0014): angle mode,
# the attitude law, the gyro chain and the rate loop in bypass with its filtered D term, configured from the committed input
# fixture (reference/attitude_t3_inputs.txt), closed with the design plant in double at tick resolution, against the
# committed oracle.
add_executable(t3_l5_attitude t3_test.cpp)
target_link_libraries(t3_l5_attitude PRIVATE
  marv_attitude marv_gyro_chain marv_rate marv_mixer marv_types marv_params_runtime marv_hal_sim GTest::gtest_main)
marv_reference_set(quad/L05/t3 l5_t3_dir l5_t3_fixture)
target_compile_definitions(t3_l5_attitude PRIVATE MARV_L5_T3_REFERENCE_DIR="${l5_t3_dir}")
marv_apply_flags(t3_l5_attitude)
marv_frozen_suite(t3_l5_attitude FIXTURES_REQUIRED ${l5_t3_fixture})
