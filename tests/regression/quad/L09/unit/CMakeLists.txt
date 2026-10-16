# The quad_fc composition driven through hal_sim, and beside it a rate-group step built as quad_fc builds it.
# marv_params_fc3v2_runtime is listed before the libraries that link the product set, so its param_get is the one
# linked (see sim/l4_rate_scripted/CMakeLists.txt).
add_executable(unit_quad_fc_gate quad_fc_gate_test.cpp)
target_link_libraries(unit_quad_fc_gate PRIVATE
  marv_composition_quad_fc marv_params_fc3v2_runtime marv_params_fc3v2_generated
  marv_rate_group marv_mixer marv_types marv_hal_sim GTest::gtest_main)
marv_apply_flags(unit_quad_fc_gate)
marv_frozen_suite(unit_quad_fc_gate)

# The FC3v2 IMU decode (fw/hal/target/decode, M1 P9): datasheet sensitivities, the mounting map, the sample flags and
# the stamping rule, on the host.
add_executable(unit_lsm6dsv_decode lsm6dsv_decode_test.cpp)
target_link_libraries(unit_lsm6dsv_decode PRIVATE marv_lsm6dsv_decode marv_types GTest::gtest_main)
marv_apply_flags(unit_lsm6dsv_decode)
marv_frozen_suite(unit_lsm6dsv_decode)
