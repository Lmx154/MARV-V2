# The quad_fc composition driven through hal_sim. marv_params_fc3v2_runtime is listed before the libraries that link the
# product set, so its param_get is the one linked (see sim/l4_rate_scripted/CMakeLists.txt).
add_executable(unit_quad_fc_gate quad_fc_gate_test.cpp)
target_link_libraries(unit_quad_fc_gate PRIVATE
  marv_composition_quad_fc marv_params_fc3v2_runtime marv_params_fc3v2_generated
  marv_mixer marv_types marv_hal_sim GTest::gtest_main)
marv_apply_flags(unit_quad_fc_gate)
marv_frozen_suite(unit_quad_fc_gate)
