# The L11 T3 golden (quad spec 4 L11, decision 0020 section 5 "T3 golden (core)"): the vertical law in float, configured from the
# committed input fixture (reference/vertical_t3_inputs.txt), closed with the design plant in double at tick resolution, against
# the committed oracle. The fixture is checked against the built defaults of marv_params_l11_vertical_scripted.
add_executable(t3_l11_vertical t3_test.cpp)
target_link_libraries(t3_l11_vertical PRIVATE
  marv_vertical marv_mixer marv_types marv_params_l11_vertical_scripted_runtime marv_params_l11_vertical_scripted_generated
  marv_hal_sim GTest::gtest_main)
marv_reference_set(quad/L11/t3 l11_t3_dir l11_t3_fixture)
target_compile_definitions(t3_l11_vertical PRIVATE MARV_L11_T3_REFERENCE_DIR="${l11_t3_dir}")
marv_apply_flags(t3_l11_vertical)
marv_frozen_suite(t3_l11_vertical FIXTURES_REQUIRED ${l11_t3_fixture})
