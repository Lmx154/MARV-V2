# The L4 T3 suite (quad spec 4 L4, decision 0005 "T3"): the firmware rate loop, configured from the committed input
# fixture (reference/rate_t3_inputs.txt), closed with the design plant in double, against the committed oracle.
add_executable(t3_l4_rate t3_test.cpp)
target_link_libraries(t3_l4_rate PRIVATE
  marv_rate marv_mixer marv_types marv_params_runtime marv_hal_sim GTest::gtest_main)
target_compile_definitions(t3_l4_rate PRIVATE MARV_L4_T3_REFERENCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/reference")
marv_apply_flags(t3_l4_rate)
marv_frozen_suite(t3_l4_rate)
