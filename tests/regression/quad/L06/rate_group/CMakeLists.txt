# L6 stage (c), commit 3 (decision 0014, "Wiring"): the shared rate-group step (fw/rate_group) in the compositions.
#
# unit_l6_rate_group_fault: the gyro chain leaves the rate loop's fault semantics as they were before it (lead decision):
# the l4_rate_scripted composition linked statically and ticked through hal_sim, the shared step, and a test-local
# reproduction of the path without the chain. The composition's parameter set is linked first, as unit_l4_composition
# does, so its param_get is the one linked (the product ids are a prefix of it).
add_executable(unit_l6_rate_group_fault fault_test.cpp)
target_link_libraries(unit_l6_rate_group_fault PRIVATE
  marv_params_l4_rate_scripted_runtime marv_params_l4_rate_scripted_generated
  marv_composition_l4_rate_scripted marv_rate_group marv_gyro_chain marv_rate marv_mixer marv_hal_sim marv_types
  marv_prim GTest::gtest_main)
marv_apply_flags(unit_l6_rate_group_fault)
marv_frozen_suite(unit_l6_rate_group_fault)

# unit_l6_replay_parity: the acro replay tool (tests/regression/quad/L04/replay, l4_acro_replay) and the l4_rate_scripted
# composition through the SIL C ABI agree bit for bit on a shared input. The test starts the replay binary; it links the
# SIL library and the composition's parameter set (parameter names and the hover thrust), the set's runtime first.
add_executable(unit_l6_replay_parity replay_parity_test.cpp)
target_compile_definitions(unit_l6_replay_parity PRIVATE "MARV_L4_ACRO_REPLAY=\"$<TARGET_FILE:l4_acro_replay>\"")
add_dependencies(unit_l6_replay_parity l4_acro_replay)
target_link_libraries(unit_l6_replay_parity PRIVATE
  marv_sil_l4_rate_scripted marv_params_l4_rate_scripted_runtime marv_params_l4_rate_scripted_generated
  marv_hal_sim marv_types marv_prim GTest::gtest_main)
marv_apply_flags(unit_l6_replay_parity)
marv_frozen_suite(unit_l6_replay_parity)
