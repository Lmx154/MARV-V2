# T1 of the truth-state entry (marv_truth_state_set, decision 0006 section B): every status code, replace-on-second-set and
# the hand-over to the composition's attitude_input. The SIL library is built with TRUTH_STATE and a test-only
# composition under composition/ that defines marv::composition::attitude_input and reports what it was handed on the
# motor outputs. marv_sil_init succeeds once per process, so every test runs in a forked child (truth_state_test.cpp).
add_library(marv_l05_truth_composition STATIC composition/composition.cpp)
target_include_directories(marv_l05_truth_composition PUBLIC composition/include)
target_link_libraries(marv_l05_truth_composition PUBLIC marv_hal marv_types)
marv_apply_flags(marv_l05_truth_composition FIRMWARE)
marv_add_sil_library(marv_sil_l05_truth_test COMPOSITION marv_l05_truth_composition PARAMS marv_params_l0 TRUTH_STATE)

add_executable(unit_truth_state truth_state_test.cpp)
target_include_directories(unit_truth_state PRIVATE composition/include)
target_link_libraries(unit_truth_state PRIVATE
  marv_sil_l05_truth_test marv_types marv_params_l0_generated GTest::gtest_main)
marv_apply_flags(unit_truth_state)
marv_frozen_suite(unit_truth_state)
