# T1 of the truth-vertical entry (marv_truth_vertical_set, decision 0020 section 1, item c): every status code, the
# valid-bit rule, replace-on-second-set and the hand-over to the composition's vertical_input. The SIL library is built
# with TRUTH_VERTICAL and a test-only composition under composition/ that defines marv::composition::vertical_input and
# reports what it was handed on the motor outputs. marv_sil_init succeeds once per process, so every test runs in a
# forked child (truth_vertical_test.cpp).
add_library(marv_l11_truth_vertical_composition STATIC composition/composition.cpp)
target_include_directories(marv_l11_truth_vertical_composition PUBLIC composition/include)
target_link_libraries(marv_l11_truth_vertical_composition PUBLIC marv_hal marv_types)
marv_apply_flags(marv_l11_truth_vertical_composition FIRMWARE)
marv_add_sil_library(marv_sil_l11_truth_vertical_test
  COMPOSITION marv_l11_truth_vertical_composition PARAMS marv_params_l0 TRUTH_VERTICAL)

add_executable(unit_truth_vertical truth_vertical_test.cpp)
target_include_directories(unit_truth_vertical PRIVATE composition/include)
target_link_libraries(unit_truth_vertical PRIVATE
  marv_sil_l11_truth_vertical_test marv_types marv_params_l0_generated GTest::gtest_main)
marv_apply_flags(unit_truth_vertical)
marv_frozen_suite(unit_truth_vertical)
