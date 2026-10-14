# T1 of the L11 vertical law (decision 0020 section 5): the law against hand-computed values, the sign, the anti-windup
# freeze rule (I4), the invalid-state fallback (I5), the configuration check, reset, and that the law never clamps.
add_executable(unit_l11_vertical vertical_test.cpp)
target_link_libraries(unit_l11_vertical PRIVATE marv_vertical marv_types GTest::gtest_main)
marv_apply_flags(unit_l11_vertical)
marv_frozen_suite(unit_l11_vertical)
