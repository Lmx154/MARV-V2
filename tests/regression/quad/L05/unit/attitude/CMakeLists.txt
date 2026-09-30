# The L5 attitude law, T1 (decision 0006 C): structure, the compensated Jacobian, agreement with a double PX4
# re-implementation (px4_oracle.hpp), the singular set, the clamp, faults, the configuration and the period check.
add_executable(unit_l5_attitude attitude_test.cpp)
target_link_libraries(unit_l5_attitude PRIVATE marv_attitude marv_types marv_hal_sim GTest::gtest_main)
target_include_directories(unit_l5_attitude PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
marv_apply_flags(unit_l5_attitude)
marv_frozen_suite(unit_l5_attitude)
