# L6 stage (b) (decision 0013), the firmware gyro chain, T1: the float coefficients against a double implementation of
# the design formula (coeff_test.cpp); the steady-state response at f0, f0 (1 +- eps), the crossover band and the rate
# loop's Nyquist, with the negative controls Q x 1.1, f0 off by one telemetry step, harmonic index + 1 and rev/s for
# rad/s (response_test.cpp); the fault injections and the controls motors permuted and an unguarded notch above Nyquist
# (fault_test.cpp). The tick period is read from the product parameter set; every other number is a labelled test value.
add_executable(unit_l6_gyro_chain coeff_test.cpp response_test.cpp fault_test.cpp)
target_link_libraries(unit_l6_gyro_chain PRIVATE
  marv_gyro_chain marv_types marv_prim marv_params_runtime marv_hal_sim GTest::gtest_main)
marv_apply_flags(unit_l6_gyro_chain)
marv_frozen_suite(unit_l6_gyro_chain)
