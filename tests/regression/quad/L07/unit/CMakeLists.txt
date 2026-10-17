# The Mahony attitude filter (fw/estimator, M3 P1): level and tilted at rest, constant rotation without correction, gyro-bias
# convergence from the gains, and the skipped correction for a zero or non-finite accelerometer vector, on the host.
add_executable(unit_mahony mahony_test.cpp)
target_link_libraries(unit_mahony PRIVATE marv_estimator GTest::gtest_main)
marv_apply_flags(unit_mahony)
marv_frozen_suite(unit_mahony)
