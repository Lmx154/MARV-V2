add_executable(unit_smoke smoke_test.cpp)
target_link_libraries(unit_smoke PRIVATE GTest::gtest_main)
marv_apply_flags(unit_smoke)
gtest_discover_tests(unit_smoke PROPERTIES TIMEOUT ${MARV_PER_PUSH_CHECK_TIME_MAX})
