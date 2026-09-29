add_executable(unit_prim_gravity gravity_test.cpp)
target_link_libraries(unit_prim_gravity PRIVATE marv_prim GTest::gtest_main)
target_compile_definitions(unit_prim_gravity PRIVATE
  MARV_GRAVITY_GOLDEN_CSV="${CMAKE_CURRENT_SOURCE_DIR}/gravity_golden.csv")
marv_apply_flags(unit_prim_gravity)
marv_frozen_suite(unit_prim_gravity)
