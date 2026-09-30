# marv_plant initial rotor speed (marv_plant_config::initial_omega_rad_s), T1 (decision 0007): honoured, zero equals the
# default bit for bit, out-of-range refused. The L1 plant fixture is included from its frozen path, not copied.
add_executable(unit_l5_plant_initial_rate initial_rate_test.cpp)
target_include_directories(unit_l5_plant_initial_rate PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../../../L01/unit/plant")
target_link_libraries(unit_l5_plant_initial_rate PRIVATE marv_plant marv_prim GTest::gtest_main)
marv_apply_flags(unit_l5_plant_initial_rate)
marv_frozen_suite(unit_l5_plant_initial_rate)
