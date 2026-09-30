# The Gazebo adapter (sim/gz), host-free: frame maps, bit-exact plant equality, tick ordering and the real SIL.
# marv_sil_init succeeds once per process, so the SIL test runs in a forked child (adapter_test.cpp).
add_executable(unit_gz_adapter adapter_test.cpp)
target_link_libraries(unit_gz_adapter PRIVATE
  marv_gz_adapter marv_plant marv_plant_card marv_prim marv_sil_l2_open_loop marv_types
  marv_params_l2_open_loop_generated GTest::gtest_main)
marv_apply_flags(unit_gz_adapter)
marv_frozen_suite(unit_gz_adapter)
