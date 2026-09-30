# The truth-attitude source (sim/gz, marv::truth, decision 0006 section B): the marv_truth_state it fills from
# marv_plant_body, its hold over a host step's m ticks, and the hand-over before each marv_sil_tick, seen through the
# motor outputs of the truth-state test composition (../truth_state). The adapter has no Gazebo header, so the test builds
# without gz. marv_sil_init succeeds once per process, so the SIL cases run in a forked child (truth_attitude_test.cpp).
add_executable(unit_truth_attitude truth_attitude_test.cpp)
target_include_directories(unit_truth_attitude PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../truth_state/composition/include")
target_link_libraries(unit_truth_attitude PRIVATE
  marv_gz_truth_attitude marv_gz_adapter marv_plant marv_sil_l05_truth_test marv_types
  marv_params_l0_generated GTest::gtest_main)
marv_apply_flags(unit_truth_attitude)
marv_frozen_suite(unit_truth_attitude)
