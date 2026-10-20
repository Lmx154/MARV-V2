# Invariant L-8 of the disarmed arena (onboard logger unit U4-2; serena memory quad-m4/architect-logger-u4-2026-10-20), host only
# (added from the top-level CMakeLists.txt): the arena lease of the config store (marv/store/arena.hpp) and the onboard logger on
# the arena (marv/log/arena_logger.hpp) against the store's activation on hal_sim's file storage. Every test that boots the
# store runs in a forked child, as unit_store does. Link order as unit_store: the FC3v2 parameter set before the libraries that
# link the product set.
add_executable(unit_arena_lease arena_lease_test.cpp)
target_link_libraries(unit_arena_lease PRIVATE
  marv_store marv_log marv_store_arena marv_params_fc3v2_runtime marv_params_fc3v2_generated marv_types marv_hal_sim
  GTest::gtest_main)
marv_apply_flags(unit_arena_lease)
marv_frozen_suite(unit_arena_lease)
