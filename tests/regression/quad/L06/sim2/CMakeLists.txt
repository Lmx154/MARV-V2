# L6 stage (a), T1 SIM-2 with noise (quad spec section 4 L6 pass bar (a)): the run driver sim/l6_noise_run (adapter +
# marv_plant + IMU + the l4_rate_scripted SIL) is run in separate processes; same seed and corner give byte-identical logs
# at corners -1, 0, +1, and the controls (seed + 1, corner -1 against +1) differ where they must. The test only starts the
# driver binary, so it links no library but the SHA-256 (sim/common) and log-layout headers it includes.
add_executable(unit_l6_sim2 sim2_test.cpp)
target_include_directories(unit_l6_sim2 PRIVATE "${PROJECT_SOURCE_DIR}/sim/l6_noise_run")
target_compile_definitions(unit_l6_sim2 PRIVATE "MARV_L6_NOISE_RUN=\"$<TARGET_FILE:l6_noise_run>\"")
add_dependencies(unit_l6_sim2 l6_noise_run)
target_link_libraries(unit_l6_sim2 PRIVATE marv_sim_sha256 GTest::gtest_main)
marv_apply_flags(unit_l6_sim2)
marv_frozen_suite(unit_l6_sim2)
