# The quad_fc composition driven through hal_sim, and beside it a rate-group step built as quad_fc builds it.
# marv_params_fc3v2_runtime is listed before the libraries that link the product set, so its param_get is the one
# linked (see sim/l4_rate_scripted/CMakeLists.txt).
add_executable(unit_quad_fc_gate quad_fc_gate_test.cpp)
target_link_libraries(unit_quad_fc_gate PRIVATE
  marv_composition_quad_fc marv_params_fc3v2_runtime marv_params_fc3v2_generated
  marv_attitude marv_estimator marv_rate_group marv_mixer marv_stick_rates marv_types marv_hal_sim GTest::gtest_main)
marv_apply_flags(unit_quad_fc_gate)
marv_frozen_suite(unit_quad_fc_gate)

# The FC3v2 IMU decode (fw/hal/target/decode, M1 P9): datasheet sensitivities, the mounting map, the sample flags and
# the stamping rule, on the host.
add_executable(unit_lsm6dsv_decode lsm6dsv_decode_test.cpp)
target_link_libraries(unit_lsm6dsv_decode PRIVATE marv_lsm6dsv_decode marv_types GTest::gtest_main)
marv_apply_flags(unit_lsm6dsv_decode)
marv_frozen_suite(unit_lsm6dsv_decode)

# The FC3v2 DShot600 output logic (fw/hal/target/decode, M1 P10): frame encoder, pad permutation, word packing, the PIO
# timing and the panic loop's stop frames, on the host.
add_executable(unit_dshot dshot_test.cpp)
target_link_libraries(unit_dshot PRIVATE marv_dshot marv_types GTest::gtest_main)
marv_apply_flags(unit_dshot)
marv_frozen_suite(unit_dshot)

# The FC3v2 console line parser (fw/hal/target/decode, M1 P11, decision 0038): the motor test command with its PROPS-OFF
# unlock token, stop, and the refusals, on the host.
add_executable(unit_console console_test.cpp)
target_link_libraries(unit_console PRIVATE marv_console_decode marv_types GTest::gtest_main)
marv_apply_flags(unit_console)
marv_frozen_suite(unit_console)

# The quad_fc composition as a SIL library (sim/quad_fc, HIL S2c, decision 0046 item 4): the staged ManualControl reaches
# the composition through the C ABI, and the staged frame stamp drives the failsafe. The library is linked, init succeeds once
# per process, so every test that initialises runs in a forked child. The test reads the set's default table (the library's
# set, marv_params_fc3v2) for the tick rational and the failsafe times.
add_executable(unit_sil_quad_fc sil_quad_fc_test.cpp)
target_link_libraries(unit_sil_quad_fc PRIVATE marv_sil_quad_fc marv_params_fc3v2_runtime marv_params_fc3v2_generated marv_types marv_hal_sim GTest::gtest_main)
marv_apply_flags(unit_sil_quad_fc)
marv_frozen_suite(unit_sil_quad_fc)

# marv_sil_quad_fc must resolve the FC3v2 parameter table, not the product set's that marv_attitude, marv_rate and marv_mixer
# also link (the link order decides): the size of the linked marv::generated::kParamDefaults. The control swaps the sets: the
# library's table is not the product set's, so the check must fail.
add_test(NAME sil_quad_fc_param_table
  COMMAND "${CMAKE_COMMAND}" -DLIB=$<TARGET_FILE:marv_sil_quad_fc> -DNM=${CMAKE_NM}
          -DEXPECT=$<TARGET_FILE:marv_params_fc3v2_generated> -DOTHER=$<TARGET_FILE:marv_params_generated>
          -P "${CMAKE_CURRENT_SOURCE_DIR}/check_param_table.cmake")
add_test(NAME sil_quad_fc_param_table_negative_control
  COMMAND "${CMAKE_COMMAND}" -DLIB=$<TARGET_FILE:marv_sil_quad_fc> -DNM=${CMAKE_NM}
          -DEXPECT=$<TARGET_FILE:marv_params_generated> -DOTHER=$<TARGET_FILE:marv_params_fc3v2_generated>
          -P "${CMAKE_CURRENT_SOURCE_DIR}/check_param_table.cmake")
set_tests_properties(sil_quad_fc_param_table sil_quad_fc_param_table_negative_control PROPERTIES LABELS frozen)
set_tests_properties(sil_quad_fc_param_table_negative_control PROPERTIES WILL_FAIL TRUE)
