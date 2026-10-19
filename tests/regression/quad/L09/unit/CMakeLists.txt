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

# The config store (fw/store, decision 0053, ground-segment unit 4b) on the host: SHA-256 known answers, the .mcfg reader and writer
# against the factory file the build writes with tools/config/mcfg.py (the file is passed in MARV_FACTORY_MCFG), the boot policy
# (invariant B-1, one test per row), the boot's apply step, revert and reset to factory, the staging refusals and commands, and
# invariant P-1. The store keeps its state in the process, so every test that boots runs in a forked child on its own temporary
# hal_sim run directory. Link order as unit_quad_fc_gate: the FC3v2 parameter set before the libraries that link the product set.
add_executable(unit_store store_test.cpp)
target_link_libraries(unit_store PRIVATE
  marv_store marv_composition_quad_fc marv_params_fc3v2_runtime marv_params_fc3v2_generated
  marv_attitude marv_estimator marv_rate_group marv_mixer marv_stick_rates marv_types marv_hal_sim GTest::gtest_main)
target_compile_definitions(unit_store PRIVATE
  MARV_FACTORY_MCFG="${CMAKE_BINARY_DIR}/generated/marv_params_fc3v2/marv_params_fc3v2.mcfg")
marv_apply_flags(unit_store)
marv_frozen_suite(unit_store)

# The onboard SD flight logger's core (fw/log, unit U1 LOG-CORE, architect ruling 2026-10-18): the formats, the ring (L-1: the
# writer never waits) and its budget (L-5: ring >= rate x the worst SD stall, with negative controls), the sector framer, the header
# and the logger state machine against a scriptable card (L-3 faults, L-9 no metadata while armed), and the round trip. The tests
# read the parameter set marv_params_fc3v2_log (the FC3v2 set plus fw/log/params/log_register.yaml); link order as unit_quad_fc_gate.
find_package(Threads REQUIRED)
add_executable(unit_log_ring log_ring_test.cpp)
target_link_libraries(unit_log_ring PRIVATE
  marv_log marv_params_fc3v2_log_runtime marv_params_fc3v2_log_generated marv_types marv_hal_sim GTest::gtest_main Threads::Threads)
marv_apply_flags(unit_log_ring)
marv_frozen_suite(unit_log_ring)

add_executable(unit_log_logger log_logger_test.cpp)
target_link_libraries(unit_log_logger PRIVATE
  marv_log marv_params_fc3v2_log_runtime marv_params_fc3v2_log_generated marv_types marv_hal_sim GTest::gtest_main)
marv_apply_flags(unit_log_logger)
marv_frozen_suite(unit_log_logger)

# The C++ writer against the Python decoder (tools/log/mlog.py): the first test writes a log and the stream it was given
# (unit_log_logger, LogRoundTrip), the second decodes the file and requires equality (tools/log/roundtrip_check.py).
set(log_roundtrip_dir "${CMAKE_CURRENT_BINARY_DIR}/log_roundtrip")
add_test(NAME log_roundtrip_write
  COMMAND "${CMAKE_COMMAND}" -E make_directory "${log_roundtrip_dir}")
add_test(NAME log_roundtrip_encode COMMAND unit_log_logger --gtest_filter=LogRoundTrip.*)
set_tests_properties(log_roundtrip_encode PROPERTIES
  ENVIRONMENT "MARV_LOG_ROUNDTRIP_DIR=${log_roundtrip_dir}"
  FIXTURES_SETUP log_roundtrip_files
  DEPENDS log_roundtrip_write)
add_test(NAME log_roundtrip_decode
  COMMAND "${MARV_PYTHON}" "${PROJECT_SOURCE_DIR}/tools/log/roundtrip_check.py" "${log_roundtrip_dir}")
set_tests_properties(log_roundtrip_decode PROPERTIES FIXTURES_REQUIRED log_roundtrip_files)
set_tests_properties(log_roundtrip_write log_roundtrip_encode log_roundtrip_decode PROPERTIES LABELS frozen)

# The bench motor-step profile (unit U3 BENCH-PROFILE, architect ruling 2026-10-18 item 8; invariant L-7): the composition variant
# marv_composition_quad_fc_bench_profile through hal_sim on marv_params_fc3v2_bench_profile (the FC3v2 set plus
# fw/compositions/quad_fc/params/bench_profile_register.yaml): the sequence to the tick, every kill at every step, once per arm,
# the arming states, the stamp end, validate and init. Link order as unit_quad_fc_gate.
add_executable(unit_quad_fc_bench_profile quad_fc_bench_profile_test.cpp)
target_link_libraries(unit_quad_fc_bench_profile PRIVATE
  marv_composition_quad_fc_bench_profile marv_params_fc3v2_bench_profile_runtime marv_params_fc3v2_bench_profile_generated
  marv_types marv_hal_sim GTest::gtest_main)
marv_apply_flags(unit_quad_fc_bench_profile)
marv_frozen_suite(unit_quad_fc_bench_profile)

# The write clamp of the bench profile (L-7: the DShot written never exceeds the fixed bench cap, whatever the configuration
# holds): the same test source against the library with the seam that overwrites the configured levels past validate
# (MARV_QUAD_FC_BENCH_PROFILE_FAULT_INJECT). Only the clamp tests run here; the others run in unit_quad_fc_bench_profile.
add_executable(unit_quad_fc_bench_profile_clamp quad_fc_bench_profile_test.cpp)
target_link_libraries(unit_quad_fc_bench_profile_clamp PRIVATE
  marv_composition_quad_fc_bench_profile_inject marv_params_fc3v2_bench_profile_runtime marv_params_fc3v2_bench_profile_generated
  marv_types marv_hal_sim GTest::gtest_main)
marv_apply_flags(unit_quad_fc_bench_profile_clamp)
marv_frozen_suite(unit_quad_fc_bench_profile_clamp TEST_FILTER BenchProfileClampTest.*)

# Invariant L-6 on the host (the m33 images run the same check after every link, fw/hal/target/CMakeLists.txt): no flight
# composition holds a symbol of the bench profile (the flight composition's archive and the SIL library of the flight
# composition), and the bench variant's archive does (the control inside the check). The negative control runs the check on
# the bench archive as a flight file: it must report the violation. bench_profile_param_ids: the bench set carries every FC3v2
# record unchanged (the libraries the variant links are compiled against those ids) and adds only bench_profile_* ids (the
# marker the symbol check looks for).
set(bench_profile_check "${PROJECT_SOURCE_DIR}/tools/ci/check_bench_profile.py")
add_test(NAME bench_profile_symbols
  COMMAND "${MARV_PYTHON}" "${bench_profile_check}" --nm "${CMAKE_NM}"
          --forbid $<TARGET_FILE:marv_composition_quad_fc> --forbid $<TARGET_FILE:marv_sil_quad_fc>
          --require $<TARGET_FILE:marv_composition_quad_fc_bench_profile>)
add_test(NAME bench_profile_symbols_negative_control
  COMMAND "${MARV_PYTHON}" "${bench_profile_check}" --nm "${CMAKE_NM}"
          --forbid $<TARGET_FILE:marv_composition_quad_fc_bench_profile>)
set_tests_properties(bench_profile_symbols_negative_control PROPERTIES PASS_REGULAR_EXPRESSION "L6-SYMBOL: ")
add_test(NAME bench_profile_param_ids
  COMMAND "${MARV_PYTHON}" "${PROJECT_SOURCE_DIR}/tools/ci/check_param_ids.py"
          $<TARGET_PROPERTY:marv_params_fc3v2_generated,MARV_PARAM_MANIFEST>
          $<TARGET_PROPERTY:marv_params_fc3v2_bench_profile_generated,MARV_PARAM_MANIFEST> bench_profile_)
set_tests_properties(bench_profile_symbols bench_profile_symbols_negative_control bench_profile_param_ids
  PROPERTIES LABELS frozen)
