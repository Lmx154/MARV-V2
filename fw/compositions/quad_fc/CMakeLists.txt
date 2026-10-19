# The quad flight composition (quad spec section 4.0 M1, decision 0027): the gyro chain, the rate loop and the mixer
# through the shared rate-group step with a zero rate setpoint and zero collective thrust, and the motor output gate in
# front of the actuator write. Static library on host and m33; the m33 image links it. M1 has no SIL library: the
# composition is driven through hal_sim directly by tests/regression/quad/L09.
add_library(marv_composition_quad_fc STATIC src/quad_fc.cpp)
target_include_directories(marv_composition_quad_fc PUBLIC include)
target_link_libraries(marv_composition_quad_fc
  PUBLIC marv_hal marv_types marv_prim marv_arming marv_params
  PRIVATE marv_attitude marv_estimator marv_sched marv_rate_group marv_rate marv_mixer marv_stick_rates marv_dshot marv_crsf_decode marv_params_fc3v2_runtime)
marv_apply_flags(marv_composition_quad_fc FIRMWARE)

# The same composition with the ESKF stepped beside Mahony and replacing it for the attitude law, the upright check and the
# published tilt (M4, decision 0055; host only: the board image and marv_sil_quad_fc do not contain it). The one source file,
# compiled with MARV_QUAD_FC_ESKF and against the FC3v2 set with the ESKF register (marv_params_fc3v2_eskf); the SIL library
# marv_sil_quad_fc_eskf of sim/quad_fc is built from it.
if(MARV_TARGET STREQUAL "host")
  add_library(marv_composition_quad_fc_eskf STATIC src/quad_fc.cpp)
  target_compile_definitions(marv_composition_quad_fc_eskf PRIVATE MARV_QUAD_FC_ESKF)
  target_include_directories(marv_composition_quad_fc_eskf PUBLIC include)
  target_link_libraries(marv_composition_quad_fc_eskf
    PUBLIC marv_hal marv_types marv_prim marv_arming marv_params
    PRIVATE marv_attitude marv_estimator marv_sched marv_rate_group marv_rate marv_mixer marv_stick_rates marv_dshot marv_crsf_decode marv_params_fc3v2_eskf_runtime)
  marv_apply_flags(marv_composition_quad_fc_eskf FIRMWARE)
endif()

# The same composition with the bench motor-step profile in place of the mixer's output while Armed (unit U3 BENCH-PROFILE,
# architect ruling 2026-10-18 item 8): the one source file, compiled with MARV_QUAD_FC_BENCH_PROFILE and against the FC3v2 set
# with the bench-profile register (marv_params_fc3v2_bench_profile). Host and m33: only the bench image marv_fc3v2_bench_profile
# (fw/hal/target) and the L09 bench-profile tests link it; no flight image may (invariant L-6, the post-link check
# tools/ci/check_bench_profile.py).
add_library(marv_composition_quad_fc_bench_profile STATIC src/quad_fc.cpp)
target_compile_definitions(marv_composition_quad_fc_bench_profile PRIVATE MARV_QUAD_FC_BENCH_PROFILE)
target_include_directories(marv_composition_quad_fc_bench_profile PUBLIC include)
target_link_libraries(marv_composition_quad_fc_bench_profile
  PUBLIC marv_hal marv_types marv_prim marv_arming marv_params
  PRIVATE marv_attitude marv_estimator marv_sched marv_rate_group marv_rate marv_mixer marv_stick_rates marv_dshot marv_crsf_decode marv_params_fc3v2_bench_profile_runtime)
marv_apply_flags(marv_composition_quad_fc_bench_profile FIRMWARE)
