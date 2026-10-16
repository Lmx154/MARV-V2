# The quad flight composition (quad spec section 4.0 M1, decision 0027): the gyro chain, the rate loop and the mixer
# through the shared rate-group step with a zero rate setpoint and zero collective thrust, and the motor output gate in
# front of the actuator write. Static library on host and m33; the m33 image links it. M1 has no SIL library: the
# composition is driven through hal_sim directly by tests/regression/quad/L09.
add_library(marv_composition_quad_fc STATIC src/quad_fc.cpp)
target_include_directories(marv_composition_quad_fc PUBLIC include)
target_link_libraries(marv_composition_quad_fc
  PUBLIC marv_hal marv_types marv_prim marv_arming
  PRIVATE marv_sched marv_rate_group marv_rate marv_mixer marv_stick_rates marv_params_fc3v2_runtime)
marv_apply_flags(marv_composition_quad_fc FIRMWARE)
