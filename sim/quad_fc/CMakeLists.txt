# The quad flight composition as a SIL library (host only; HIL S2c, decision 0046 item 4): the SIL entry built from the
# quad_fc composition with the FC3v2 parameter set (marv_params_fc3v2, the set the flight image links), without the
# test-only truth-state entries, so the library exports marv_sil_* only (marv_sil.map). Its inputs are the IMU samples and
# the staged ManualControl (marv_sil_stage_manual_control); the tick period is the caller's through marv_sil_init (the
# board's, from tick_period_num_us / tick_period_den of the set). With STORAGE its table comes from the config store's boot
# step (decision 0053, ground-segment unit 4b), on the run directory marv_sil_storage_open names, else the compiled one.
# The fw/ targets below are built without PIC by default;
# the property is set here, on the existing targets, as sim/l7_attitude_estimated does.
set_target_properties(marv_attitude marv_rate_group marv_gyro_chain marv_rate marv_mixer marv_sched marv_estimator
  marv_stick_rates marv_arming marv_hal marv_prim marv_types
  PROPERTIES POSITION_INDEPENDENT_CODE ON)
marv_add_sil_library(marv_sil_quad_fc
  COMPOSITION marv_composition_quad_fc PARAMS marv_params_fc3v2 STORAGE)
# The same composition with the ESKF as the attitude source (decision 0055; marv_composition_quad_fc_eskf, the FC3v2 set with
# the ESKF register). No STORAGE: the config store is built for marv_params_fc3v2, whose table this set's does not equal.
marv_add_sil_library(marv_sil_quad_fc_eskf
  COMPOSITION marv_composition_quad_fc_eskf PARAMS marv_params_fc3v2_eskf)
