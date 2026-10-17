# The L7 attitude-estimated SIL entry library (host only): the SIL entry built from the l7_attitude_estimated composition,
# without the test-only truth-state entry (the attitude is the estimator's, decision 0042 item 4), so the library exports
# marv_sil_* only (marv_sil.map). The Gazebo plugin (preset host-gz-l7) loads it. The fw/ targets below are built without
# PIC by default; the property is set here, on the existing targets, as sim/l5_attitude_scripted does. marv_estimator is
# built here in the same way.
#
# marv_attitude, marv_rate_group, marv_rate and marv_mixer read the product set (marv_params_runtime) while the
# composition's set marv_params_l7_attitude_estimated is the product set plus the registers of the composition, appended
# last, so every product ParamId keeps its value.
set_target_properties(marv_attitude marv_rate_group marv_gyro_chain marv_rate marv_mixer marv_sched marv_estimator marv_hal
  marv_prim marv_types
  PROPERTIES POSITION_INDEPENDENT_CODE ON)
marv_add_sil_library(marv_sil_l7_attitude_estimated
  COMPOSITION marv_composition_l7_attitude_estimated PARAMS marv_params_l7_attitude_estimated)
