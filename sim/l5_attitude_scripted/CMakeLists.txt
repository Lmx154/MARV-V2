# The L5 attitude-scripted SIL entry library (host only): the SIL entry built from the l5_attitude_scripted composition,
# with the test-only truth-state entry (TRUTH_STATE, decision 0006 B). The Gazebo plugin (preset host-gz-l5) and the L5
# composition tests load it. marv_attitude, marv_rate, marv_mixer and marv_sched are fw/ targets built without PIC by
# default; the property is set here, on the existing targets, as sim/l4_rate_scripted does.
#
# marv_attitude, marv_rate and marv_mixer read the product set (marv_params_runtime) while the composition's set
# marv_params_l5_attitude_scripted is the product set plus the composition's own register, appended last, so every product
# ParamId keeps its value. The composition's set is linked first and defines param_get, param_defaults and the generated
# tables; the product set's archive members are not pulled in.
set_target_properties(marv_attitude marv_rate marv_mixer marv_sched marv_hal marv_prim marv_types
  PROPERTIES POSITION_INDEPENDENT_CODE ON)
marv_add_sil_library(marv_sil_l5_attitude_scripted
  COMPOSITION marv_composition_l5_attitude_scripted PARAMS marv_params_l5_attitude_scripted TRUTH_STATE)
