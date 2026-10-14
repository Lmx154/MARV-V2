# The L11 vertical-scripted SIL entry library (host only): the SIL entry built from the l11_vertical_scripted
# composition, with the test-only truth-state and truth-vertical entries (TRUTH_STATE, decision 0006 B; TRUTH_VERTICAL,
# decision 0020 section 1, item c). marv_attitude, marv_vertical, marv_rate_group, marv_gyro_chain, marv_rate,
# marv_mixer and marv_sched are fw/ targets built without PIC by default; the property is set here, on the existing
# targets, as sim/l5_attitude_scripted does.
#
# marv_attitude, marv_rate_group, marv_rate and marv_mixer read the product set (marv_params_runtime) while the
# composition's set marv_params_l11_vertical_scripted is the product set plus the composition's own register, appended
# last, so every product ParamId keeps its value. The composition's set is linked first and defines param_get,
# param_defaults and the generated tables; the product set's archive members are not pulled in.
set_target_properties(marv_attitude marv_vertical marv_rate_group marv_gyro_chain marv_rate marv_mixer marv_sched
  marv_hal marv_prim marv_types
  PROPERTIES POSITION_INDEPENDENT_CODE ON)
marv_add_sil_library(marv_sil_l11_vertical_scripted
  COMPOSITION marv_composition_l11_vertical_scripted PARAMS marv_params_l11_vertical_scripted TRUTH_STATE TRUTH_VERTICAL)
