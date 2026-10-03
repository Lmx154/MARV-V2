# The L4 rate-scripted SIL entry library (host only): the SIL entry built from the l4_rate_scripted composition. The
# Gazebo plugin (preset host-gz-l4) and the L4 composition tests load it. marv_rate_group, marv_gyro_chain, marv_rate,
# marv_mixer and marv_sched are fw/ targets built without PIC by default; the property is set here, on the existing
# targets, as sim/gz does for marv_prim.
#
# marv_rate_group, marv_rate and marv_mixer read the product set (marv_params_runtime) while the composition's set
# marv_params_l4_rate_scripted is the product set plus the composition's own register, appended last, so every product
# ParamId keeps its value. The composition's set is linked first and defines param_get, param_defaults and the generated
# tables; the product set's archive members are not pulled in.
set_target_properties(marv_rate_group marv_gyro_chain marv_rate marv_mixer marv_sched marv_hal marv_prim marv_types
  PROPERTIES POSITION_INDEPENDENT_CODE ON)
marv_add_sil_library(marv_sil_l4_rate_scripted
  COMPOSITION marv_composition_l4_rate_scripted PARAMS marv_params_l4_rate_scripted)
