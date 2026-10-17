# The L7 attitude-estimated test composition (quad spec section 4.0 M3, decision 0042 item 4): the L5 attitude-scripted logic
# (angle mode, the attitude law, the rate loop in bypass and the mixer, a scripted stick and thrust from scenario parameters)
# with the attitude state taken from the Mahony estimator (fw/estimator) on the IMU sample, not from a truth hook. The
# scripted inputs are l5_script.hpp's, included from the L5 composition's include directory (PRIVATE and after this one, so
# <marv/composition.hpp> is this composition's); nothing of the L5 composition is compiled here. Static library on host. The
# SIL library is built from it by marv_add_sil_library in sim/l7_attitude_estimated, without TRUTH_STATE.
add_library(marv_composition_l7_attitude_estimated STATIC src/l7_attitude_estimated.cpp)
target_include_directories(marv_composition_l7_attitude_estimated PUBLIC include)
target_include_directories(marv_composition_l7_attitude_estimated PRIVATE ../l5_attitude_scripted/include)
target_link_libraries(marv_composition_l7_attitude_estimated
  PUBLIC marv_hal marv_types marv_prim
  PRIVATE marv_sched marv_attitude marv_rate_group marv_rate marv_mixer marv_estimator
          marv_params_l7_attitude_estimated_runtime)
marv_apply_flags(marv_composition_l7_attitude_estimated FIRMWARE)
