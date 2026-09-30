# The L2 open-loop SIL entry library (host only): the SIL entry built from the l2_open_loop composition. The Gazebo
# plugin (T4) and the L2 tests load it.
marv_add_sil_library(marv_sil_l2_open_loop COMPOSITION marv_composition_l2_open_loop PARAMS marv_params_l2_open_loop)
