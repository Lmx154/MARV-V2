# The null plant (host only): a synthetic IMU stream and output checks around the SIL entry, no physics.
#   marv_sil_l0          the SIL entry library built from the fixed L0 composition
#   marv_null_plant      library: PRNG, IMU stream, trace hash, output checks, manifest lookup
#   marv_null_plant_run  runner: one SIL run per process, prints the trace hash
marv_add_sil_library(marv_sil_l0 COMPOSITION marv_composition_l0 PARAMS marv_params_l0)

add_library(marv_null_plant STATIC src/imu_synth.cpp src/trace.cpp src/manifest.cpp)
target_include_directories(marv_null_plant PUBLIC include)
target_link_libraries(marv_null_plant PUBLIC marv_sil_l0)
marv_apply_flags(marv_null_plant)

add_executable(marv_null_plant_run src/main.cpp)
target_link_libraries(marv_null_plant_run PRIVATE marv_null_plant marv_sil_l0)
target_compile_definitions(marv_null_plant_run PRIVATE
  MARV_NULL_PLANT_DEFAULT_MANIFEST="$<TARGET_PROPERTY:marv_params_l0_generated,MARV_PARAM_MANIFEST>")
marv_apply_flags(marv_null_plant_run)
