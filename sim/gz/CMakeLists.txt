# The Gazebo adapter (host only, quad spec section 4 L2, decision 0003 items 5, 6 and 8): frame maps and the per-host-step
# loop over marv_plant and a command source, with no Gazebo header. The Gazebo plugin (a shared object) links it, so it
# and marv_plant are built position independent. marv_sil_tick is resolved by whatever links the adapter (a SIL
# library); only its header is used here. -ffp-contract=off comes from marv_apply_flags on host.
add_library(marv_gz_adapter STATIC adapter/src/adapter.cpp adapter/src/truth_gyro.cpp)
target_include_directories(marv_gz_adapter PUBLIC adapter/include)
target_include_directories(marv_gz_adapter PUBLIC "${PROJECT_SOURCE_DIR}/fw/sil/include")
target_link_libraries(marv_gz_adapter PUBLIC marv_plant)
set_target_properties(marv_gz_adapter marv_plant PROPERTIES POSITION_INDEPENDENT_CODE ON)
marv_apply_flags(marv_gz_adapter)

# The Gazebo lockstep plugin (quad spec 3.1): a gz-sim 8 (Harmonic) System plugin, built only with -DMARV_GZ=ON (preset
# host-gz). The marv-ci image has no Gazebo, so nothing under this option is configured by the other presets.
option(MARV_GZ "Build the gz-sim 8 lockstep plugin (needs gz-sim8)" OFF)
if(MARV_GZ)
  # The plugin is a shared object, so everything linked into it is position independent. marv_prim and marv_types are
  # fw/ targets: the property is set here, on the existing targets, and only in this build option.
  set_target_properties(marv_prim marv_types PROPERTIES POSITION_INDEPENDENT_CODE ON)
  add_subdirectory(plugin)
endif()
