# The gz-sim 8 lockstep plugin marv::gz::Lockstep, file libmarv_gz_lockstep.so (the <plugin filename> the world
# generator writes is marv_gz_lockstep). A thin shell over marv_gz_adapter. Included only when MARV_GZ is ON.
#
# MARV_GZ_SIL     the SIL shared library the plugin links (a marv_add_sil_library target); the later composition swap
#                 changes this variable and nothing in the plugin.
# MARV_GZ_PARAMS  the parameter set of that composition (marv_add_param_set name): the SIL C ABI addresses parameters
#                 by id, and the world names them, so the plugin needs the set's name table (marv::generated::
#                 kParamNames). It must be the set the SIL library was built with.
set(MARV_GZ_SIL "marv_sil_l2_open_loop" CACHE STRING "SIL library linked into the Gazebo lockstep plugin")
set(MARV_GZ_PARAMS "marv_params_l2_open_loop" CACHE STRING "Parameter set of MARV_GZ_SIL (name table for sil_override)")

find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
if(NOT TARGET "${MARV_GZ_PARAMS}_generated")
  message(FATAL_ERROR "MARV_GZ: target '${MARV_GZ_PARAMS}_generated' does not exist (MARV_GZ_PARAMS='${MARV_GZ_PARAMS}')")
endif()
# MARV_GZ_SIL may be defined after this directory is processed (a test SIL library under tests/regression), so its
# existence is checked at the end of the top-level directory, and its properties are read by generator expressions.
function(marv_gz_check_sil_target sil)
  if(NOT TARGET "${sil}")
    message(FATAL_ERROR "MARV_GZ: target '${sil}' does not exist (MARV_GZ_SIL='${sil}')")
  endif()
endfunction()
cmake_language(DEFER DIRECTORY "${PROJECT_SOURCE_DIR}" CALL marv_gz_check_sil_target "${MARV_GZ_SIL}")

add_library(marv_gz_lockstep SHARED
  src/lockstep.cpp
  src/lockstep_log.cpp
  src/lockstep_log.hpp
  src/constants.hpp)
target_include_directories(marv_gz_lockstep PRIVATE src "${PROJECT_SOURCE_DIR}/fw/sil/include")
target_link_libraries(marv_gz_lockstep PRIVATE
  marv_gz_adapter marv_plant "${MARV_GZ_SIL}" "${MARV_GZ_PARAMS}_generated"
  gz-sim8::gz-sim8 gz-plugin2::register)
set_target_properties("${MARV_GZ_PARAMS}_generated" PROPERTIES POSITION_INDEPENDENT_CODE ON)
# gz dlopens the plugin from GZ_SIM_SYSTEM_PLUGIN_PATH: its own directory and the SIL library's are on its RUNPATH.
set_target_properties(marv_gz_lockstep PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  BUILD_RPATH "\$ORIGIN;$<TARGET_FILE_DIR:${MARV_GZ_SIL}>")
marv_apply_flags(marv_gz_lockstep)

# The truth-attitude source (quad L5, decision 0006 section B) is linked only when the SIL library has marv_truth_state_set,
# i.e. the target property MARV_SIL_TRUTH_STATE (marv_add_sil_library TRUTH_STATE), read at generate time. Without it the
# plugin refuses the <attitude_source> element at run time and still builds against a library without the entry.
set(marv_gz_sil_truth_state "$<BOOL:$<TARGET_PROPERTY:${MARV_GZ_SIL},MARV_SIL_TRUTH_STATE>>")
target_link_libraries(marv_gz_lockstep PRIVATE "$<${marv_gz_sil_truth_state}:marv_gz_truth_attitude>")
target_compile_definitions(marv_gz_lockstep PRIVATE "$<${marv_gz_sil_truth_state}:MARV_GZ_TRUTH_STATE=1>")
