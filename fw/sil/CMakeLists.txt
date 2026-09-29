add_library(marv_sil INTERFACE)

# marv_add_sil_library(<name> COMPOSITION <static-lib-target> [PARAMS <param-set>])
#
# Host only. Builds the SIL entry as the shared library <name>: fw/sil sources, the composition, marv_hal_sim and
# the runtime of the parameter set PARAMS (marv_add_param_set; default marv_params, the product set), which must be the
# set the composition links. Only the marv_sil_* C functions are exported: the entry points carry a default-visibility
# attribute in marv_sil.cpp and the rest of the library is compiled with -fvisibility=hidden
# -fvisibility-inlines-hidden; the linker version script marv_sil.map additionally makes every other symbol local,
# which covers the objects taken from the static libraries. The sil and hal_sim include directories are PRIVATE to
# <name>; the composition target never sees them. Static libraries linked into the shared library are built PIC.
function(marv_add_sil_library name)
  cmake_parse_arguments(ARG "" "COMPOSITION;PARAMS" "" ${ARGN})
  if(NOT ARG_PARAMS)
    set(ARG_PARAMS marv_params)
  endif()

  if(NOT MARV_TARGET STREQUAL "host")
    message(FATAL_ERROR "marv_add_sil_library(${name}): the SIL library is host only")
  endif()
  if(NOT ARG_COMPOSITION OR NOT TARGET "${ARG_COMPOSITION}")
    message(FATAL_ERROR "marv_add_sil_library(${name}): COMPOSITION must name an existing target")
  endif()
  if(NOT TARGET "${ARG_PARAMS}_runtime" OR NOT TARGET "${ARG_PARAMS}_generated")
    message(FATAL_ERROR "marv_add_sil_library(${name}): PARAMS '${ARG_PARAMS}' is not a parameter set")
  endif()
  get_target_property(composition_type "${ARG_COMPOSITION}" TYPE)
  if(NOT composition_type STREQUAL "STATIC_LIBRARY")
    message(FATAL_ERROR
      "marv_add_sil_library(${name}): COMPOSITION '${ARG_COMPOSITION}' must be a static library, is ${composition_type}")
  endif()

  set(sil_dir "${CMAKE_CURRENT_FUNCTION_LIST_DIR}")
  add_library(${name} SHARED
    "${sil_dir}/src/marv_sil.cpp"
    "${sil_dir}/src/param_override.cpp"
    "${sil_dir}/src/param_override.hpp")
  target_include_directories(${name} PUBLIC "${sil_dir}/include")
  target_include_directories(${name} PRIVATE "${sil_dir}/src")
  target_link_libraries(${name} PRIVATE
    "${ARG_COMPOSITION}" marv_hal_sim ${ARG_PARAMS}_runtime ${ARG_PARAMS}_generated)
  target_compile_options(${name} PRIVATE -fvisibility=hidden -fvisibility-inlines-hidden)
  target_link_options(${name} PRIVATE "LINKER:--version-script=${sil_dir}/marv_sil.map")
  set_property(TARGET ${name} APPEND PROPERTY LINK_DEPENDS "${sil_dir}/marv_sil.map")
  set_target_properties("${ARG_COMPOSITION}" marv_hal_sim ${ARG_PARAMS}_runtime ${ARG_PARAMS}_generated
    PROPERTIES POSITION_INDEPENDENT_CODE ON)
  marv_apply_flags(${name} FIRMWARE)
endfunction()
