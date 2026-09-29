# marv_apply_flags(<target> [FIRMWARE])
#
# Applies the project compile flags to one of our own targets. Never applied
# globally, so third-party code (GoogleTest) is not built with -Werror.
# Firmware targets (source directory under fw/, or the FIRMWARE keyword) also get
# -fno-exceptions -fno-rtti. -fsingle-precision-constant is never used (core §3).
function(marv_apply_flags target)
  cmake_parse_arguments(ARG "FIRMWARE" "" "" ${ARGN})

  get_target_property(type ${target} TYPE)
  if(type STREQUAL "INTERFACE_LIBRARY")
    return()
  endif()

  target_compile_options(${target} PRIVATE
    -Wall -Wextra -Werror -Wdouble-promotion -Wfloat-conversion)

  if(MARV_TARGET STREQUAL "host")
    target_compile_options(${target} PRIVATE -ffp-contract=off)
  endif()

  get_target_property(src_dir ${target} SOURCE_DIR)
  cmake_path(IS_PREFIX PROJECT_SOURCE_DIR "${src_dir}" NORMALIZE under_project)
  cmake_path(RELATIVE_PATH src_dir BASE_DIRECTORY "${PROJECT_SOURCE_DIR}" OUTPUT_VARIABLE rel_dir)
  if(ARG_FIRMWARE OR (under_project AND rel_dir MATCHES "^fw(/|$)"))
    target_compile_options(${target} PRIVATE
      $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions -fno-rtti>)
  endif()
endfunction()
