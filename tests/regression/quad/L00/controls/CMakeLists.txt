if(MARV_TARGET STREQUAL "m33")
  add_library(pb2_double_promotion OBJECT EXCLUDE_FROM_ALL pb2_double_promotion.cpp)
  marv_apply_flags(pb2_double_promotion FIRMWARE)
endif()

# G3 negative controls. Each is built only by its CI step and listed, as if it were a flight (or SIL) target, in its
# own manifest so that check_g3.py is fed it explicitly; none is in the real manifest.
add_library(g3_planted_truth STATIC EXCLUDE_FROM_ALL g3_planted_truth.cpp)
marv_apply_flags(g3_planted_truth FIRMWARE)
marv_write_g3_manifest("${CMAKE_CURRENT_BINARY_DIR}/g3_control_symbol.json" FLIGHT g3_planted_truth)

add_library(g3_planted_include STATIC EXCLUDE_FROM_ALL g3_planted_include.cpp)
target_include_directories(g3_planted_include PRIVATE "${PROJECT_SOURCE_DIR}/fw/hal/sim/include")
target_link_libraries(g3_planted_include PRIVATE marv_hal)
marv_apply_flags(g3_planted_include FIRMWARE)
marv_write_g3_manifest("${CMAKE_CURRENT_BINARY_DIR}/g3_control_include.json" FLIGHT g3_planted_include)

if(MARV_TARGET STREQUAL "host")
  add_library(g3_planted_export SHARED EXCLUDE_FROM_ALL g3_planted_export.cpp)
  marv_apply_flags(g3_planted_export FIRMWARE)
  marv_write_g3_manifest("${CMAKE_CURRENT_BINARY_DIR}/g3_control_export.json" SIL g3_planted_export)
endif()

# G3 target-coverage controls: an OBJECT library, an INTERFACE library and a forced include, each planted in the way
# the widened checker must catch.
add_library(g3_planted_object OBJECT EXCLUDE_FROM_ALL g3_planted_truth.cpp)
marv_apply_flags(g3_planted_object FIRMWARE)
marv_write_g3_manifest("${CMAKE_CURRENT_BINARY_DIR}/g3_control_object.json" FLIGHT g3_planted_object)

# The INTERFACE library has no translation unit of its own, so the manifest also lists a clean carrier that does not
# consume it: the violation can only come from the INTERFACE include rule and the run is not vacuous.
add_library(g3_planted_interface INTERFACE)
target_include_directories(g3_planted_interface INTERFACE "${PROJECT_SOURCE_DIR}/fw/hal/sim/include")
add_library(g3_planted_interface_carrier STATIC EXCLUDE_FROM_ALL g3_clean.cpp)
marv_apply_flags(g3_planted_interface_carrier FIRMWARE)
marv_write_g3_manifest("${CMAKE_CURRENT_BINARY_DIR}/g3_control_interface.json"
  FLIGHT g3_planted_interface g3_planted_interface_carrier)

add_library(g3_planted_forced STATIC EXCLUDE_FROM_ALL g3_clean.cpp)
target_compile_options(g3_planted_forced PRIVATE
  "SHELL:-include ${PROJECT_SOURCE_DIR}/fw/hal/sim/include/marv/hal_sim/hal_sim.hpp")
target_link_libraries(g3_planted_forced PRIVATE marv_hal)
marv_apply_flags(g3_planted_forced FIRMWARE)
marv_write_g3_manifest("${CMAKE_CURRENT_BINARY_DIR}/g3_control_forced.json" FLIGHT g3_planted_forced)
