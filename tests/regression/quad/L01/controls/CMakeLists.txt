# G3 negative control for the truth plant: a planted marv_plant_ C symbol and marv::plant:: C++ symbol in a library
# that the manifest lists as a flight target. Built only by its CI step (EXCLUDE_FROM_ALL); not in the real manifest.
add_library(g3_planted_plant STATIC EXCLUDE_FROM_ALL g3_planted_plant.cpp)
marv_apply_flags(g3_planted_plant FIRMWARE)
marv_write_g3_manifest("${CMAKE_CURRENT_BINARY_DIR}/g3_control_plant.json" FLIGHT g3_planted_plant)
