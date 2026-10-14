# G3 negative controls for the truth_vertical export rule (decision 0020 section 1, G3 rule). Each is built only by its CI
# step (EXCLUDE_FROM_ALL) and listed, as if it were a SIL library, in its own manifest; none is in the real manifest
# (marv_collect_sil_libraries skips tests/regression/<product>/Lnn/controls).

# Unflagged, exports marv_truth_vertical_set: G3-EXPORT under the product rule.
add_library(g3_truth_vertical_unflagged_export SHARED EXCLUDE_FROM_ALL g3_truth_vertical_unflagged.cpp)
marv_apply_flags(g3_truth_vertical_unflagged_export FIRMWARE)
marv_write_g3_manifest("${CMAKE_CURRENT_BINARY_DIR}/g3_control_truth_vertical_unflagged.json" SIL
  g3_truth_vertical_unflagged_export)

# Flagged, also exports marv_truth_vertical_planted: G3-EXPORT under the truth_vertical rule.
add_library(g3_truth_vertical_planted_export SHARED EXCLUDE_FROM_ALL g3_truth_vertical_planted.cpp)
set_target_properties(g3_truth_vertical_planted_export PROPERTIES MARV_SIL_TRUTH_VERTICAL TRUE)
marv_apply_flags(g3_truth_vertical_planted_export FIRMWARE)
marv_write_g3_manifest("${CMAKE_CURRENT_BINARY_DIR}/g3_control_truth_vertical_planted.json" SIL
  g3_truth_vertical_planted_export)

# Flagged, does not export marv_truth_vertical_set: G3-ERROR.
add_library(g3_truth_vertical_missing_export SHARED EXCLUDE_FROM_ALL g3_truth_vertical_missing.cpp)
set_target_properties(g3_truth_vertical_missing_export PROPERTIES MARV_SIL_TRUTH_VERTICAL TRUE)
marv_apply_flags(g3_truth_vertical_missing_export FIRMWARE)
marv_write_g3_manifest("${CMAKE_CURRENT_BINARY_DIR}/g3_control_truth_vertical_missing.json" SIL
  g3_truth_vertical_missing_export)
