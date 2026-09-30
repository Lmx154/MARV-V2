# G3 negative controls for the truth-state export rule (docs/decisions/0006 section B). Each is built only by its CI
# step (EXCLUDE_FROM_ALL) and listed, as if it were a SIL library, in its own manifest; none is in the real manifest
# (marv_collect_sil_libraries skips tests/regression/<product>/Lnn/controls). The third control, a flagged entry
# without the export, is a pytest (tests/regression/quad/L05/tools).

# Unflagged, exports marv_truth_state_set: G3-EXPORT under the product rule.
add_library(g3_truth_unflagged_export SHARED EXCLUDE_FROM_ALL g3_truth_unflagged.cpp)
marv_apply_flags(g3_truth_unflagged_export FIRMWARE)
marv_write_g3_manifest("${CMAKE_CURRENT_BINARY_DIR}/g3_control_truth_unflagged.json" SIL g3_truth_unflagged_export)

# Flagged, also exports marv_truth_planted: G3-EXPORT under the truth_state rule.
add_library(g3_truth_planted_export SHARED EXCLUDE_FROM_ALL g3_truth_planted.cpp)
set_target_properties(g3_truth_planted_export PROPERTIES MARV_SIL_TRUTH_STATE TRUE)
marv_apply_flags(g3_truth_planted_export FIRMWARE)
marv_write_g3_manifest("${CMAKE_CURRENT_BINARY_DIR}/g3_control_truth_planted.json" SIL g3_truth_planted_export)
