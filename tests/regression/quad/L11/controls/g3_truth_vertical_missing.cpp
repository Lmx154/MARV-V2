// A library flagged truth_vertical that does not export marv_truth_vertical_set: the flag without the export must be a
// tool-level error (G3-ERROR).
extern "C" __attribute__((visibility("default"))) int marv_sil_planted_ok() {
  return 1;
}
