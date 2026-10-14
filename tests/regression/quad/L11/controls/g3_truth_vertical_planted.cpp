// A library flagged truth_vertical that exports marv_truth_vertical_set (allowed) and marv_truth_vertical_planted (not):
// the truth_vertical rule must reject the extra export (G3-EXPORT).
extern "C" __attribute__((visibility("default"))) int marv_sil_planted_ok() {
  return 1;
}

extern "C" __attribute__((visibility("default"))) int marv_truth_vertical_set() {
  return 1;
}

extern "C" __attribute__((visibility("default"))) int marv_truth_vertical_planted() {
  return 1;
}
