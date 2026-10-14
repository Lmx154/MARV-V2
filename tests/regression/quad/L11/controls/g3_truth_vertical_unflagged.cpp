// A SIL-style library whose manifest entry is NOT flagged truth_vertical but which exports marv_truth_vertical_set: the
// product rule must reject it (G3-EXPORT).
extern "C" __attribute__((visibility("default"))) int marv_sil_planted_ok() {
  return 1;
}

extern "C" __attribute__((visibility("default"))) int marv_truth_vertical_set() {
  return 1;
}
