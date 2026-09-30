// A library flagged truth_state that exports marv_truth_state_set (allowed) and marv_truth_planted (not): the truth_state
// rule must reject the extra export (G3-EXPORT).
extern "C" __attribute__((visibility("default"))) int marv_sil_planted_ok() {
  return 1;
}

extern "C" __attribute__((visibility("default"))) int marv_truth_state_set() {
  return 1;
}

extern "C" __attribute__((visibility("default"))) int marv_truth_planted() {
  return 1;
}
