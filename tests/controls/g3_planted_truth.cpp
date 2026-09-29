namespace marv::truth {
int planted_truth_state() {
  return 1;
}
}  // namespace marv::truth

extern "C" int marv_truth_planted_c() {
  return 1;
}
