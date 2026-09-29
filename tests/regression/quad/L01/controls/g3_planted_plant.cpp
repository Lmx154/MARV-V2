namespace marv::plant {
int planted_plant_state() {
  return 1;
}
}  // namespace marv::plant

extern "C" int marv_plant_planted_c() {
  return 1;
}
