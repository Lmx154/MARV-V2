// Negative control for the G1 sim-plant scope: a physics constant written as a bare literal.
double planted_plant_drag_coefficient(double speed_m_s) {
  return 0.0123 * speed_m_s;
}
