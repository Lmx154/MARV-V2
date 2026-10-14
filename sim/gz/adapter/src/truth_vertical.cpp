#include "marv/gz/truth_vertical.hpp"

#include <cmath>
#include <cstring>

namespace marv::truth {

void TruthVertical::set_body(const marv_plant_body& body) {
  // Zeroed as bytes, as the truth attitude's state.
  std::memset(&state_, 0, sizeof(state_));
  state_.struct_size = sizeof(state_);
  const auto z = static_cast<float>(body.pos_ned_m[2]);
  const auto vz = static_cast<float>(body.vel_ned_m_s[2]);
  if (std::isfinite(z) && std::isfinite(vz)) {
    state_.flags = std::uint32_t{1} << MARV_TRUTH_VERTICAL_VALID;
    state_.z_ned_m = z;
    state_.vz_ned_m_s = vz;
  }
}

bool TruthVertical::dshot(std::uint64_t tick, marv::gz::Dshot& out) {
  state_.tick = tick;
  status_ = marv_truth_vertical_set(&state_);
  if (status_ != MARV_SIL_OK) {
    return false;
  }
  return inner_.dshot(tick, out);
}

bool TruthVertical::dshot(std::uint64_t tick, const marv_imu_meas* imu, marv::gz::Dshot& out) {
  state_.tick = tick;
  status_ = marv_truth_vertical_set(&state_);
  if (status_ != MARV_SIL_OK) {
    return false;
  }
  return inner_.dshot(tick, imu, out);
}

bool TruthVertical::dshot(std::uint64_t tick, const marv_imu_meas* imu, const marv_rotor_speed_meas* rotor,
                          marv::gz::Dshot& out) {
  state_.tick = tick;
  status_ = marv_truth_vertical_set(&state_);
  if (status_ != MARV_SIL_OK) {
    return false;
  }
  return inner_.dshot(tick, imu, rotor, out);
}

}  // namespace marv::truth
