#include "marv/gz/truth_attitude.hpp"

#include <cstddef>
#include <cstring>

namespace marv::truth {

void TruthAttitude::set_body(const marv_plant_body& body) {
  // Zeroed as bytes, padding included: the plugin logs the struct's bytes (lockstep_log.hpp, record 5).
  std::memset(&state_, 0, sizeof(state_));
  state_.struct_size = sizeof(state_);
  state_.flags = std::uint32_t{1} << MARV_TRUTH_ATTITUDE_VALID;
  for (std::size_t i = 0; i < MARV_TRUTH_Q_COUNT; ++i) {
    state_.q_wxyz[i] = static_cast<float>(body.q_wxyz[i]);
  }
  // Canonical sign, w >= 0 (core section 3), applied to the float cast: the plant's q is not canonical.
  if (state_.q_wxyz[MARV_TRUTH_Q_W] < 0) {
    for (std::size_t i = 0; i < MARV_TRUTH_Q_COUNT; ++i) {
      state_.q_wxyz[i] = -state_.q_wxyz[i];
    }
  }
  state_.omega_frd_rad_s.x = static_cast<float>(body.omega_frd_rad_s[0]);
  state_.omega_frd_rad_s.y = static_cast<float>(body.omega_frd_rad_s[1]);
  state_.omega_frd_rad_s.z = static_cast<float>(body.omega_frd_rad_s[2]);
}

bool TruthAttitude::dshot(std::uint64_t tick, marv::gz::Dshot& out) {
  state_.tick = tick;
  status_ = marv_truth_state_set(&state_);
  if (status_ != MARV_SIL_OK) {
    return false;
  }
  return inner_.dshot(tick, out);
}

bool TruthAttitude::dshot(std::uint64_t tick, const marv_imu_meas* imu, marv::gz::Dshot& out) {
  state_.tick = tick;
  status_ = marv_truth_state_set(&state_);
  if (status_ != MARV_SIL_OK) {
    return false;
  }
  return inner_.dshot(tick, imu, out);
}

bool TruthAttitude::dshot(std::uint64_t tick, const marv_imu_meas* imu, const marv_rotor_speed_meas* rotor,
                          marv::gz::Dshot& out) {
  state_.tick = tick;
  status_ = marv_truth_state_set(&state_);
  if (status_ != MARV_SIL_OK) {
    return false;
  }
  return inner_.dshot(tick, imu, rotor, out);
}

}  // namespace marv::truth
