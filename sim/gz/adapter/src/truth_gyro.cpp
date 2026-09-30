#include "marv/gz/truth_gyro.hpp"

namespace marv::truth {

void TruthGyroSilCommandSource::set_body(const marv_plant_body& body) {
  marv_imu_meas m{};  // accel and temperature zero, their valid bits clear
  m.gyro_rad_s.x = static_cast<float>(body.omega_frd_rad_s[0]);
  m.gyro_rad_s.y = static_cast<float>(body.omega_frd_rad_s[1]);
  m.gyro_rad_s.z = static_cast<float>(body.omega_frd_rad_s[2]);
  m.flags = std::uint32_t{1} << MARV_IMU_GYRO_VALID;
  imu_ = m;
}

bool TruthGyroSilCommandSource::dshot(std::uint64_t tick, marv::gz::Dshot& out) {
  std::uint64_t t_us = 0;
  marv::gz::Dshot d{};
  marv_sil_out o{};
  o.struct_size = sizeof(o);
  o.capacity_ticks = 1;
  o.t_us = &t_us;
  o.dshot = d.data();
  o.servo_us = nullptr;
  status_ = marv_sil_tick(tick, 1, &imu_, &o);
  if (status_ != MARV_SIL_OK) {
    return false;
  }
  stamp_us_ = t_us;
  out = d;
  return true;
}

}  // namespace marv::truth
