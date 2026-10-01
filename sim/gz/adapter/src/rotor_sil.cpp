#include <cstddef>

#include "marv/gz/adapter.hpp"
#include "marv_sil.h"

namespace marv::gz {

static_assert(sizeof(marv_plant_rotor_speed_out) == sizeof(marv_rotor_speed_meas),
              "the plant's rotor-speed output is the SIL's sample");
static_assert(offsetof(marv_plant_rotor_speed_out, omega_rad_s) == offsetof(marv_rotor_speed_meas, omega_rad_s));
static_assert(offsetof(marv_plant_rotor_speed_out, flags) == offsetof(marv_rotor_speed_meas, flags));
static_assert(sizeof(marv_plant_rotor_speed_out::omega_rad_s) == sizeof(marv_rotor_speed_meas::omega_rad_s));
static_assert(static_cast<int>(MARV_PLANT_N_MOTORS) == static_cast<int>(MARV_ROTOR_SPEED_MOTORS));
static_assert(static_cast<int>(MARV_PLANT_ROTOR_SPEED_M1_VALID) == static_cast<int>(MARV_ROTOR_SPEED_M1_VALID) &&
              static_cast<int>(MARV_PLANT_ROTOR_SPEED_M2_VALID) == static_cast<int>(MARV_ROTOR_SPEED_M2_VALID) &&
              static_cast<int>(MARV_PLANT_ROTOR_SPEED_M3_VALID) == static_cast<int>(MARV_ROTOR_SPEED_M3_VALID) &&
              static_cast<int>(MARV_PLANT_ROTOR_SPEED_M4_VALID) == static_cast<int>(MARV_ROTOR_SPEED_M4_VALID));

bool RotorSilCommandSource::dshot(std::uint64_t tick, Dshot& out) { return dshot(tick, nullptr, nullptr, out); }

bool RotorSilCommandSource::dshot(std::uint64_t tick, const marv_imu_meas* imu, Dshot& out) {
  return dshot(tick, imu, nullptr, out);
}

bool RotorSilCommandSource::dshot(std::uint64_t tick, const marv_imu_meas* imu, const marv_rotor_speed_meas* rotor,
                                  Dshot& out) {
  const marv_imu_meas zero{};
  std::uint64_t t_us = 0;
  Dshot d{};
  marv_sil_out o{};
  o.struct_size = sizeof(o);
  o.capacity_ticks = 1;
  o.t_us = &t_us;
  o.dshot = d.data();
  o.servo_us = nullptr;
  const marv_imu_meas* sample = imu != nullptr ? imu : &zero;
  status_ = rotor != nullptr ? marv_sil_tick_with_rotor_speed(tick, 1, sample, rotor, &o)
                             : marv_sil_tick(tick, 1, sample, &o);
  if (status_ != MARV_SIL_OK) {
    return false;
  }
  stamp_us_ = t_us;
  out = d;
  return true;
}

}  // namespace marv::gz
