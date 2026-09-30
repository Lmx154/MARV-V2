#pragma once

// Sim-side command source for quad L4: the SIL's gyro sample is the truth body rate. Lives under sim/ only (quad spec
// 4 L4, core 7.4 G3): the marv::truth namespace never reaches fw/. Built as part of marv_gz_adapter; the flight-target
// manifest (cmake/flight_targets.cmake) lists targets defined under fw/ only.
//
// Freshness: within one host step the sample is the body state at the step's start (the host reads the state once per
// step and holds it over the step's m ticks, adapter.hpp).

#include <cstdint>

#include "marv/gz/adapter.hpp"
#include "marv_plant.h"
#include "marv_sil.h"

namespace marv::truth {

// marv_sil_tick(tick, 1, &imu, out) with imu.gyro_rad_s = (float)body.omega_frd_rad_s per axis, flags = GyroValid only,
// accel and temperature zero with their valid bits clear. set_body is called before Adapter::step. The caller has run
// marv_sil_init.
class TruthGyroSilCommandSource final : public marv::gz::CommandSource {
 public:
  void set_body(const marv_plant_body& body);
  bool dshot(std::uint64_t tick, marv::gz::Dshot& out) override;
  std::int32_t last_status() const { return status_; }        // marv_sil_status of the last call
  std::uint64_t last_stamp_us() const { return stamp_us_; }   // the SIL's t_us of the last tick
  const marv_imu_meas& imu() const { return imu_; }           // the sample the next (or last) tick passes

 private:
  marv_imu_meas imu_{};
  std::int32_t status_ = 0;
  std::uint64_t stamp_us_ = 0;
};

}  // namespace marv::truth
