#include "marv/gz/adapter.hpp"

#include <utility>

#include "marv/gz/constants.hpp"
#include "marv_sil.h"

namespace marv::gz {

double tick_period_s(std::uint32_t num_us, std::uint32_t den) {
  return static_cast<double>(num_us) / (static_cast<double>(den) * kMicrosecondsPerSecond);
}

bool ScriptedCommandSource::dshot(std::uint64_t tick, Dshot& out) {
  out = script_(tick);
  return true;
}

bool SilCommandSource::dshot(std::uint64_t tick, Dshot& out) {
  const marv_imu_meas imu{};  // zeroed; flags 0 leaves every gyro, accel and temperature valid bit clear
  std::uint64_t t_us = 0;
  Dshot d{};
  marv_sil_out o{};
  o.struct_size = sizeof(o);
  o.capacity_ticks = 1;
  o.t_us = &t_us;
  o.dshot = d.data();
  o.servo_us = nullptr;
  status_ = marv_sil_tick(tick, 1, &imu, &o);
  if (status_ != MARV_SIL_OK) {
    return false;
  }
  stamp_us_ = t_us;
  out = d;
  return true;
}

Adapter::Adapter(marv_plant* plant, CommandSource& source, double t_tick_s)
    : plant_(plant), source_(source), t_tick_s_(t_tick_s) {}

Adapter::~Adapter() { marv_plant_destroy(plant_); }

StepResult Adapter::step(const marv_plant_body& body, std::uint64_t first_tick, std::uint32_t m) {
  StepResult r;
  if (m < 1) {
    r.status = Status::kZeroTicks;
    return r;
  }
  r.ticks.reserve(m);
  Wrench sum{};
  for (std::uint32_t i = 0; i < m; ++i) {
    TickOutput t;
    t.tick = first_tick + i;
    if (!source_.dshot(t.tick, t.dshot)) {
      r.status = Status::kCommandSource;
      return r;
    }
    marv_plant_cmd cmd{};
    cmd.struct_size = sizeof(cmd);
    for (std::size_t k = 0; k < t.dshot.size(); ++k) {
      cmd.dshot[k] = t.dshot[k];
    }
    t.out.struct_size = sizeof(t.out);
    const marv_plant_status s = marv_plant_step(plant_, &body, &cmd, t_tick_s_, &t.out);
    if (s != MARV_PLANT_OK) {
      r.status = Status::kPlant;
      r.plant_status = s;
      return r;
    }
    t.wrench_enu = wrench_ned_to_enu({t.out.force_ned_n[0], t.out.force_ned_n[1], t.out.force_ned_n[2]},
                                     {t.out.torque_ned_nm[0], t.out.torque_ned_nm[1], t.out.torque_ned_nm[2]});
    if (i == 0) {
      sum = t.wrench_enu;
    } else {
      for (std::size_t c = 0; c < 3; ++c) {
        sum.force[c] += t.wrench_enu.force[c];
        sum.torque[c] += t.wrench_enu.torque[c];
      }
    }
    r.ticks.push_back(t);
  }
  const double count = static_cast<double>(m);
  for (std::size_t c = 0; c < 3; ++c) {
    r.wrench_enu.force[c] = sum.force[c] / count;
    r.wrench_enu.torque[c] = sum.torque[c] / count;
  }
  return r;
}

}  // namespace marv::gz
