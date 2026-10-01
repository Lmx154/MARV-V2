#include "marv/gz/adapter.hpp"

#include <cmath>
#include <cstring>
#include <limits>
#include <utility>

#include "marv/gz/constants.hpp"
#include "marv_sil.h"

namespace marv::gz {

double tick_period_s(std::uint32_t num_us, std::uint32_t den) {
  return static_cast<double>(num_us) / (static_cast<double>(den) * kMicrosecondsPerSecond);
}

double clock_error(int corner, double odr_error) {
  if (corner < -1 || corner > 1 || !std::isfinite(odr_error) || std::fabs(odr_error) >= 1.0) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return static_cast<double>(corner) * odr_error;
}

double true_tick_period_s(double t_nom_s, double e) {
  if (!std::isfinite(e) || !(1.0 + e > 0.0)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return t_nom_s / (1.0 + e);
}

bool CommandSource::dshot(std::uint64_t tick, const marv_imu_meas* imu, Dshot& out) {
  static_cast<void>(imu);
  return dshot(tick, out);
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

static_assert(sizeof(marv_plant_imu_out) == sizeof(marv_imu_meas), "the plant's IMU output is the SIL's sample");

bool ImuSilCommandSource::dshot(std::uint64_t tick, Dshot& out) { return dshot(tick, nullptr, out); }

bool ImuSilCommandSource::dshot(std::uint64_t tick, const marv_imu_meas* imu, Dshot& out) {
  const marv_imu_meas zero{};
  std::uint64_t t_us = 0;
  Dshot d{};
  marv_sil_out o{};
  o.struct_size = sizeof(o);
  o.capacity_ticks = 1;
  o.t_us = &t_us;
  o.dshot = d.data();
  o.servo_us = nullptr;
  status_ = marv_sil_tick(tick, 1, imu != nullptr ? imu : &zero, &o);
  if (status_ != MARV_SIL_OK) {
    return false;
  }
  stamp_us_ = t_us;
  out = d;
  return true;
}

Adapter::Adapter(marv_plant* plant, CommandSource& source, double t_tick_s)
    : Adapter(plant, source, AdapterConfig{t_tick_s, 0, 0.0, nullptr}) {}

Adapter::Adapter(marv_plant* plant, CommandSource& source, const AdapterConfig& cfg)
    : plant_(plant),
      source_(source),
      t_tick_s_(true_tick_period_s(cfg.t_tick_nominal_s, clock_error(cfg.clock_corner, cfg.odr_error))) {
  if (cfg.imu != nullptr) {
    imu_enabled_ = true;
    imu_attach_status_ = marv_plant_imu_attach(plant_, cfg.imu);
  }
}

Adapter::~Adapter() { marv_plant_destroy(plant_); }

StepResult Adapter::step(const marv_plant_body& body, std::uint64_t first_tick, std::uint32_t m) {
  StepResult r;
  if (m < 1) {
    r.status = Status::kZeroTicks;
    return r;
  }
  if (imu_attach_status_ != MARV_PLANT_OK) {
    r.status = Status::kPlant;
    r.plant_status = imu_attach_status_;
    return r;
  }
  r.ticks.reserve(m);
  Wrench sum{};
  for (std::uint32_t i = 0; i < m; ++i) {
    TickOutput t;
    t.tick = first_tick + i;
    bool got = false;
    if (imu_enabled_) {
      const marv_plant_status is = marv_plant_imu_sample(plant_, &body, t_tick_s_, &t.imu);
      if (is != MARV_PLANT_OK) {
        r.status = Status::kPlant;
        r.plant_status = is;
        return r;
      }
      marv_imu_meas meas{};
      std::memcpy(&meas, &t.imu, sizeof(meas));
      got = source_.dshot(t.tick, &meas, t.dshot);
    } else {
      got = source_.dshot(t.tick, t.dshot);
    }
    if (!got) {
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
