#include "marv/gz/adapter.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <utility>

#include "marv/gz/constants.hpp"
#include "marv_sil.h"

namespace marv::gz {

namespace {

// The touchdown of the opt-in ground (adapter.hpp): the impact read from the body passed, and the rest body. Roll, pitch and yaw
// are the ZYX angles of the body -> NED quaternion (w, x, y, z): roll = atan2(2 (w x + y z), 1 - 2 (x^2 + y^2)), pitch =
// asin(2 (w y - x z)) (the argument clamped to [-1, 1]), yaw = atan2(2 (w z + x y), 1 - 2 (y^2 + z^2)); the rest attitude is
// (cos(yaw / 2), 0, 0, sin(yaw / 2)), w >= 0 for yaw in [-pi, pi].
Touchdown touchdown_of(const marv_plant_body& b, std::uint64_t tick, double ground_down_m) {
  const double w = b.q_wxyz[0];
  const double x = b.q_wxyz[1];
  const double y = b.q_wxyz[2];
  const double z = b.q_wxyz[3];
  Touchdown td;
  td.tick = tick;
  td.down_speed_m_s = b.vel_ned_m_s[2];
  td.horizontal_speed_m_s = std::hypot(b.vel_ned_m_s[0], b.vel_ned_m_s[1]);
  td.roll_rad = std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
  td.pitch_rad = std::asin(std::clamp(2.0 * (w * y - x * z), -1.0, 1.0));
  const double half_yaw = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)) / 2.0;
  marv_plant_body& r = td.rest;
  r.struct_size = sizeof(r);
  r.pos_ned_m[0] = b.pos_ned_m[0];
  r.pos_ned_m[1] = b.pos_ned_m[1];
  r.pos_ned_m[2] = ground_down_m;
  r.q_wxyz[0] = std::cos(half_yaw);
  r.q_wxyz[3] = std::sin(half_yaw);
  return td;
}

}  // namespace

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

bool CommandSource::dshot(std::uint64_t tick, const marv_imu_meas* imu, const marv_rotor_speed_meas* rotor, Dshot& out) {
  static_cast<void>(rotor);
  return imu != nullptr ? dshot(tick, imu, out) : dshot(tick, out);
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
    : Adapter(plant, source, AdapterConfig{t_tick_s, 0, 0.0, nullptr, nullptr, 0.0, 0, false, false}) {}

Adapter::Adapter(marv_plant* plant, CommandSource& source, const AdapterConfig& cfg)
    : plant_(plant),
      source_(source),
      t_tick_s_(cfg.t_tick_true_s != 0.0
                    ? cfg.t_tick_true_s
                    : true_tick_period_s(cfg.t_tick_nominal_s, clock_error(cfg.clock_corner, cfg.odr_error))) {
  delay_ = cfg.command_delay_ticks;
  pad_ = cfg.launch_pad;
  held_ = pad_;
  ground_ = pad_ && cfg.ground;
  if (delay_ > 0 && delay_ <= kMaxCommandDelayTicks) {
    delay_line_.assign(delay_, Dshot{});
  }
  if (cfg.imu != nullptr) {
    imu_enabled_ = true;
    imu_attach_status_ = marv_plant_imu_attach(plant_, cfg.imu);
  }
  if (cfg.rotor_speed != nullptr) {
    rotor_speed_enabled_ = true;
    rotor_speed_attach_status_ = marv_plant_rotor_speed_attach(plant_, cfg.rotor_speed);
  }
}

Adapter::~Adapter() { marv_plant_destroy(plant_); }

StepResult Adapter::step(const marv_plant_body& body, std::uint64_t first_tick, std::uint32_t m) {
  StepResult r;
  if (m < 1) {
    r.status = Status::kZeroTicks;
    return r;
  }
  if (delay_ > kMaxCommandDelayTicks) {
    r.status = Status::kBadDelay;
    return r;
  }
  if (imu_attach_status_ != MARV_PLANT_OK) {
    r.status = Status::kPlant;
    r.plant_status = imu_attach_status_;
    return r;
  }
  if (rotor_speed_attach_status_ != MARV_PLANT_OK) {
    r.status = Status::kPlant;
    r.plant_status = rotor_speed_attach_status_;
    return r;
  }
  r.ticks.reserve(m);
  const marv_plant_body* bp = &body;  // the body of every tick of this step: the one passed, or the rest body of a touchdown
  if (ground_) {
    if (!ground_down_m_) {
      ground_down_m_ = body.pos_ned_m[2];
    }
    if (!held_ && body.pos_ned_m[2] >= *ground_down_m_ && body.vel_ned_m_s[2] > 0.0) {
      r.touchdown = touchdown_of(body, first_tick, *ground_down_m_);
      bp = &r.touchdown->rest;
      held_ = true;
    }
  }
  Wrench sum{};
  bool have_sum = false;
  for (std::uint32_t i = 0; i < m; ++i) {
    TickOutput t;
    t.tick = first_tick + i;
    bool got = false;
    if (held_) {
      // The probe of the header comment: the total force for the rotor state this tick's IMU sample sees.
      marv_plant_cmd stop{};
      stop.struct_size = sizeof(stop);
      marv_plant_out probe{};
      probe.struct_size = sizeof(probe);
      marv_plant_status ps = marv_plant_step(plant_, bp, &stop, std::numeric_limits<double>::min(), &probe);
      std::array<double, 3> contact{-probe.force_ned_n[0], -probe.force_ned_n[1], -probe.force_ned_n[2]};
      if (ps == MARV_PLANT_OK && contact[2] >= 0.0) {
        contact = {0.0, 0.0, 0.0};
        held_ = false;
        t.pad_released = true;
        if (!release_tick_) {
          release_tick_ = t.tick;
        }
      } else {
        t.pad_held = true;
      }
      if (ps == MARV_PLANT_OK) {
        ps = marv_plant_contact_set(plant_, contact.data());
      }
      if (ps != MARV_PLANT_OK) {
        r.status = Status::kPlant;
        r.plant_status = ps;
        return r;
      }
    }
    if (imu_enabled_) {
      const marv_plant_status is = marv_plant_imu_sample(plant_, bp, t_tick_s_, &t.imu);
      if (is != MARV_PLANT_OK) {
        r.status = Status::kPlant;
        r.plant_status = is;
        return r;
      }
    }
    marv_imu_meas meas{};
    std::memcpy(&meas, &t.imu, sizeof(meas));
    if (rotor_speed_enabled_) {
      const marv_plant_status rs = marv_plant_rotor_speed_sample(plant_, &t.rotor_speed);
      if (rs != MARV_PLANT_OK) {
        r.status = Status::kPlant;
        r.plant_status = rs;
        return r;
      }
      marv_rotor_speed_meas rotor{};
      std::memcpy(&rotor, &t.rotor_speed, sizeof(rotor));
      got = source_.dshot(t.tick, imu_enabled_ ? &meas : nullptr, &rotor, t.dshot);
    } else if (imu_enabled_) {
      got = source_.dshot(t.tick, &meas, t.dshot);
    } else {
      got = source_.dshot(t.tick, t.dshot);
    }
    if (!got) {
      r.status = Status::kCommandSource;
      return r;
    }
    t.dshot_applied = t.dshot;
    if (delay_ > 0) {
      t.dshot_applied = delay_line_[delay_head_];
      delay_line_[delay_head_] = t.dshot;
      delay_head_ = (delay_head_ + 1) % delay_;
    }
    marv_plant_cmd cmd{};
    cmd.struct_size = sizeof(cmd);
    for (std::size_t k = 0; k < t.dshot_applied.size(); ++k) {
      cmd.dshot[k] = t.dshot_applied[k];
    }
    t.out.struct_size = sizeof(t.out);
    const marv_plant_status s = marv_plant_step(plant_, bp, &cmd, t_tick_s_, &t.out);
    if (s != MARV_PLANT_OK) {
      r.status = Status::kPlant;
      r.plant_status = s;
      return r;
    }
    t.wrench_enu = wrench_ned_to_enu({t.out.force_ned_n[0], t.out.force_ned_n[1], t.out.force_ned_n[2]},
                                     {t.out.torque_ned_nm[0], t.out.torque_ned_nm[1], t.out.torque_ned_nm[2]});
    if (t.pad_held) {
      // The pad balances this tick's wrench: nothing reaches the body.
    } else if (!have_sum) {
      sum = t.wrench_enu;
      have_sum = true;
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
