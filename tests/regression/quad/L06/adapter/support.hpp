// Shared helpers of the L6 adapter tests. Every number is a labelled test value with its reason, or derived on the line.
#pragma once

#include <gtest/gtest.h>

#include <marv_sil.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <vector>

#include "marv/gz/adapter.hpp"
#include "marv_plant.h"
#include "plant_fixture.hpp"

namespace marv::gz::l6test {

using marv::plant::test::fixture_config;

// The SIL period 625 / 4 us = 156.25 us (design/scenario_values.yaml tick_period_num_us and tick_period_den).
constexpr std::uint32_t kPeriodNumUs = 625;
constexpr std::uint32_t kPeriodDen = 4;
// The worst-case linear sum of the crystal's tolerance, stability and first-year aging, 30 + 30 + 5 ppm (decision 0012,
// owner decision 2); the magnitude the profile will supply. Labelled test value here.
// Decision 0012: +-65 ppm worst-case linear sum, ABM8-272-T3 (Abracon #456603 rev IR), Pico 2 test setup, first year.
constexpr double kOdrError = 65.0e-6;
// 24 ticks: divisible by every host-step multiplicity under test (1, 2, 3, 4).
constexpr std::size_t kTicks = 24;

inline double t_nom() { return tick_period_s(kPeriodNumUs, kPeriodDen); }

// The twin and the adapter's plant: the L1 fixture (scenario values), motor sub-step = the nominal tick as in the L2
// adapter test, `seed` as rng_seed.
inline marv_plant* make_plant(std::uint64_t seed) {
  marv_plant_config c = fixture_config();
  c.motor_substep_s = t_nom();
  c.rng_seed = seed;
  marv_plant* p = nullptr;
  EXPECT_EQ(marv_plant_create(&c, &p), MARV_PLANT_OK);
  return p;
}

// Body states, labelled test values: unit quaternions (exact to double rounding), rates inside the IMU's gyro range at
// the LSB below, mixed signs; position and velocity are validated and unused by the plant.
// Reasons: the quaternions are the identity, (1/2)(1, 1, 1, 1) and (0.6, 0.8, 0, 0), each of norm 1 (0.36 + 0.64 = 1), so
// three distinct attitudes are cycled. The rates are at most 0.3 rad/s, inside the +-0.5 rad/s word range with margin
// for the noise (sigma_d 5.7e-3) and the bias; their magnitudes and signs differ per axis and per state, so a swapped
// axis or a stale state changes the bytes. Position -10 m (NED z, 10 m up) and velocity 1.5 m/s are arbitrary nonzero
// finite values that pass validation.
inline marv_plant_body make_body(const std::array<double, 4>& q, const std::array<double, 3>& w) {
  marv_plant_body b{};
  b.struct_size = sizeof(b);
  b.pos_ned_m[2] = -10.0;
  b.vel_ned_m_s[0] = 1.5;
  for (std::size_t i = 0; i < 4; ++i) {
    b.q_wxyz[i] = q[i];
  }
  for (std::size_t i = 0; i < 3; ++i) {
    b.omega_frd_rad_s[i] = w[i];
  }
  return b;
}
inline const std::array<marv_plant_body, 3>& states() {
  static const std::array<marv_plant_body, 3> s = {
      make_body({1.0, 0.0, 0.0, 0.0}, {0.1, -0.2, 0.05}),
      make_body({0.5, 0.5, 0.5, 0.5}, {-0.25, 0.15, 0.2}),
      make_body({0.6, 0.8, 0.0, 0.0}, {0.3, 0.0, -0.3}),
  };
  return s;
}

// Noise parameters, labelled test values (not sensor figures): gyro N = 1e-4 rad/s/sqrt(Hz), B = 2e-4 rad/s, LSB 2^-20
// (word range +-0.5 rad/s, rates above stay inside with the noise: sigma_d at dt = 156.25 us is
// 1e-4 / sqrt(2 * 156.25e-6) = 5.7e-3); accel N = 2e-3, B = 4e-3 m/s^2, LSB 2^-14 (word range +-32 m/s^2; the thrust of
// the largest DShot in stub_dshot is 4 * 2e-7 * 3622^2 / 0.75 = 14 m/s^2); full scale 100 binds nothing.
// B = 2 N for both sensors, so the Allan minimum sits at tau* = (N/B)^2 = 0.25 s: the walk term is exercised without
// dominating the white term at the tick. The turn-on biases (gyro axis 0 +0.01 rad/s, accel axis 2 -0.02 m/s^2) are
// nonzero on one axis each, with opposite signs, so the bias path and the axis mapping are seen; both are far below
// the word ranges.
inline marv_plant_imu_config noisy_imu_config() {
  marv_plant_imu_config c{};
  c.struct_size = sizeof(c);
  c.latency_samples = 2;  // labelled: a nonzero delay so the delay line is exercised
  c.gyro.noise_density = 1.0e-4;
  c.gyro.bias_instability = 2.0e-4;
  c.gyro.lsb = 1.0 / 1048576.0;
  c.gyro.full_scale = 100.0;
  c.gyro.turn_on_bias[0] = 0.01;
  c.accel.noise_density = 2.0e-3;
  c.accel.bias_instability = 4.0e-3;
  c.accel.lsb = 1.0 / 16384.0;
  c.accel.full_scale = 100.0;
  c.accel.turn_on_bias[2] = -0.02;
  return c;
}

// The DShot of tick n, the same function for the adapter's source and the twin: 48 + ((37 n + 211 k) mod 1900) is a
// legal value (48..1947) that moves every motor differently every tick, so the accel truth changes tick to tick.
// Reasons: DShot throttle is 48..2047 (core contracts, actuators line: "throttle 48-2047"), and 48 + 1899 =
// 1947 stays inside it; 37 and 211 are primes coprime to 1900 = 2^2 * 5^2 * 19, so the value
// steps by a different amount per tick (37) and per motor (211) and does not repeat within a short run.
inline Dshot stub_dshot(std::uint64_t n) {
  Dshot d{};
  for (std::size_t k = 0; k < d.size(); ++k) {
    d[k] = static_cast<std::uint16_t>(48 + ((37 * n + 211 * k) % 1900));
  }
  return d;
}

// What the recording marv_sil_tick (imu_path_test.cpp) saw.
struct Recorder {
  std::vector<marv_imu_meas> seen;
  std::vector<std::uint64_t> first_ticks;
  std::vector<std::uint32_t> counts;
  std::size_t null_imu = 0;
};
inline Recorder g_rec;

inline bool same_bytes(const void* a, const void* b, std::size_t n) { return std::memcmp(a, b, n) == 0; }
inline bool same_bits(const marv_plant_out& a, const marv_plant_out& b) { return same_bytes(&a, &b, sizeof(a)); }
inline bool same_bits(const marv_plant_imu_out& a, const marv_plant_imu_out& b) { return same_bytes(&a, &b, sizeof(a)); }
inline bool same_bits(const marv_imu_meas& a, const marv_plant_imu_out& b) {
  static_assert(sizeof(marv_imu_meas) == sizeof(marv_plant_imu_out));
  return same_bytes(&a, &b, sizeof(a));
}
inline bool same_bits(double a, double b) { return same_bytes(&a, &b, sizeof(a)); }

using BodyOf = std::function<marv_plant_body(std::size_t host_step)>;

// The state of host step h cycles through states() from `salt`.
inline BodyOf cycling(std::size_t salt = 0) {
  return [salt](std::size_t h) { return states()[(h + salt) % states().size()]; };
}

struct RunSpec {
  std::uint64_t seed = 0;
  const marv_plant_imu_config* imu = nullptr;
  int corner = 0;
  double odr_error = 0.0;
  std::uint32_t m = 1;
  BodyOf body = cycling();
  bool legacy_ctor = false;  // the three-argument constructor, source Scripted
  bool scripted = false;     // ScriptedCommandSource(stub_dshot) instead of ImuSilCommandSource
};

struct AdapterRun {
  std::vector<TickOutput> ticks;
  double t_tick_s = 0.0;
};

inline AdapterRun run_adapter(const RunSpec& s) {
  g_rec = Recorder{};
  AdapterRun run;
  ScriptedCommandSource scripted(stub_dshot);
  ImuSilCommandSource imu_src;
  CommandSource& src = s.scripted ? static_cast<CommandSource&>(scripted) : static_cast<CommandSource&>(imu_src);
  AdapterConfig cfg;
  cfg.t_tick_nominal_s = t_nom();
  cfg.clock_corner = s.corner;
  cfg.odr_error = s.odr_error;
  cfg.imu = s.imu;
  Adapter adapter = s.legacy_ctor ? Adapter(make_plant(s.seed), src, t_nom()) : Adapter(make_plant(s.seed), src, cfg);
  run.t_tick_s = adapter.t_tick_s();
  for (std::size_t h = 0; h * s.m < kTicks; ++h) {
    const StepResult r = adapter.step(s.body(h), h * s.m, s.m);
    EXPECT_EQ(r.status, Status::kOk);
    if (r.status != Status::kOk) {
      break;
    }
    run.ticks.insert(run.ticks.end(), r.ticks.begin(), r.ticks.end());
  }
  return run;
}

// The direct sequence on a twin plant: per tick j, sample (when `imu`) of the body of host step j / m at dt, then step
// with stub_dshot(j) at dt.
struct TwinRun {
  std::vector<marv_plant_imu_out> imu;
  std::vector<marv_plant_out> out;
};

inline TwinRun run_twin(std::uint64_t seed, const marv_plant_imu_config* imu, double dt, std::uint32_t m,
                        const BodyOf& body) {
  marv_plant* p = make_plant(seed);
  TwinRun t;
  if (imu != nullptr) {
    EXPECT_EQ(marv_plant_imu_attach(p, imu), MARV_PLANT_OK);
  }
  for (std::size_t j = 0; j < kTicks; ++j) {
    const marv_plant_body b = body(j / m);
    if (imu != nullptr) {
      marv_plant_imu_out o{};
      EXPECT_EQ(marv_plant_imu_sample(p, &b, dt, &o), MARV_PLANT_OK);
      t.imu.push_back(o);
    }
    const Dshot d = stub_dshot(j);
    marv_plant_cmd cmd{};
    cmd.struct_size = sizeof(cmd);
    for (std::size_t k = 0; k < d.size(); ++k) {
      cmd.dshot[k] = d[k];
    }
    marv_plant_out out{};
    out.struct_size = sizeof(out);
    EXPECT_EQ(marv_plant_step(p, &b, &cmd, dt, &out), MARV_PLANT_OK);
    t.out.push_back(out);
  }
  marv_plant_destroy(p);
  return t;
}

}  // namespace marv::gz::l6test
