// L6 stage (a), the clock-error map (decision 0012, owner decisions 2 and 3): e enters in exactly one place, the sim-side
// true tick time t_nom / (1 + e), which is the adapter's tick time and the dt of marv_plant_step and
// marv_plant_imu_sample. It never reaches the SIL: the SIL's own stamps are checked against the real SIL in sil_test.cpp;
// here the recorder shows the SIL is called with the integer tick numbers whatever e is.
//
// The command delay (decision 0046 item 2), at the end of this file: the plant is stepped at tick j with the source's command
// of tick j - N and the stop command (DShot 0) before that; N = 0 is the adapter without it, bitwise; an N above the bound is
// refused.
//
// The launch pad (decision 0047), after the delay: while it holds, the IMU's truth specific force is R^T (0, 0, -g(h)) for any
// rotor state and body; it releases once, on the first tick whose contact force would pull (the vertical thrust has reached
// the weight), and never re-engages; after the release the run is a pad-free run from the same state; without the option
// every output is the adapter's without it.
#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <optional>

#include "marv/prim/gravity.hpp"
#include "support.hpp"

namespace {

using namespace marv::gz;
using namespace marv::gz::l6test;

constexpr std::array<int, 3> kCorners = {-1, 0, 1};

TEST(L6AdapterClock, TheMapIsCornerTimesOdrErrorAndTheTickIsTnomOverOnePlusE) {
  for (const int c : kCorners) {
    const double e = clock_error(c, kOdrError);
    EXPECT_TRUE(same_bits(e, static_cast<double>(c) * kOdrError));
    EXPECT_TRUE(same_bits(true_tick_period_s(t_nom(), e), t_nom() / (1.0 + e)));
  }
  // e = 0 gives t_nom bitwise (1 + 0 = 1 exactly).
  EXPECT_TRUE(same_bits(true_tick_period_s(t_nom(), clock_error(0, kOdrError)), t_nom()));
  EXPECT_TRUE(same_bits(true_tick_period_s(t_nom(), 0.0), t_nom()));
  // Signs: a fast clock (e > 0) has the shorter true tick.
  EXPECT_LT(true_tick_period_s(t_nom(), kOdrError), t_nom());
  EXPECT_GT(true_tick_period_s(t_nom(), -kOdrError), t_nom());
  // Refusals are NaN: a corner outside {-1, 0, 1}, a non-finite or |e| >= 1 magnitude, 1 + e <= 0.
  EXPECT_TRUE(std::isnan(clock_error(2, kOdrError)));
  EXPECT_TRUE(std::isnan(clock_error(-2, kOdrError)));
  EXPECT_TRUE(std::isnan(clock_error(1, std::numeric_limits<double>::infinity())));
  EXPECT_TRUE(std::isnan(clock_error(1, std::nan(""))));
  EXPECT_TRUE(std::isnan(clock_error(1, 1.0)));
  EXPECT_TRUE(std::isnan(true_tick_period_s(t_nom(), -1.0)));
  EXPECT_TRUE(std::isnan(true_tick_period_s(t_nom(), std::nan(""))));
}

TEST(L6AdapterClock, AnInvalidClockIsRefusedByThePlantAsADtError) {
  ScriptedCommandSource src(stub_dshot);
  AdapterConfig cfg;
  cfg.t_tick_nominal_s = t_nom();
  cfg.clock_corner = 2;
  cfg.odr_error = kOdrError;
  Adapter adapter(make_plant(0), src, cfg);
  const StepResult r = adapter.step(states()[0], 0, 1);
  EXPECT_EQ(r.status, Status::kPlant);
  EXPECT_EQ(r.plant_status, MARV_PLANT_E_DT);
}

TEST(L6AdapterClock, AdapterPlantAndImuOutputsAreATwinSteppedAtTnomOverOnePlusE) {
  const marv_plant_imu_config imu = noisy_imu_config();
  for (const int corner : kCorners) {
    const double e = clock_error(corner, kOdrError);
    const double dt = t_nom() / (1.0 + e);  // independent of true_tick_period_s
    for (const std::uint32_t m : {1u, 4u}) {
      RunSpec s;
      s.seed = 7;  // labelled: any seed
      s.imu = &imu;
      s.corner = corner;
      s.odr_error = kOdrError;
      s.m = m;
      const AdapterRun run = run_adapter(s);
      ASSERT_EQ(run.ticks.size(), kTicks);
      EXPECT_TRUE(same_bits(run.t_tick_s, dt));
      const TwinRun at_true = run_twin(7, &imu, dt, m, s.body);
      const TwinRun at_nom = run_twin(7, &imu, t_nom(), m, s.body);
      std::size_t out_vs_nom = 0;
      std::size_t imu_vs_nom = 0;
      for (std::size_t j = 0; j < kTicks; ++j) {
        EXPECT_TRUE(same_bits(run.ticks[j].out, at_true.out[j])) << "corner " << corner << " m " << m << " tick " << j;
        EXPECT_TRUE(same_bits(run.ticks[j].imu, at_true.imu[j])) << "corner " << corner << " m " << m << " tick " << j;
        out_vs_nom += same_bits(run.ticks[j].out, at_nom.out[j]) ? 0u : 1u;
        imu_vs_nom += same_bits(run.ticks[j].imu, at_nom.imu[j]) ? 0u : 1u;
      }
      if (corner == 0) {
        EXPECT_EQ(out_vs_nom, 0u);
        EXPECT_EQ(imu_vs_nom, 0u);
      } else {
        // The controls: a twin at the nominal tick differs (otherwise the equality above would not see e).
        EXPECT_GT(out_vs_nom, 0u) << "corner " << corner;
        EXPECT_GT(imu_vs_nom, 0u) << "corner " << corner;
      }
      // e never reaches the SIL: it is called once per tick with the integer tick number.
      ASSERT_EQ(g_rec.first_ticks.size(), kTicks);
      for (std::size_t j = 0; j < kTicks; ++j) {
        EXPECT_EQ(g_rec.first_ticks[j], j);
      }
    }
  }
}

TEST(L6AdapterClock, ZeroErrorIsBitIdenticalToTheAdapterWithoutTheClockConfig) {
  for (const std::uint32_t m : {1u, 3u}) {
    RunSpec legacy;
    legacy.legacy_ctor = true;
    legacy.scripted = true;
    legacy.m = m;
    RunSpec zero;
    zero.scripted = true;
    zero.m = m;
    zero.corner = 0;
    zero.odr_error = kOdrError;  // corner 0 zeroes it
    const AdapterRun a = run_adapter(legacy);
    const AdapterRun b = run_adapter(zero);
    EXPECT_TRUE(same_bits(a.t_tick_s, t_nom()));
    EXPECT_TRUE(same_bits(b.t_tick_s, t_nom()));
    ASSERT_EQ(a.ticks.size(), b.ticks.size());
    for (std::size_t j = 0; j < a.ticks.size(); ++j) {
      EXPECT_TRUE(same_bits(a.ticks[j].out, b.ticks[j].out));
      EXPECT_EQ(a.ticks[j].dshot, b.ticks[j].dshot);
      EXPECT_TRUE(same_bytes(&a.ticks[j].wrench_enu, &b.ticks[j].wrench_enu, sizeof(Wrench)));
    }
  }
}

// ---- the command delay (decision 0046 item 2) -------------------------------------------------------------------------------

// The adapter at command delay N over the stub command stream against an adapter without delay whose source is that stream
// shifted by N ticks (the stop command Dshot{} before it): same plant seed, same states, so the plant outputs and the wrenches
// must be the same bytes tick for tick.
std::vector<TickOutput> run_with_delay(std::uint32_t delay, std::uint32_t m, const std::function<Dshot(std::uint64_t)>& script) {
  ScriptedCommandSource src(script);
  AdapterConfig cfg;
  cfg.t_tick_nominal_s = t_nom();
  cfg.command_delay_ticks = delay;
  Adapter adapter(make_plant(7), src, cfg);  // labelled: any seed
  EXPECT_EQ(adapter.command_delay_ticks(), delay);
  std::vector<TickOutput> ticks;
  for (std::size_t h = 0; h * m < kTicks; ++h) {
    const StepResult r = adapter.step(cycling()(h), h * m, m);
    EXPECT_EQ(r.status, Status::kOk);
    ticks.insert(ticks.end(), r.ticks.begin(), r.ticks.end());
  }
  return ticks;
}

TEST(L6AdapterDelay, ThePlantTakesTheCommandOfTickJMinusNAndTheStopCommandBeforeIt) {
  // N = 1; 3 (the unit test's value, fewer than the 24 ticks); 10 (the HIL S1 round trip, 9.5 ticks, rounded up; HIL S1 README).
  for (const std::uint32_t n : {1u, 3u, 10u}) {
    for (const std::uint32_t m : {1u, 3u, 4u}) {
      const std::vector<TickOutput> delayed = run_with_delay(n, m, stub_dshot);
      const std::vector<TickOutput> shifted =
          run_with_delay(0, m, [n](std::uint64_t j) { return j < n ? Dshot{} : stub_dshot(j - n); });
      const std::vector<TickOutput> plain = run_with_delay(0, m, stub_dshot);
      ASSERT_EQ(delayed.size(), kTicks);
      ASSERT_EQ(shifted.size(), kTicks);
      std::size_t differs_from_plain = 0;
      for (std::size_t j = 0; j < kTicks; ++j) {
        const Dshot want = j < n ? Dshot{} : stub_dshot(j - n);
        EXPECT_EQ(delayed[j].dshot, stub_dshot(j)) << "N " << n << " m " << m << " tick " << j;  // the source's own command
        EXPECT_EQ(delayed[j].dshot_applied, want) << "N " << n << " m " << m << " tick " << j;
        EXPECT_TRUE(same_bits(delayed[j].out, shifted[j].out)) << "N " << n << " m " << m << " tick " << j;
        EXPECT_TRUE(same_bytes(&delayed[j].wrench_enu, &shifted[j].wrench_enu, sizeof(Wrench)));
        differs_from_plain += same_bits(delayed[j].out, plain[j].out) ? 0u : 1u;
      }
      // The control: the plant does feel the delay (otherwise the equality above would not see the shift).
      EXPECT_GT(differs_from_plain, 0u) << "N " << n << " m " << m;
      // The control: a stream shifted by one tick too few or too many is not the delayed run.
      for (const std::uint32_t wrong : {n - 1, n + 1}) {
        const std::vector<TickOutput> off = run_with_delay(0, m, [wrong](std::uint64_t j) { return j < wrong ? Dshot{} : stub_dshot(j - wrong); });
        std::size_t differs = 0;
        for (std::size_t j = 0; j < kTicks; ++j) {
          differs += same_bits(delayed[j].out, off[j].out) ? 0u : 1u;
        }
        EXPECT_GT(differs, 0u) << "N " << n << " m " << m << " shift " << wrong;
      }
    }
  }
}

TEST(L6AdapterDelay, ZeroDelayIsBitIdenticalToTheAdapterWithoutTheConfig) {
  for (const std::uint32_t m : {1u, 3u}) {
    RunSpec legacy;
    legacy.legacy_ctor = true;
    legacy.scripted = true;
    legacy.m = m;
    const AdapterRun a = run_adapter(legacy);
    const std::vector<TickOutput> b = run_with_delay(0, m, stub_dshot);
    ASSERT_EQ(a.ticks.size(), b.size());
    for (std::size_t j = 0; j < b.size(); ++j) {
      EXPECT_TRUE(same_bits(a.ticks[j].out, b[j].out));
      EXPECT_TRUE(same_bytes(&a.ticks[j].wrench_enu, &b[j].wrench_enu, sizeof(Wrench)));
      EXPECT_EQ(a.ticks[j].dshot, b[j].dshot);
      EXPECT_EQ(b[j].dshot_applied, b[j].dshot);
    }
  }
}

TEST(L6AdapterDelay, ADelayAboveTheBoundIsRefusedAndTheBoundItselfIsAccepted) {
  int calls = 0;
  ScriptedCommandSource counting([&calls](std::uint64_t j) {
    ++calls;
    return stub_dshot(j);
  });
  AdapterConfig cfg;
  cfg.t_tick_nominal_s = t_nom();
  cfg.command_delay_ticks = kMaxCommandDelayTicks + 1;
  Adapter refused(make_plant(0), counting, cfg);
  const StepResult r = refused.step(states()[0], 0, 1);
  EXPECT_EQ(r.status, Status::kBadDelay);
  EXPECT_TRUE(r.ticks.empty());
  EXPECT_EQ(calls, 0);
  cfg.command_delay_ticks = kMaxCommandDelayTicks;
  Adapter accepted(make_plant(0), counting, cfg);
  EXPECT_EQ(accepted.step(states()[0], 0, 1).status, Status::kOk);
  EXPECT_EQ(calls, 1);
}

// ---- the launch pad (decision 0047) -----------------------------------------------------------------------------------------

namespace pad {

using marv::plant::test::kLat;
using marv::plant::test::kMass;
using marv::plant::test::kSiteHeight;
using marv::plant::test::kThrustCoeff;

using Omega = std::array<double, MARV_PLANT_N_MOTORS>;

// A noiseless IMU so the sample is the truth to the quantiser: no latency, no noise, no bias walk, no turn-on bias. The LSBs
// are labelled test values: 2^-20 rad/s (gyro, word range +-0.5 rad/s, the rates are 0) and 2^-14 m/s^2 (accel, word range
// +-32 m/s^2, as noisy_imu_config; g is 9.8 m/s^2); full scale 100 binds nothing.
marv_plant_imu_config quiet_imu() {
  marv_plant_imu_config c{};
  c.struct_size = sizeof(c);
  c.gyro.lsb = 1.0 / 1048576.0;
  c.gyro.full_scale = 100.0;
  c.accel.lsb = 1.0 / 16384.0;
  c.accel.full_scale = 100.0;
  return c;
}
// The quantiser's half step 2^-15 plus the float rounding of the output at 16 m/s^2 (a half ulp, 2^-20): the most a noiseless
// sample can lie from the truth.
constexpr double kAccelTol = 1.0 / 32768.0 + 1.0 / 1048576.0;

marv_plant* make_pad_plant(const Omega& omega0) {
  marv_plant_config c = marv::plant::test::fixture_config();
  c.motor_substep_s = t_nom();
  for (std::size_t i = 0; i < omega0.size(); ++i) {
    c.initial_omega_rad_s[i] = omega0[i];
  }
  marv_plant* p = nullptr;
  EXPECT_EQ(marv_plant_create(&c, &p), MARV_PLANT_OK);
  return p;
}

// A body at rest: zero velocity and rates, the given attitude and height (NED z = -height above the site).
marv_plant_body rest_body(const std::array<double, 4>& q, double down_m) {
  marv_plant_body b = make_body(q, {0.0, 0.0, 0.0});
  b.vel_ned_m_s[0] = 0.0;
  b.pos_ned_m[2] = down_m;
  return b;
}

// R^T (0, 0, -g), computed from the third row of the body -> NED rotation of the Hamilton quaternion (independent of the
// plant's quaternion code): R[2][0] = 2 (xz - wy), R[2][1] = 2 (yz + wx), R[2][2] = w^2 - x^2 - y^2 + z^2.
std::array<double, 3> down_specific_force(const marv_plant_body& b) {
  const double w = b.q_wxyz[0], x = b.q_wxyz[1], y = b.q_wxyz[2], z = b.q_wxyz[3];
  const double g = marv::prim::normal_gravity<double>(kLat, kSiteHeight - b.pos_ned_m[2]);
  return {-g * 2.0 * (x * z - w * y), -g * 2.0 * (y * z + w * x), -g * (w * w - x * x - y * y + z * z)};
}

double weight_n(const marv_plant_body& b) { return kMass * marv::prim::normal_gravity<double>(kLat, kSiteHeight - b.pos_ned_m[2]); }

struct PadRun {
  std::vector<TickOutput> ticks;
  std::vector<StepResult> steps;
  std::optional<std::uint64_t> release;
};

PadRun run_pad(const Omega& omega0, std::uint32_t m, std::size_t n_ticks, const std::function<Dshot(std::uint64_t)>& script,
               const marv_plant_body& body, bool launch_pad) {
  const marv_plant_imu_config imu = quiet_imu();
  ScriptedCommandSource src(script);
  AdapterConfig cfg;
  cfg.t_tick_nominal_s = t_nom();
  cfg.imu = &imu;
  cfg.launch_pad = launch_pad;
  Adapter adapter(make_pad_plant(omega0), src, cfg);
  EXPECT_EQ(adapter.launch_pad(), launch_pad);
  PadRun run;
  for (std::size_t h = 0; h * m < n_ticks; ++h) {
    StepResult r = adapter.step(body, h * m, m);
    EXPECT_EQ(r.status, Status::kOk);
    if (r.status != Status::kOk) {
      break;
    }
    run.ticks.insert(run.ticks.end(), r.ticks.begin(), r.ticks.end());
    run.steps.push_back(std::move(r));
  }
  run.release = adapter.release_tick();
  return run;
}

// The thrust of the rotor state a tick's IMU sample sees (the state after the previous step, the initial state at tick 0),
// summed from the plant's own reported rotor speeds: k sum omega^2.
double sampled_thrust(const PadRun& run, const Omega& omega0, std::size_t j) {
  double sum = 0.0;
  for (std::size_t i = 0; i < omega0.size(); ++i) {
    const double w = j == 0 ? omega0[i] : run.ticks[j - 1].out.rotor_speed_rad_s[i];
    sum += kThrustCoeff * w * w;
  }
  return sum;
}

// A ramp that crosses the hover thrust (DShot 1 higher every tick, full throttle from tick 1999, the fixture's 48..2047) and is
// cut to the stop command from tick 2500, when the thrust is far above the weight and the release long past.
Dshot ramp_then_stop(std::uint64_t j) {
  if (j >= 2500) {
    return Dshot{};
  }
  const auto d = static_cast<std::uint16_t>(std::min<std::uint64_t>(48 + j, 2047));
  return Dshot{d, d, d, d};
}
constexpr std::size_t kRampTicks = 4500;  // the thrust has decayed to 0 again by here: e^-(2000 ticks / 192 ticks) = 3e-5

}  // namespace pad

TEST(L6AdapterPad, WhileHeldTheImuReadsMinusGAlongTheDownAxisForAnyRotorStateAndAttitude) {
  // Rotor states: at rest; a quarter of the way up, equal; unequal, one rotor nearly stopped. The thrust of each stays below
  // the weight 7.35 N (0.75 kg, 9.8 m/s^2) at the command below, so the pad holds: sum k omega^2 = 0, 5.0 and 2.6 N at the start.
  // Bodies: level at the site, the three tilted attitudes of states() and a different height (g changes with height).
  const std::array<pad::Omega, 3> rotors = {pad::Omega{0.0, 0.0, 0.0, 0.0}, pad::Omega{2500.0, 2500.0, 2500.0, 2500.0},
                                            pad::Omega{1000.0, 2800.0, 300.0, 2000.0}};
  const std::array<marv_plant_body, 5> bodies = {
      pad::rest_body({1.0, 0.0, 0.0, 0.0}, 0.0), pad::rest_body({0.5, 0.5, 0.5, 0.5}, 0.0),
      pad::rest_body({0.6, 0.8, 0.0, 0.0}, 0.0), pad::rest_body({0.0, 1.0, 0.0, 0.0}, 0.0),
      pad::rest_body({1.0, 0.0, 0.0, 0.0}, -5000.0)};
  // A command that moves every motor every tick but stays far below the hover thrust (DShot 48..447: omega_cmd <= 960 rad/s).
  const auto command = [](std::uint64_t j) {
    Dshot d{};
    for (std::size_t k = 0; k < d.size(); ++k) {
      d[k] = static_cast<std::uint16_t>(48 + ((37 * j + 211 * k) % 400));
    }
    return d;
  };
  for (const pad::Omega& omega0 : rotors) {
    for (const marv_plant_body& body : bodies) {
      for (const std::uint32_t m : {1u, 3u}) {
        const pad::PadRun run = pad::run_pad(omega0, m, 240, command, body, true);
        const std::array<double, 3> want = pad::down_specific_force(body);
        ASSERT_EQ(run.ticks.size(), 240u);
        EXPECT_FALSE(run.release.has_value());
        double span = 0.0;  // how far the rotors moved: the held reading did not depend on a constant rotor state
        for (std::size_t j = 0; j < run.ticks.size(); ++j) {
          const TickOutput& t = run.ticks[j];
          EXPECT_TRUE(t.pad_held) << "tick " << j;
          for (std::size_t a = 0; a < 3; ++a) {
            const double got = static_cast<double>(a == 0 ? t.imu.accel_m_s2.x : a == 1 ? t.imu.accel_m_s2.y : t.imu.accel_m_s2.z);
            EXPECT_NEAR(got, want[a], pad::kAccelTol) << "tick " << j << " axis " << a;
          }
          EXPECT_EQ(t.imu.gyro_rad_s.x, 0.0F);
          EXPECT_EQ(t.imu.gyro_rad_s.y, 0.0F);
          EXPECT_EQ(t.imu.gyro_rad_s.z, 0.0F);
          span = std::max(span, std::fabs(pad::sampled_thrust(run, omega0, j) - pad::sampled_thrust(run, omega0, 0)));
        }
        EXPECT_GT(span, 0.1) << "the rotor state did not change over the run";  // thrust moved by > 0.1 N
        // The wrench a held host step applies is exactly zero (forces and torques).
        for (const StepResult& r : run.steps) {
          for (std::size_t c = 0; c < 3; ++c) {
            EXPECT_EQ(r.wrench_enu.force[c], 0.0);
            EXPECT_EQ(r.wrench_enu.torque[c], 0.0);
          }
        }
        // Control: the same run without the pad reads thrust only (nowhere near -g on the down axis for the level body).
        if (body.q_wxyz[0] == 1.0 && body.pos_ned_m[2] == 0.0) {
          const pad::PadRun free_run = pad::run_pad(omega0, m, 240, command, body, false);
          EXPECT_GT(std::fabs(static_cast<double>(free_run.ticks[100].imu.accel_m_s2.z) - want[2]), 1.0);
          EXPECT_FALSE(free_run.ticks[100].pad_held);
        }
      }
    }
  }
}

TEST(L6AdapterPad, ReleasesOnceOnTheFirstTickTheVerticalThrustReachesTheWeightAndNeverRehold) {
  const pad::Omega rest{};
  for (const marv_plant_body& body : {pad::rest_body({1.0, 0.0, 0.0, 0.0}, 0.0), pad::rest_body({1.0, 0.0, 0.0, 0.0}, -5000.0)}) {
    for (const std::uint32_t m : {1u, 4u}) {
      const pad::PadRun run = pad::run_pad(rest, m, pad::kRampTicks, pad::ramp_then_stop, body, true);
      ASSERT_EQ(run.ticks.size(), pad::kRampTicks);
      ASSERT_TRUE(run.release.has_value()) << "m " << m;
      // The expected release tick from the plant's own reported rotor speeds: the first tick whose sampled thrust (level body,
      // so all of it is vertical) reaches m g(h); computed here from the weight alone.
      std::optional<std::size_t> want;
      for (std::size_t j = 0; j < run.ticks.size() && !want; ++j) {
        if (pad::sampled_thrust(run, rest, j) >= pad::weight_n(body)) {
          want = j;
        }
      }
      ASSERT_TRUE(want.has_value());
      EXPECT_EQ(*run.release, *want) << "m " << m;
      EXPECT_GT(*want, 100u);  // not at tick 0: the ramp had to climb
      for (std::size_t j = 0; j < run.ticks.size(); ++j) {
        EXPECT_EQ(run.ticks[j].pad_held, j < *want) << "tick " << j << " m " << m;
      }
      // A tick away from the release the reading is the free vehicle's: just before it is still -g (held) and the thrust there
      // is below the weight; at it the thrust is at or above the weight (the rule, both sides).
      EXPECT_LT(pad::sampled_thrust(run, rest, *want - 1), pad::weight_n(body));
      EXPECT_GE(pad::sampled_thrust(run, rest, *want), pad::weight_n(body));
      // After the stop command the thrust decays below the weight again; the pad does not re-engage.
      EXPECT_LT(pad::sampled_thrust(run, rest, run.ticks.size() - 1), pad::weight_n(body));
      EXPECT_EQ(run.ticks.back().pad_held, false);
      // W_bar of a host step: the held ticks add nothing, the free ones are summed in tick order from the first free one and
      // divided by m.
      for (const StepResult& r : run.steps) {
        Wrench sum{};
        bool have = false;
        for (const TickOutput& t : r.ticks) {
          if (t.pad_held) {
            continue;
          }
          for (std::size_t c = 0; c < 3; ++c) {
            sum.force[c] = have ? sum.force[c] + t.wrench_enu.force[c] : t.wrench_enu.force[c];
            sum.torque[c] = have ? sum.torque[c] + t.wrench_enu.torque[c] : t.wrench_enu.torque[c];
          }
          have = true;
        }
        for (std::size_t c = 0; c < 3; ++c) {
          EXPECT_EQ(r.wrench_enu.force[c], have ? sum.force[c] / static_cast<double>(m) : 0.0);
          EXPECT_EQ(r.wrench_enu.torque[c], have ? sum.torque[c] / static_cast<double>(m) : 0.0);
        }
      }
    }
  }
}

TEST(L6AdapterPad, AfterTheReleaseTheRunIsAPadFreeRunFromTheSameRotorState) {
  const pad::Omega rest{};
  const marv_plant_body body = pad::rest_body({1.0, 0.0, 0.0, 0.0}, 0.0);
  const pad::PadRun pad_run = pad::run_pad(rest, 1, pad::kRampTicks, pad::ramp_then_stop, body, true);
  ASSERT_TRUE(pad_run.release.has_value());
  const std::size_t release = static_cast<std::size_t>(*pad_run.release);
  ASSERT_GT(release, 0u);
  // A pad-free adapter whose plant starts from the rotor speeds after the pad run's last held tick, fed the same command
  // stream from the release tick on.
  pad::Omega from_state{};
  for (std::size_t i = 0; i < from_state.size(); ++i) {
    from_state[i] = pad_run.ticks[release - 1].out.rotor_speed_rad_s[i];
  }
  const marv_plant_imu_config imu = pad::quiet_imu();
  ScriptedCommandSource src(pad::ramp_then_stop);
  AdapterConfig cfg;
  cfg.t_tick_nominal_s = t_nom();
  cfg.imu = &imu;
  Adapter free_adapter(pad::make_pad_plant(from_state), src, cfg);
  std::size_t compared = 0;
  for (std::size_t j = release; j < pad_run.ticks.size(); ++j) {
    const StepResult r = free_adapter.step(body, j, 1);
    ASSERT_EQ(r.status, Status::kOk);
    const TickOutput& a = pad_run.ticks[j];
    const TickOutput& b = r.ticks[0];
    EXPECT_FALSE(a.pad_held);
    EXPECT_TRUE(same_bits(a.out, b.out)) << "tick " << j;
    EXPECT_TRUE(same_bits(a.imu, b.imu)) << "tick " << j;
    EXPECT_TRUE(same_bytes(&a.wrench_enu, &b.wrench_enu, sizeof(Wrench))) << "tick " << j;
    ++compared;
  }
  EXPECT_GT(compared, 1000u);
  // Control: tick 0, held with the rotors at rest, is not the pad-free reading (the pad's contact force is what makes it read -g).
  ScriptedCommandSource src2(pad::ramp_then_stop);
  Adapter free_at_rest(pad::make_pad_plant(rest), src2, cfg);
  const StepResult at_rest = free_at_rest.step(body, 0, 1);
  EXPECT_FALSE(same_bits(pad_run.ticks[0].imu, at_rest.ticks[0].imu));
}

TEST(L6AdapterPad, WithoutThePadEveryOutputIsTheAdapterWithoutTheOption) {
  for (const std::uint32_t m : {1u, 3u}) {
    const marv_plant_imu_config imu = noisy_imu_config();
    RunSpec legacy;
    legacy.imu = &imu;
    legacy.scripted = true;
    legacy.m = m;
    const AdapterRun a = run_adapter(legacy);
    ScriptedCommandSource src(stub_dshot);
    AdapterConfig cfg;
    cfg.t_tick_nominal_s = t_nom();
    cfg.imu = &imu;
    cfg.launch_pad = false;
    Adapter adapter(make_plant(0), src, cfg);
    std::vector<TickOutput> b;
    for (std::size_t h = 0; h * m < kTicks; ++h) {
      const StepResult r = adapter.step(cycling()(h), h * m, m);
      ASSERT_EQ(r.status, Status::kOk);
      b.insert(b.end(), r.ticks.begin(), r.ticks.end());
    }
    EXPECT_FALSE(adapter.release_tick().has_value());
    ASSERT_EQ(a.ticks.size(), b.size());
    for (std::size_t j = 0; j < b.size(); ++j) {
      EXPECT_TRUE(same_bits(a.ticks[j].out, b[j].out));
      EXPECT_TRUE(same_bits(a.ticks[j].imu, b[j].imu));
      EXPECT_TRUE(same_bytes(&a.ticks[j].wrench_enu, &b[j].wrench_enu, sizeof(Wrench)));
      EXPECT_FALSE(b[j].pad_held);
    }
  }
}

}  // namespace
