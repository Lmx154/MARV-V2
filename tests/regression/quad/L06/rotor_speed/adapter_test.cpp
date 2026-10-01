// L6 stage (b), the adapter's opt-in rotor-speed path (decision 0013): the bytes handed to the SIL equal the direct
// rotor-speed sample sequence of a twin plant, in order, for every host-step multiplicity; the rotor sample has the
// freshness of the IMU sample; the path off leaves the adapter as it was. marv_sil_tick and marv_sil_tick_with_rotor_speed
// are replaced here by recorders (the adapter library leaves them to the linker), so the test sees exactly the bytes the SIL
// would get and which of the two entries was called.
//
// Controls that must fail: the sequence shifted by one sample, a one-ulp change of one rotor-speed float, a twin with
// another delay.
#include <cmath>

#include "support.hpp"
#include "../adapter/support.hpp"

namespace {

using namespace marv::gz;
using namespace marv::gz::l6test;
namespace rotor_test = marv::plant::rotor_test;

struct Call {
  bool with_rotor = false;
  std::uint64_t first_tick = 0;
  std::uint32_t k = 0;
  marv_imu_meas imu{};
  marv_rotor_speed_meas rotor{};
};
std::vector<Call> g_calls;

void fill_out(std::uint64_t first_tick, marv_sil_out* out) {
  const marv::gz::Dshot d = stub_dshot(first_tick);
  for (std::size_t i = 0; i < d.size(); ++i) {
    out->dshot[i] = d[i];
  }
  *out->t_us = first_tick;
}

}  // namespace

extern "C" marv_sil_status marv_sil_tick(std::uint64_t first_tick, std::uint32_t k, const marv_imu_meas* imu,
                                         marv_sil_out* out) {
  Call c;
  c.with_rotor = false;
  c.first_tick = first_tick;
  c.k = k;
  c.imu = *imu;
  g_calls.push_back(c);
  fill_out(first_tick, out);
  return MARV_SIL_OK;
}

extern "C" marv_sil_status marv_sil_tick_with_rotor_speed(std::uint64_t first_tick, std::uint32_t k,
                                                          const marv_imu_meas* imu, const marv_rotor_speed_meas* rotor,
                                                          marv_sil_out* out) {
  Call c;
  c.with_rotor = true;
  c.first_tick = first_tick;
  c.k = k;
  c.imu = *imu;
  c.rotor = *rotor;
  g_calls.push_back(c);
  fill_out(first_tick, out);
  return MARV_SIL_OK;
}

namespace {

constexpr std::array<std::uint64_t, 3> kSeeds = {0, 1, 0x9E3779B97F4A7C15ull};  // labelled: any distinct seeds
constexpr std::array<std::uint32_t, 3> kMultiplicities = {1, 3, 4};             // the packet's host-step ticks m
// design/scenario_values.yaml rate_loop_divisor: 2 ticks per rate-loop period; the scenario's delay is one rate-loop period.
constexpr std::uint32_t kLatency = 2;
// Initial rotor speeds, labelled test values: all inside the fixture's [250, 3800] rad/s ESC range and spread (a period of
// 3.2 ms down to 0.34 ms), so that every motor is measured, not stopped, from the first tick, and a swapped motor is seen.
constexpr std::array<double, MARV_PLANT_N_MOTORS> kInitialOmega = {800.0, 1200.0, 1800.0, 2600.0};

// The L6 adapter test plant (fixture, motor sub-step = the nominal tick, seed) with the rotors already turning.
marv_plant* make_rotating_plant(std::uint64_t seed) {
  marv_plant_config c = marv::plant::test::fixture_config();
  c.motor_substep_s = t_nom();
  c.rng_seed = seed;
  for (std::size_t i = 0; i < MARV_PLANT_N_MOTORS; ++i) {
    c.initial_omega_rad_s[i] = kInitialOmega[i];
  }
  marv_plant* p = nullptr;
  EXPECT_EQ(marv_plant_create(&c, &p), MARV_PLANT_OK);
  return p;
}

marv_plant_rotor_speed_config rotor_config(std::uint32_t latency = kLatency, std::uint32_t poles = 14) {
  // pole count 14: sensors/profiles/marv_v2_board_default.yaml rotor_speed.pole_count. The grid, exponent 3 and mantissa 9
  // bits with a period unit of 1 us: Betaflight src/main/drivers/dshot.c at 5a09417ee75e91e81003cf7891bc1f4de86f3a83.
  marv_plant_rotor_speed_config c{};
  c.struct_size = sizeof(c);
  c.pole_count = poles;
  c.latency_ticks = latency;
  c.exponent_bits = 3;
  c.mantissa_bits = 9;
  c.period_unit_s = 1.0e-6;
  return c;
}

struct Spec {
  std::uint64_t seed = 0;
  const marv_plant_imu_config* imu = nullptr;
  const marv_plant_rotor_speed_config* rotor = nullptr;
  std::uint32_t m = 1;
  BodyOf body = cycling();
  bool scripted = false;       // ScriptedCommandSource(stub_dshot)
  bool imu_sil_source = false; // ImuSilCommandSource (the stage (a) source) instead of RotorSilCommandSource
};

struct Result {
  std::vector<TickOutput> ticks;
  double t_tick_s = 0.0;
};

Result run_adapter(const Spec& s) {
  g_calls.clear();
  Result run;
  ScriptedCommandSource scripted(stub_dshot);
  ImuSilCommandSource imu_src;
  RotorSilCommandSource rotor_src;
  CommandSource& src = s.scripted ? static_cast<CommandSource&>(scripted)
                                  : (s.imu_sil_source ? static_cast<CommandSource&>(imu_src)
                                                      : static_cast<CommandSource&>(rotor_src));
  AdapterConfig cfg;
  cfg.t_tick_nominal_s = t_nom();
  cfg.imu = s.imu;
  cfg.rotor_speed = s.rotor;
  Adapter adapter(make_rotating_plant(s.seed), src, cfg);
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

struct Twin {
  std::vector<marv_plant_imu_out> imu;
  std::vector<marv_plant_rotor_speed_out> rotor;
  std::vector<marv_plant_out> out;
};

// Per tick j: IMU sample (when enabled), rotor sample (when enabled), step with stub_dshot(j), all at dt.
Twin run_twin(const Spec& s, double dt) {
  marv_plant* p = make_rotating_plant(s.seed);
  Twin t;
  if (s.imu != nullptr) {
    EXPECT_EQ(marv_plant_imu_attach(p, s.imu), MARV_PLANT_OK);
  }
  if (s.rotor != nullptr) {
    EXPECT_EQ(marv_plant_rotor_speed_attach(p, s.rotor), MARV_PLANT_OK);
  }
  for (std::size_t j = 0; j < kTicks; ++j) {
    const marv_plant_body b = s.body(j / s.m);
    if (s.imu != nullptr) {
      marv_plant_imu_out o{};
      EXPECT_EQ(marv_plant_imu_sample(p, &b, dt, &o), MARV_PLANT_OK);
      t.imu.push_back(o);
    }
    if (s.rotor != nullptr) {
      marv_plant_rotor_speed_out o{};
      EXPECT_EQ(marv_plant_rotor_speed_sample(p, &o), MARV_PLANT_OK);
      t.rotor.push_back(o);
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

bool same_rotor(const marv_rotor_speed_meas& a, const marv_plant_rotor_speed_out& b) {
  static_assert(sizeof(a) == sizeof(b));
  return std::memcmp(&a, &b, sizeof(a)) == 0;
}
bool same_rotor(const marv_plant_rotor_speed_out& a, const marv_plant_rotor_speed_out& b) {
  return std::memcmp(&a, &b, sizeof(a)) == 0;
}

// Ticks i of `seen` whose bytes differ from twin[i + shift].
std::size_t rotor_mismatches(const std::vector<Call>& seen, const std::vector<marv_plant_rotor_speed_out>& twin,
                             std::size_t shift) {
  std::size_t n = 0;
  for (std::size_t i = 0; i + shift < seen.size() && i + shift < twin.size(); ++i) {
    n += same_rotor(seen[i].rotor, twin[i + shift]) ? 0U : 1U;
  }
  return n;
}

TEST(RotorSpeedAdapter, BytesHandedToTheSilAreTheDirectSampleSequenceOfATwinPlant) {
  const marv_plant_imu_config imu = noisy_imu_config();
  const marv_plant_rotor_speed_config rotor = rotor_config();
  for (std::size_t salt = 0; salt < states().size(); ++salt) {
    for (const std::uint64_t seed : kSeeds) {
      for (const std::uint32_t m : kMultiplicities) {
        Spec s;
        s.seed = seed;
        s.imu = &imu;
        s.rotor = &rotor;
        s.m = m;
        s.body = cycling(salt);
        const Result run = run_adapter(s);
        ASSERT_EQ(run.ticks.size(), kTicks);
        ASSERT_EQ(g_calls.size(), kTicks);
        const Twin twin = run_twin(s, run.t_tick_s);
        for (std::size_t j = 0; j < kTicks; ++j) {
          EXPECT_TRUE(g_calls[j].with_rotor) << "tick " << j;
          EXPECT_EQ(g_calls[j].first_tick, j);
          EXPECT_EQ(g_calls[j].k, 1U);
          EXPECT_TRUE(same_rotor(g_calls[j].rotor, twin.rotor[j])) << "salt " << salt << " seed " << seed << " m " << m << " tick " << j;
          EXPECT_TRUE(same_bits(g_calls[j].imu, twin.imu[j])) << "tick " << j;
          EXPECT_TRUE(same_rotor(run.ticks[j].rotor_speed, twin.rotor[j]));
          EXPECT_TRUE(same_bits(run.ticks[j].out, twin.out[j]));
        }

        // Non-vacuous: the samples move over the run (the rotors follow stub_dshot) and are measured, not stopped.
        EXPECT_FALSE(same_rotor(twin.rotor[0], twin.rotor[kTicks - 1]));
        EXPECT_EQ(twin.rotor[kTicks - 1].flags, 0xFU);
        // Control: the sequence shifted by one sample does not match.
        EXPECT_GT(rotor_mismatches(g_calls, twin.rotor, 1), 0U);
        // Control: one ulp of one float of one tick's sample is seen (the comparison is bit-exact).
        std::vector<marv_plant_rotor_speed_out> nudged = twin.rotor;
        nudged[kTicks - 1].omega_rad_s[2] = std::nextafter(nudged[kTicks - 1].omega_rad_s[2], 0.0F);
        EXPECT_EQ(rotor_mismatches(g_calls, nudged, 0), 1U);
        // Control: a twin with another delay does not match.
        const marv_plant_rotor_speed_config other = rotor_config(kLatency + 1);
        Spec t = s;
        t.rotor = &other;
        EXPECT_GT(rotor_mismatches(g_calls, run_twin(t, run.t_tick_s).rotor, 0), 0U);
      }
    }
  }
}

TEST(RotorSpeedAdapter, TheRotorSampleOfTickJIsTheRotorStateAfterStepJMinusOne) {
  // Delay 0: sample j is the grid reading of the plant's own speeds after step j - 1 (the initial speeds at the first tick).
  const marv_plant_rotor_speed_config rotor = rotor_config(0);
  Spec s;
  s.rotor = &rotor;
  s.m = 3;
  const Result run = run_adapter(s);
  ASSERT_EQ(run.ticks.size(), kTicks);
  ASSERT_EQ(g_calls.size(), kTicks);
  std::size_t one_tick_fresher_matches = 0;  // control: the reading of the state after step j
  for (std::size_t j = 0; j < kTicks; ++j) {
    for (std::size_t i = 0; i < MARV_PLANT_N_MOTORS; ++i) {
      const double before = j == 0 ? kInitialOmega[i] : run.ticks[j - 1].out.rotor_speed_rad_s[i];
      const rotor_test::Reading want = rotor_test::oracle(before, rotor.pole_count);
      EXPECT_EQ(std::memcmp(&g_calls[j].rotor.omega_rad_s[i], &want.omega, sizeof(float)), 0) << "tick " << j << " motor " << i;
      const rotor_test::Reading newer = rotor_test::oracle(run.ticks[j].out.rotor_speed_rad_s[i], rotor.pole_count);
      one_tick_fresher_matches += std::memcmp(&g_calls[j].rotor.omega_rad_s[i], &newer.omega, sizeof(float)) == 0 ? 1U : 0U;
    }
    EXPECT_EQ(g_calls[j].rotor.flags, 0xFU);
  }
  EXPECT_LT(one_tick_fresher_matches, kTicks * MARV_PLANT_N_MOTORS);
}

TEST(RotorSpeedAdapter, RotorPathWithoutTheImuPassesAZeroedImuSample) {
  const marv_plant_rotor_speed_config rotor = rotor_config();
  Spec s;
  s.seed = 5;
  s.rotor = &rotor;
  s.m = 3;
  const Result run = run_adapter(s);
  ASSERT_EQ(g_calls.size(), kTicks);
  const Twin twin = run_twin(s, run.t_tick_s);
  const marv_imu_meas zero{};
  for (std::size_t j = 0; j < kTicks; ++j) {
    EXPECT_TRUE(g_calls[j].with_rotor);
    EXPECT_TRUE(same_bytes(&g_calls[j].imu, &zero, sizeof(zero)));
    EXPECT_TRUE(same_rotor(g_calls[j].rotor, twin.rotor[j]));
  }
}

TEST(RotorSpeedAdapter, OffTheAdapterIsAsBeforeAndTheModelNeverChangesTheStep) {
  const marv_plant_imu_config imu = noisy_imu_config();
  const marv_plant_rotor_speed_config rotor = rotor_config();
  for (const std::uint32_t m : kMultiplicities) {
    // IMU only, the stage (a) source: the plain entry, no rotor bytes, no rotor output.
    Spec off;
    off.seed = 3;
    off.imu = &imu;
    off.m = m;
    off.imu_sil_source = true;
    const Result a = run_adapter(off);
    ASSERT_EQ(g_calls.size(), kTicks);
    const Twin twin = run_twin(off, a.t_tick_s);
    const marv_plant_rotor_speed_out zero{};
    for (std::size_t j = 0; j < kTicks; ++j) {
      EXPECT_FALSE(g_calls[j].with_rotor);
      EXPECT_TRUE(same_bits(g_calls[j].imu, twin.imu[j]));
      EXPECT_TRUE(same_bits(a.ticks[j].out, twin.out[j]));
      EXPECT_TRUE(same_rotor(a.ticks[j].rotor_speed, zero));
    }
    // The same run with the rotor-speed path on: the step outputs and the IMU bytes are bit-identical to the run without.
    Spec on = off;
    on.rotor = &rotor;
    on.imu_sil_source = false;
    const Result b = run_adapter(on);
    ASSERT_EQ(g_calls.size(), kTicks);
    for (std::size_t j = 0; j < kTicks; ++j) {
      EXPECT_TRUE(g_calls[j].with_rotor);
      EXPECT_TRUE(same_bits(g_calls[j].imu, twin.imu[j]));
      EXPECT_TRUE(same_bits(b.ticks[j].out, a.ticks[j].out));
      EXPECT_TRUE(same_bits(b.ticks[j].imu, a.ticks[j].imu));
    }
    // A source that does not take the rotor sample (the scripted one) leaves the dshot to its script, and the plant too.
    Spec scripted = on;
    scripted.scripted = true;
    const Result c = run_adapter(scripted);
    EXPECT_TRUE(g_calls.empty());
    for (std::size_t j = 0; j < kTicks; ++j) {
      EXPECT_TRUE(same_bits(c.ticks[j].out, a.ticks[j].out));
      EXPECT_EQ(c.ticks[j].rotor_speed.flags, 0xFU);
    }
  }
}

TEST(RotorSpeedAdapter, ABadConfigIsReportedAsAPlantFailureAndNothingRuns) {
  marv_plant_rotor_speed_config rotor = rotor_config();
  rotor.pole_count = 13;  // odd
  g_calls.clear();
  ScriptedCommandSource src(stub_dshot);
  AdapterConfig cfg;
  cfg.t_tick_nominal_s = t_nom();
  cfg.rotor_speed = &rotor;
  Adapter adapter(make_rotating_plant(0), src, cfg);
  EXPECT_TRUE(adapter.rotor_speed_enabled());
  EXPECT_EQ(adapter.rotor_speed_attach_status(), MARV_PLANT_E_CONFIG);
  const StepResult r = adapter.step(states()[0], 0, 1);
  EXPECT_EQ(r.status, Status::kPlant);
  EXPECT_EQ(r.plant_status, MARV_PLANT_E_CONFIG);
  EXPECT_TRUE(r.ticks.empty());
  EXPECT_TRUE(g_calls.empty());
}

}  // namespace
