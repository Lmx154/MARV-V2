// L6 stage (a), the adapter's opt-in IMU path (decision 0012): the bytes handed to the SIL are the plant's samples
// verbatim and in order; the per-tick IMU is independent of how ticks are batched into host steps; the IMU never
// changes marv_plant_step. marv_sil_tick is replaced here by a recorder (the adapter library leaves it to the linker),
// so the test sees exactly the bytes the SIL would get.
//
// Every claim has a control that must fail: a sample index shifted by one, a different seed, a one-ulp body change.
#include <cmath>

#include "support.hpp"

extern "C" marv_sil_status marv_sil_tick(std::uint64_t first_tick, std::uint32_t k, const marv_imu_meas* imu,
                                         marv_sil_out* out) {
  using namespace marv::gz::l6test;
  g_rec.first_ticks.push_back(first_tick);
  g_rec.counts.push_back(k);
  if (imu == nullptr) {
    ++g_rec.null_imu;
  } else {
    g_rec.seen.push_back(*imu);
  }
  const marv::gz::Dshot d = stub_dshot(first_tick);
  for (std::size_t i = 0; i < d.size(); ++i) {
    out->dshot[i] = d[i];
  }
  *out->t_us = first_tick;
  return MARV_SIL_OK;
}

namespace {

using namespace marv::gz;
using namespace marv::gz::l6test;

constexpr std::array<std::uint64_t, 3> kSeeds = {0, 1, 0x9E3779B97F4A7C15ull};  // labelled: any distinct seeds
constexpr std::array<std::uint32_t, 3> kMultiplicities = {1, 3, 4};             // the packet's host-step ticks m

// Counts the ticks i whose recorded sample differs from twin[i + shift].
std::size_t mismatches(const std::vector<marv_imu_meas>& seen, const std::vector<marv_plant_imu_out>& twin,
                       std::size_t shift) {
  std::size_t n = 0;
  for (std::size_t i = 0; i + shift < seen.size() && i + shift < twin.size(); ++i) {
    n += same_bits(seen[i], twin[i + shift]) ? 0u : 1u;
  }
  return n;
}

TEST(L6AdapterImu, BytesHandedToTheSilAreTheDirectSampleSequenceOfATwinPlant) {
  const marv_plant_imu_config cfg = noisy_imu_config();
  for (std::size_t salt = 0; salt < states().size(); ++salt) {
    for (const std::uint64_t seed : kSeeds) {
      for (const std::uint32_t m : kMultiplicities) {
        RunSpec s;
        s.seed = seed;
        s.imu = &cfg;
        s.m = m;
        s.body = cycling(salt);
        const AdapterRun run = run_adapter(s);
        ASSERT_EQ(run.ticks.size(), kTicks);
        ASSERT_EQ(g_rec.seen.size(), kTicks);
        EXPECT_EQ(g_rec.null_imu, 0u);
        const TwinRun twin = run_twin(seed, &cfg, run.t_tick_s, m, s.body);
        EXPECT_EQ(mismatches(g_rec.seen, twin.imu, 0), 0u) << "salt " << salt << " seed " << seed << " m " << m;
        for (std::size_t j = 0; j < kTicks; ++j) {
          EXPECT_TRUE(same_bits(run.ticks[j].imu, twin.imu[j]));
          EXPECT_EQ(g_rec.first_ticks[j], j);
          EXPECT_EQ(g_rec.counts[j], 1u);
        }

        // Control: the sequence shifted by one sample does not match (the noise and the moving rotor state make
        // consecutive samples differ).
        EXPECT_GT(mismatches(g_rec.seen, twin.imu, 1), 0u);
        // Control: a twin of another seed does not match.
        const TwinRun other = run_twin(seed + 1, &cfg, run.t_tick_s, m, s.body);
        EXPECT_GT(mismatches(g_rec.seen, other.imu, 0), 0u);
      }
    }
  }
}

// A zero-noise config, LSB 2^-3 rad/s, so a rate of exactly half an LSB (0.0625) is a rounding tie: one ulp above it
// rounds to count 1 and one ulp below to count 0, whatever the tie rule, so a one-ulp change of the body is visible.
TEST(L6AdapterImu, ABodyChangeOfOneUlpChangesTheBytes) {
  marv_plant_imu_config cfg{};
  cfg.struct_size = sizeof(cfg);
  cfg.gyro.lsb = 0.125;  // 2^-3, binary-exact
  cfg.gyro.full_scale = 100.0;
  cfg.accel.lsb = 0.0625;  // 2^-4
  cfg.accel.full_scale = 100.0;
  const double tie = 0.0625;  // half the gyro LSB
  const auto body_with = [&](double wx) {
    return [wx](std::size_t) {
      marv_plant_body b = states()[0];
      b.omega_frd_rad_s[0] = wx;
      return b;
    };
  };
  const double up = std::nextafter(tie, 1.0);
  const double down = std::nextafter(tie, 0.0);
  RunSpec s;
  s.imu = &cfg;
  s.m = 3;
  s.body = body_with(up);
  const AdapterRun run = run_adapter(s);
  ASSERT_EQ(g_rec.seen.size(), kTicks);
  const TwinRun same = run_twin(0, &cfg, run.t_tick_s, 3, body_with(up));
  EXPECT_EQ(mismatches(g_rec.seen, same.imu, 0), 0u);
  EXPECT_EQ(g_rec.seen[0].gyro_rad_s.x, 0.125F);
  // Control: the twin fed the body one ulp lower gives other bytes on every tick.
  const TwinRun lower = run_twin(0, &cfg, run.t_tick_s, 3, body_with(down));
  EXPECT_EQ(mismatches(g_rec.seen, lower.imu, 0), kTicks);
  EXPECT_EQ(lower.imu[0].gyro_rad_s.x, 0.0F);
}

TEST(L6AdapterImu, PerTickImuIsIdenticalForOneTwoAndFourTicksPerHostStep) {
  const marv_plant_imu_config cfg = noisy_imu_config();
  const auto constant = [](std::size_t) { return states()[1]; };
  std::vector<AdapterRun> runs;
  std::vector<std::vector<marv_imu_meas>> seen;
  for (const std::uint32_t m : {1u, 2u, 4u}) {
    RunSpec s;
    s.seed = kSeeds[1];
    s.imu = &cfg;
    s.m = m;
    s.body = constant;
    runs.push_back(run_adapter(s));
    seen.push_back(g_rec.seen);
  }
  for (std::size_t r = 1; r < runs.size(); ++r) {
    ASSERT_EQ(runs[r].ticks.size(), kTicks);
    for (std::size_t j = 0; j < kTicks; ++j) {
      EXPECT_TRUE(same_bits(runs[r].ticks[j].imu, runs[0].ticks[j].imu)) << "run " << r << " tick " << j;
      EXPECT_TRUE(same_bits(seen[r][j], runs[0].ticks[j].imu));
      EXPECT_TRUE(same_bits(runs[r].ticks[j].out, runs[0].ticks[j].out));
    }
  }
  // Control: when the state changes every host step the batching shows (the state is held over a host step's ticks), so
  // m = 1 and m = 2 differ at some tick: the equality above is not vacuous.
  RunSpec a;
  a.seed = kSeeds[1];
  a.imu = &cfg;
  a.m = 1;
  const AdapterRun one = run_adapter(a);
  a.m = 2;
  const AdapterRun two = run_adapter(a);
  std::size_t differing = 0;
  for (std::size_t j = 0; j < kTicks; ++j) {
    differing += same_bits(one.ticks[j].imu, two.ticks[j].imu) ? 0u : 1u;
  }
  EXPECT_GT(differing, 0u);
}

TEST(L6AdapterImu, PlantStepBitsAreIdenticalWithAndWithoutTheImuAndAcrossSeeds) {
  const marv_plant_imu_config cfg = noisy_imu_config();
  for (const std::uint32_t m : kMultiplicities) {
    RunSpec base;
    base.m = m;
    base.scripted = true;  // no IMU path at all
    const AdapterRun without = run_adapter(base);
    ASSERT_EQ(without.ticks.size(), kTicks);
    for (const std::uint64_t seed : kSeeds) {
      RunSpec s;
      s.seed = seed;
      s.imu = &cfg;
      s.m = m;
      const AdapterRun with = run_adapter(s);
      ASSERT_EQ(with.ticks.size(), kTicks);
      for (std::size_t j = 0; j < kTicks; ++j) {
        EXPECT_TRUE(same_bits(with.ticks[j].out, without.ticks[j].out)) << "seed " << seed << " m " << m << " tick " << j;
        EXPECT_TRUE(same_bits(with.ticks[j].wrench_enu.force[0], without.ticks[j].wrench_enu.force[0]));
        EXPECT_EQ(with.ticks[j].dshot, without.ticks[j].dshot);
      }
    }
  }
  // Control: the comparison sees a difference when the step really differs (a step at another dt).
  RunSpec skewed;
  skewed.m = 1;
  skewed.scripted = true;
  skewed.corner = 1;
  skewed.odr_error = kOdrError;
  RunSpec base;
  base.m = 1;
  base.scripted = true;
  const AdapterRun a = run_adapter(skewed);
  const AdapterRun b = run_adapter(base);
  std::size_t differing = 0;
  for (std::size_t j = 0; j < kTicks; ++j) {
    differing += same_bits(a.ticks[j].out, b.ticks[j].out) ? 0u : 1u;
  }
  EXPECT_GT(differing, 0u);
}

TEST(L6AdapterImu, TheImuIsOffByDefaultAndTheSourceGetsNoSample) {
  RunSpec s;
  s.m = 4;
  const AdapterRun run = run_adapter(s);  // ImuSilCommandSource, imu config null
  ASSERT_EQ(g_rec.seen.size(), kTicks);
  const marv_imu_meas zero{};
  for (std::size_t j = 0; j < kTicks; ++j) {
    EXPECT_TRUE(same_bytes(&g_rec.seen[j], &zero, sizeof(zero)));
    EXPECT_TRUE(same_bits(run.ticks[j].imu, marv_plant_imu_out{}));
  }
}

TEST(L6AdapterImu, ARefusedAttachIsReportedAndNothingRuns) {
  marv_plant_imu_config bad = noisy_imu_config();
  bad.latency_samples = MARV_PLANT_IMU_MAX_LATENCY_SAMPLES + 1;
  ImuSilCommandSource src;
  AdapterConfig cfg;
  cfg.t_tick_nominal_s = t_nom();
  cfg.imu = &bad;
  Adapter adapter(make_plant(0), src, cfg);
  EXPECT_EQ(adapter.imu_attach_status(), MARV_PLANT_E_CONFIG);
  g_rec = Recorder{};
  const StepResult r = adapter.step(states()[0], 0, 1);
  EXPECT_EQ(r.status, Status::kPlant);
  EXPECT_EQ(r.plant_status, MARV_PLANT_E_CONFIG);
  EXPECT_TRUE(g_rec.seen.empty());
}

}  // namespace
