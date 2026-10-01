// L6 stage (a), the clock-error map (decision 0012, owner decisions 2 and 3): e enters in exactly one place, the sim-side
// true tick time t_nom / (1 + e), which is the adapter's tick time and the dt of marv_plant_step and
// marv_plant_imu_sample. It never reaches the SIL: the SIL's own stamps are checked against the real SIL in sil_test.cpp;
// here the recorder shows the SIL is called with the integer tick numbers whatever e is.
#include <cmath>
#include <limits>

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

}  // namespace
