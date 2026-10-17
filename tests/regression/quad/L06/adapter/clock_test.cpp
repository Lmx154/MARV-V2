// L6 stage (a), the clock-error map (decision 0012, owner decisions 2 and 3): e enters in exactly one place, the sim-side
// true tick time t_nom / (1 + e), which is the adapter's tick time and the dt of marv_plant_step and
// marv_plant_imu_sample. It never reaches the SIL: the SIL's own stamps are checked against the real SIL in sil_test.cpp;
// here the recorder shows the SIL is called with the integer tick numbers whatever e is.
//
// The command delay (decision 0046 item 2), at the end of this file: the plant is stepped at tick j with the source's command
// of tick j - N and the stop command (DShot 0) before that; N = 0 is the adapter without it, bitwise; an N above the bound is
// refused.
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

}  // namespace
