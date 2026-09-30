// The L4 rate-loop bypass (quad spec 4 L5, decision 0006 "A"): execute_bypass skips the reference-model prefilter and
// nothing else.
//   I-A1  acro is bit-identical: execute reproduces a golden generated at tag quad-L4-pass (control: kp one ulp up).
//   I-A2  the bypass skips only the prefilter: two loops, one input stream, bit-equal at every execution
//         (control: the reference perturbed by one ulp at one execution).
//   I-A3  the switch back is bumpless: execute after bypassed executions continues its prefilter from the last
//         applied reference (control: the prefilter seeded from the gyro instead).
//   I-A4  faults and the period check behave as in execute (control: a run without a fault must not satisfy the check).
//
// Numbers in this file are one of: read from the fixture files, derived (the rule is stated), or a "scenario test
// value" named with its reason.
#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include <marv/mixer/mixer.hpp>
#include <marv/prim/constants.hpp>
#include <marv/prim/vec.hpp>
#include <marv/rate/rate_loop.hpp>
#include <marv/types/imu_sample.hpp>

#include "acro_scenario.hpp"

namespace {

using namespace marv;
using marv::l5_rate_bypass::from_bits;
using marv::l5_rate_bypass::to_bits;
using marv::rate::RateConfig;
using marv::rate::RateLoop;
using marv::rate::RateOutput;
using V3 = prim::Vec3<float>;

constexpr std::size_t kA = rate::kTorqueAxes;
constexpr float kMaxFloat = std::numeric_limits<float>::max();
constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
constexpr std::size_t kNone = static_cast<std::size_t>(-1);

std::string reference_path(const char* name) { return std::string(MARV_L5_RATE_BYPASS_DIR) + "/" + name; }

// ---- I-A1: acro is bit-identical to quad-L4-pass ----------------------------------------------------------------

struct Acro {
  RateConfig<float> cfg;
  std::vector<l5_rate_bypass::Step> steps;
  std::string golden;
};

const Acro* acro() {
  static const Acro* built = []() -> const Acro* {
    static Acro a;
    if (!l5_rate_bypass::read_config(reference_path("rate_bypass_inputs.txt"), a.cfg) ||
        !l5_rate_bypass::read_fixture(reference_path("rate_bypass_fixture.txt"), a.steps)) {
      return nullptr;
    }
    a.golden = l5_rate_bypass::golden_body(l5_rate_bypass::read_file(reference_path("rate_bypass_acro_golden.txt")));
    return a.golden.empty() ? nullptr : &a;
  }();
  return built;
}

#define REQUIRE_ACRO()                                               \
  const Acro* const ap = acro();                                     \
  ASSERT_NE(ap, nullptr) << "reading the rate_bypass reference files"; \
  const Acro& acro_ref = *ap

TEST(L5RateBypassAcro, ExecuteReproducesTheQuadL4PassGoldenBitForBit) {
  REQUIRE_ACRO();
  EXPECT_EQ(l5_rate_bypass::run_acro(acro_ref.cfg, acro_ref.steps), acro_ref.golden);
}

TEST(L5RateBypassAcro, TheGoldenExercisesFaultsAndTheIntegrator) {
  REQUIRE_ACRO();
  // The golden must not be a trivial all-zero stream: it has fault rows (the fixture scripts them) and a nonzero
  // integrator. Rows: n u0 u1 u2 fault_active fault_latched fault_count i0 i1 i2.
  const std::vector<std::string> t = l5_rate_bypass::tokens_of(acro_ref.golden);
  constexpr std::size_t kColumns = 10;
  ASSERT_EQ(t.size(), acro_ref.steps.size() * kColumns);
  std::size_t faults = 0;
  std::size_t nonzero_integrator = 0;
  for (std::size_t n = 0; n < acro_ref.steps.size(); ++n) {
    faults += t[n * kColumns + 4] == "1" ? 1U : 0U;
    nonzero_integrator += t[n * kColumns + 7] != "0x00000000" ? 1U : 0U;
  }
  EXPECT_GT(faults, 0U);
  EXPECT_GT(nonzero_integrator, 0U);
}

TEST(L5RateBypassAcro, NegativeControlKpOneUlpUpDoesNotReproduceTheGolden) {
  REQUIRE_ACRO();
  for (std::size_t a = 0; a < kA; ++a) {
    RateConfig<float> cfg = acro_ref.cfg;
    cfg.kp[a] = std::nextafter(cfg.kp[a], std::numeric_limits<float>::infinity());
    EXPECT_NE(l5_rate_bypass::run_acro(cfg, acro_ref.steps), acro_ref.golden) << "axis " << a;
  }
}

// ---- the rig shared by I-A2 to I-A4 -----------------------------------------------------------------------------

// Scenario test value: the design period T. One millisecond, a whole number of microseconds, so the +-1 us jitter
// cases are exact (as the L04 unit suite).
constexpr std::uint64_t kPeriodUs = 1000;
// Scenario test value: the first sample stamp, nonzero so a stamp of 0 is not mistaken for "no stamp".
constexpr std::uint64_t kT0Us = 5000;
// Scenario test values: gains as the L04 hand table (small numbers, per-axis scale so an axis mix-up shows), nonzero
// kd so the D path runs, and a prefilter time constant of ten periods so alpha is about 0.1, neither 0 nor 1.
constexpr float kBaseKp = 2;
constexpr float kBaseKi = 100;
constexpr float kBaseKd = 0.004F;
constexpr float kTauPeriods = 10;

float period_s() { return static_cast<float>(kPeriodUs) / static_cast<float>(prim::kMicrosecondsPerSecond); }

RateConfig<float> test_config() {
  RateConfig<float> c;
  for (std::size_t a = 0; a < kA; ++a) {
    const float scale = static_cast<float>(a + 1);
    c.kp[a] = kBaseKp * scale;
    c.ki[a] = kBaseKi * scale;
    c.kd[a] = kBaseKd * scale;
    c.tau_ref[a] = kTauPeriods * period_s();
  }
  c.period = period_s();
  return c;
}

// xorshift64*: integer arithmetic only, so the stream is the same on every host and standard library.
struct Rng {
  std::uint64_t x;
  std::uint64_t next() {
    x ^= x >> 12;
    x ^= x << 25;
    x ^= x >> 27;
    return x * 0x2545F4914F6CDD1DULL;
  }
  // Uniform in [-1, 1): 24 random bits, exact in binary32.
  float uniform() { return static_cast<float>(next() >> 40) / static_cast<float>(1U << 23) - 1.0F; }
};

// Scenario test values: the stream seed (any nonzero) and the rate scale of the random gyro and references, rad/s.
constexpr std::uint64_t kSeed = 0x4D41525635524154ULL;
constexpr float kRateScale = 1;

V3 random_v3(Rng& rng) { return V3(kRateScale * rng.uniform(), kRateScale * rng.uniform(), kRateScale * rng.uniform()); }

// The prefilter of the header, in the same float operations (alpha from dt = float(dt_us) / float(1e6), then
// r + alpha (sp - r)).
V3 prefilter(const RateConfig<float>& cfg, const V3& r, const V3& sp, std::uint64_t dt_us) {
  const float dt = static_cast<float>(dt_us) / static_cast<float>(prim::kMicrosecondsPerSecond);
  V3 out;
  for (std::size_t a = 0; a < kA; ++a) {
    const float alpha = 1.0F - std::exp(-dt / cfg.tau_ref[a]);
    out[a] = r[a] + alpha * (sp[a] - r[a]);
  }
  return out;
}

struct Rig {
  RateLoop<float> loop;
  std::uint64_t t = kT0Us;
  std::uint64_t last_dt_us = 0;
  bool first = true;

  explicit Rig(const RateConfig<float>& cfg) { loop.init(cfg, mixer::MixerConfig<float>()); }

  // The stamp of the next execution: T + jitter after the previous one, or kT0Us for the first.
  ImuSample sample(const V3& y, std::int64_t jitter_us, bool gyro_valid = true) {
    if (!first) {
      last_dt_us = static_cast<std::uint64_t>(static_cast<std::int64_t>(kPeriodUs) + jitter_us);
      t += last_dt_us;
    }
    first = false;
    ImuSample s{};
    s.t_us = t;
    s.gyro_rad_s = y;
    s.flags = gyro_valid ? imu_flag(ImuFlag::GyroValid) : 0U;
    return s;
  }
};

bool bits_equal(const V3& a, const V3& b) {
  return to_bits(a[0]) == to_bits(b[0]) && to_bits(a[1]) == to_bits(b[1]) && to_bits(a[2]) == to_bits(b[2]);
}

bool same_state(const RateOutput<float>& oa, const RateLoop<float>& la, const RateOutput<float>& ob,
                const RateLoop<float>& lb) {
  return bits_equal(oa.torque, ob.torque) && oa.fault_active == ob.fault_active &&
         oa.fault_latched == ob.fault_latched && oa.fault_count == ob.fault_count &&
         bits_equal(la.integrator(), lb.integrator()) && la.fault_latched() == lb.fault_latched() &&
         la.fault_count() == lb.fault_count();
}

// Both loops record the same scripted allocation after every execution: flags from the stream, requested = the
// execution's own torque, achieved = half of it where the flag is set (a shortfall far above the rounding bound) and
// equal to it elsewhere. Saturating allocations then freeze the integrator of a loop whose error has the same sign.
constexpr float kShortfall = 0.5F;  // scenario test value

void record(RateLoop<float>& loop, const V3& torque, Rng& rng) {
  mixer::Allocation<float> al;
  const bool flag[kA] = {rng.uniform() > 0, rng.uniform() > 0, rng.uniform() > 0};
  for (std::size_t a = 0; a < kA; ++a) {
    al.achieved_torque[a] = flag[a] ? kShortfall * torque[a] : torque[a];
  }
  al.flags.roll = flag[0];
  al.flags.pitch = flag[1];
  al.flags.yaw = flag[2];
  loop.record_allocation(torque, al);
}

// ---- I-A2: the bypass skips only the prefilter ------------------------------------------------------------------

// Scenario test values: execution count (several freeze episodes and both fault kinds), the execution that gets a NaN
// setpoint and the one with GyroValid cleared.
constexpr std::size_t kPairExecutions = 400;
constexpr std::size_t kPairNanSetpointAt = 150;
constexpr std::size_t kPairGyroInvalidAt = 250;

// Runs loop A (execute) and loop B (execute_bypass with r from the prefilter formula) on one input stream. Returns the
// first execution at which any output or state differs, or kNone. perturb_at: the execution whose reference is
// moved one ulp up on every axis before it reaches B.
std::size_t run_pair(std::size_t perturb_at) {
  const RateConfig<float> cfg = test_config();
  Rig a(cfg);
  Rig b(cfg);
  Rng in{kSeed};
  Rng alloc_a{kSeed + 1};
  Rng alloc_b{kSeed + 1};
  V3 r;
  bool seed = true;
  for (std::size_t n = 0; n < kPairExecutions; ++n) {
    const V3 y = random_v3(in);
    V3 sp = random_v3(in);
    // Scenario test values: jitter of -1, 0 or +1 us, drawn from the stream.
    const std::int64_t jitter = static_cast<std::int64_t>(in.next() % 3) - 1;
    const bool valid = n != kPairGyroInvalidAt;
    if (n == kPairNanSetpointAt) {
      sp[1] = kNaN;
    }
    const ImuSample sa = a.sample(y, jitter, valid);
    const ImuSample sb = b.sample(y, jitter, valid);
    r = seed ? y : prefilter(cfg, r, sp, a.last_dt_us);
    V3 ref = r;
    if (n == perturb_at) {
      // All three axes: a one-ulp change of a small r can vanish in e = r - y, never in all three at once.
      for (std::size_t k = 0; k < kA; ++k) {
        ref[k] = std::nextafter(ref[k], std::numeric_limits<float>::infinity());
      }
    }
    const RateOutput<float> oa = a.loop.execute(sa, sp);
    const RateOutput<float> ob = b.loop.execute_bypass(sb, ref);
    if (!same_state(oa, a.loop, ob, b.loop)) {
      return n;
    }
    seed = oa.fault_active;  // a failed check resets the loop: its next execution is a seed
    record(a.loop, oa.torque, alloc_a);
    record(b.loop, ob.torque, alloc_b);
  }
  return kNone;
}

TEST(L5RateBypassOnlyPrefilter, BypassWithThePrefilterFormulaIsBitEqualToExecuteAtEveryExecution) {
  EXPECT_EQ(run_pair(kNone), kNone);
}

TEST(L5RateBypassOnlyPrefilter, NegativeControlOneUlpOnOneReferenceBreaksTheEquality) {
  EXPECT_EQ(run_pair(kNone), kNone);  // the baseline passes
  // One ulp of r is below the rounding of the torque at many executions (kp e is small against the D term), so the
  // control scans the executions and requires that a perturbation at some one of them breaks the equality, and that
  // nothing differs before the perturbed execution.
  std::size_t breaking = 0;
  for (std::size_t n = 1; n < kPairExecutions; ++n) {
    const std::size_t first = run_pair(n);
    if (first != kNone) {
      ++breaking;
      EXPECT_GE(first, n) << "perturbed at " << n;  // the stored error reaches the integrator one execution later
    }
  }
  EXPECT_GT(breaking, 0U);
}

// ---- I-A3: the switch back is bumpless --------------------------------------------------------------------------

// Scenario test values: bypassed executions after the seed, and executions after the switch back.
constexpr std::size_t kBypassed = 3;
constexpr std::size_t kAfterSwitch = 4;

// X: seed, kBypassed bypassed executions, then kAfterSwitch x execute(sp). Z: the same history, then execute_bypass
// with the prefilter formula started from r_last (bumpless) or from the last gyro (what a reset would give).
// Returns whether Z equals X at every execution after the switch.
bool switch_matches(bool model_seeded_from_gyro) {
  const RateConfig<float> cfg = test_config();
  Rig x(cfg);
  Rig z(cfg);
  Rng in{kSeed};
  V3 r_last;
  V3 y_last;
  for (std::size_t n = 0; n <= kBypassed; ++n) {
    const V3 y = random_v3(in);
    const V3 ref = random_v3(in);
    (void)x.loop.execute_bypass(x.sample(y, 0), ref);
    (void)z.loop.execute_bypass(z.sample(y, 0), ref);
    r_last = ref;  // the seed execution (n == 0) does not apply it; the last one (n == kBypassed) does
    y_last = y;
  }
  V3 r = model_seeded_from_gyro ? y_last : r_last;
  bool equal = true;
  for (std::size_t n = 0; n < kAfterSwitch; ++n) {
    const V3 y = random_v3(in);
    const V3 sp = random_v3(in);
    const ImuSample sx = x.sample(y, 0);
    const ImuSample sz = z.sample(y, 0);
    r = prefilter(cfg, r, sp, x.last_dt_us);
    const RateOutput<float> ox = x.loop.execute(sx, sp);
    const RateOutput<float> oz = z.loop.execute_bypass(sz, r);
    equal = equal && same_state(ox, x.loop, oz, z.loop);
  }
  return equal;
}

TEST(L5RateBypassSwitchBack, ExecuteAfterBypassContinuesItsPrefilterFromTheLastAppliedReference) {
  EXPECT_TRUE(switch_matches(false));
}

TEST(L5RateBypassSwitchBack, NegativeControlAPrefilterSeededFromTheGyroDiffers) {
  EXPECT_TRUE(switch_matches(false));  // the baseline passes
  EXPECT_FALSE(switch_matches(true));
}

// ---- I-A4: faults and the period check --------------------------------------------------------------------------

enum class Bad { None, ReferenceNanRoll, ReferenceNanPitch, ReferenceNanYaw, GyroInvalid, OutputNotFinite };

// Runs the case through execute_bypass and checks the response of decision 0005 "Fault extension": zero torque,
// fault_active, the latch, the count, the state reset, and a seed at the next execution (zero torque), which leaves the
// loop equal to a fresh one seeded at the same sample. Returns whether every check held.
bool fault_response_holds(Bad bad) {
  RateConfig<float> cfg = test_config();
  if (bad == Bad::OutputNotFinite) {
    cfg.kp[0] = kMaxFloat;  // scenario test value: any error above 1 rad/s overflows kp * e
  }
  // Scenario test values: a gyro sample and a reference (axis 0 at zero so kp = max does not overflow while healthy).
  const V3 y(0, 1, -1);
  const V3 healthy(0, 2, -2);
  V3 bad_ref = healthy;
  bool gyro_valid = true;
  switch (bad) {
    case Bad::None: break;
    case Bad::ReferenceNanRoll: bad_ref[0] = kNaN; break;
    case Bad::ReferenceNanPitch: bad_ref[1] = kNaN; break;
    case Bad::ReferenceNanYaw: bad_ref[2] = kNaN; break;
    case Bad::GyroInvalid: gyro_valid = false; break;
    case Bad::OutputNotFinite: bad_ref[0] = 2; break;  // e = 2 - 0, kp * e = inf
  }
  Rig rig(cfg);
  Rig fresh(cfg);
  bool ok = true;
  const auto zero_torque = [](const RateOutput<float>& o) { return bits_equal(o.torque, V3()); };

  (void)rig.loop.execute_bypass(rig.sample(y, 0), healthy);  // seed
  (void)rig.loop.execute_bypass(rig.sample(y, 0), healthy);
  const RateOutput<float> o1 = rig.loop.execute_bypass(rig.sample(y, 0, gyro_valid), bad_ref);
  ok = ok && zero_torque(o1) && o1.fault_active && o1.fault_latched && o1.fault_count == 1;
  ok = ok && rig.loop.fault_latched() && rig.loop.fault_count() == 1 && bits_equal(rig.loop.integrator(), V3());

  // The next execution is a seed: zero torque, no fault, the latch and the count kept.
  const RateOutput<float> o2 = rig.loop.execute_bypass(rig.sample(y, 0), healthy);
  ok = ok && zero_torque(o2) && !o2.fault_active && o2.fault_latched && o2.fault_count == 1;
  // ... and the loop now equals a fresh one seeded at the same sample.
  (void)fresh.loop.execute_bypass(fresh.sample(y, 0), healthy);
  const RateOutput<float> o3 = rig.loop.execute_bypass(rig.sample(y, 0), healthy);
  const RateOutput<float> of = fresh.loop.execute_bypass(fresh.sample(y, 0), healthy);
  ok = ok && bits_equal(o3.torque, of.torque) && bits_equal(rig.loop.integrator(), fresh.loop.integrator()) &&
       !o3.fault_active && o3.fault_latched && o3.fault_count == 1;
  // A second fault counts again.
  const RateOutput<float> o4 = rig.loop.execute_bypass(rig.sample(y, 0, gyro_valid), bad_ref);
  ok = ok && zero_torque(o4) && o4.fault_active && o4.fault_count == 2 && rig.loop.fault_count() == 2;
  return ok;
}

TEST(L5RateBypassFaults, EveryCheckBehavesAsInExecute) {
  for (const Bad bad : {Bad::ReferenceNanRoll, Bad::ReferenceNanPitch, Bad::ReferenceNanYaw, Bad::GyroInvalid,
                        Bad::OutputNotFinite}) {
    EXPECT_TRUE(fault_response_holds(bad)) << "case " << static_cast<int>(bad);
  }
}

TEST(L5RateBypassFaults, NegativeControlARunWithoutAFaultDoesNotSatisfyTheCheck) {
  EXPECT_FALSE(fault_response_holds(Bad::None));
}

TEST(L5RateBypassDeathTest, SpacingOutsideOneMicrosecondOfTheDesignPeriodPanics) {
  const RateConfig<float> cfg = test_config();
  const auto run = [&](std::int64_t jitter) {
    Rig rig(cfg);
    (void)rig.loop.execute_bypass(rig.sample(V3(), 0), V3());
    (void)rig.loop.execute_bypass(rig.sample(V3(), jitter), V3());
  };
  // Inside the bound: no panic (negative control for the death cases).
  run(-1);
  run(0);
  run(1);
  EXPECT_DEATH(run(2), "more than 1 us");
  EXPECT_DEATH(run(-2), "more than 1 us");
  EXPECT_DEATH(run(-static_cast<std::int64_t>(kPeriodUs)), "more than 1 us");  // a repeated stamp
  EXPECT_DEATH(run(static_cast<std::int64_t>(kPeriodUs)), "more than 1 us");   // a dropped sample
}

}  // namespace
