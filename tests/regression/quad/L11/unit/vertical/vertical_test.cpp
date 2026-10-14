// The L11 vertical law (decision 0020 sections 1 and 5), T1.
//   law          hand-computed values at three executions (the deferred integrator is visible); the sign
//   I4           the freeze iff the thrust is clamped beyond the rounding bound and e_v drives further into the limit;
//                the rounding case integrates; planted saturation on both sides; a clamp with e_v pointing out of the
//                limit does not freeze; the law never clamps
//   I5           valid bit clear, non-finite z, non-finite v_z, non-finite reference: exactly T_h, I zero, fault set
//   config       every refused case; reset
//
// Numbers in this file are "scenario test values": chosen to be exactly representable in float (powers of two and small
// integers) so each expected value below is exact arithmetic, worked in the comment beside it.
#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <limits>

#include <marv/mixer/mixer.hpp>
#include <marv/prim/vec.hpp>
#include <marv/types/vertical_state.hpp>
#include <marv/vertical/config.hpp>
#include <marv/vertical/vertical_law.hpp>

namespace {

using namespace marv;
using vertical::ConfigError;
using vertical::VerticalConfig;
using vertical::VerticalLaw;
using vertical::VerticalOutput;

constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
constexpr float kInf = std::numeric_limits<float>::infinity();

// k_z = 2 1/s, k_v = 4 N s/m, k_i = 8 N/m, T = 0.25 s, T_h = 10 N.
VerticalConfig<float> config() {
  VerticalConfig<float> c;
  c.kp_z = 2.0F;
  c.kp_v = 4.0F;
  c.ki_v = 8.0F;
  c.period = 0.25F;
  c.hover_thrust = 10.0F;
  return c;
}

VerticalState<float> state(float z, float vz, bool valid = true) { return VerticalState<float>{0, z, vz, valid}; }

// M-hat of a quad-X shape with every entry +-0.25 (scenario test value): each column of |M-hat| sums to 1, so the thrust
// row of |B-hat| |M-hat| is [1, 1, 1, 1] and the rounding bound is (gamma_n + eps) (|achieved_thrust| + sum |torque|).
mixer::MixerConfig<float> mixer_config() {
  constexpr std::array<std::array<float, mixer::kAxes>, mixer::kMotors> signs{{
      {0.25F, -0.25F, 0.25F, 0.25F},
      {0.25F, 0.25F, 0.25F, -0.25F},
      {0.25F, 0.25F, -0.25F, 0.25F},
      {0.25F, -0.25F, -0.25F, -0.25F},
  }};
  mixer::MixerConfig<float> c;
  for (std::size_t i = 0; i < mixer::kMotors; ++i) {
    for (std::size_t j = 0; j < mixer::kAxes; ++j) {
      c.m(i, j) = signs[i][j];
    }
  }
  return c;
}

// An allocation with the given achieved collective and achieved roll torque.
mixer::Allocation<float> allocation(float achieved, float roll_torque = 0.0F) {
  mixer::Allocation<float> al;
  al.achieved_thrust = achieved;
  al.achieved_torque = prim::Vec3<float>(roll_torque, 0.0F, 0.0F);
  return al;
}

VerticalLaw<float> make() {
  VerticalLaw<float> law;
  law.init(config(), mixer_config());
  return law;
}

TEST(L11Vertical, TheLawAtThreeExecutionsWithTheDeferredIntegrator) {
  VerticalLaw<float> law = make();
  // n = 1: z = 0, v_z = 0, z_ref = 1: e_z = 1, v_sp = 2, e_v = 2; I = 0 + 8 * 0 * 0.25 = 0; u = 4 * 2 + 0 = 8; T_req = 2.
  VerticalOutput<float> o = law.execute(state(0.0F, 0.0F), 1.0F);
  EXPECT_EQ(o.e_z, 1.0F);
  EXPECT_EQ(o.e_v, 2.0F);
  EXPECT_EQ(law.integrator(), 0.0F);
  EXPECT_EQ(o.thrust_request, 2.0F);
  EXPECT_FALSE(o.fault);
  // n = 2: z = 0.5, v_z = 1: e_z = 0.5, v_sp = 1, e_v = 0; I = 0 + 8 * e_v(1) * 0.25 = 4 (the integrator uses the previous
  // e_v = 2, not the present 0); u = 0 + 4 = 4; T_req = 6.
  o = law.execute(state(0.5F, 1.0F), 1.0F);
  EXPECT_EQ(o.e_z, 0.5F);
  EXPECT_EQ(o.e_v, 0.0F);
  EXPECT_EQ(law.integrator(), 4.0F);
  EXPECT_EQ(o.thrust_request, 6.0F);
  // n = 3: z = 0.5, v_z = 0.5: e_v = 1 - 0.5 = 0.5; I = 4 + 8 * 0 * 0.25 = 4; u = 4 * 0.5 + 4 = 6; T_req = 4.
  o = law.execute(state(0.5F, 0.5F), 1.0F);
  EXPECT_EQ(o.e_v, 0.5F);
  EXPECT_EQ(law.integrator(), 4.0F);
  EXPECT_EQ(o.thrust_request, 4.0F);
}

TEST(L11Vertical, BelowTheReferenceGivesMoreThrustAndAboveGivesLess) {
  VerticalLaw<float> law = make();
  // Below the reference in NED: z = 1 (down), z_ref = 0: e_z = -1, e_v = -2, u = -8, T_req = 10 + 8 = 18 > T_h.
  VerticalOutput<float> o = law.execute(state(1.0F, 0.0F), 0.0F);
  EXPECT_EQ(o.thrust_request, 18.0F);
  EXPECT_GT(o.thrust_request, config().hover_thrust);
  // Above: z = -1, z_ref = 0: e_v = 2, T_req = 2 < T_h.
  law = make();
  o = law.execute(state(-1.0F, 0.0F), 0.0F);
  EXPECT_EQ(o.thrust_request, 2.0F);
  EXPECT_LT(o.thrust_request, config().hover_thrust);
  // On the reference at rest: exactly T_h.
  law = make();
  o = law.execute(state(0.0F, 0.0F), 0.0F);
  EXPECT_EQ(o.thrust_request, config().hover_thrust);
}

// Runs execute at (z, vz, z_ref), records `al` for the request, then returns the integrator after a second execution
// at the same inputs (the second execution applies the increment k_i e_v(1) T unless frozen).
struct Frozen {
  bool frozen;
  float integrator_after;
};
Frozen clamp_case(float z, float vz, float achieved, float roll_torque = 0.0F) {
  VerticalLaw<float> law = make();
  const VerticalOutput<float> o = law.execute(state(z, vz), 0.0F);
  law.record_allocation(allocation(achieved, roll_torque), o.thrust_request);
  const bool frozen = law.frozen();
  (void)law.execute(state(z, vz), 0.0F);
  return Frozen{frozen, law.integrator()};
}

TEST(L11Vertical, FreezeOnALowSideClampWhenEvIsPositive) {
  // z = -1: e_v = 2, T_req = 2; the mixer raised it to 5: diff = -3 < 0 and e_v > 0 drives further down: freeze.
  const Frozen r = clamp_case(-1.0F, 0.0F, 5.0F);
  EXPECT_TRUE(r.frozen);
  EXPECT_EQ(r.integrator_after, 0.0F);
}

TEST(L11Vertical, FreezeOnAHighSideClampWhenEvIsNegative) {
  // z = 1: e_v = -2, T_req = 18; the mixer lowered it to 12: diff = 6 > 0 and e_v < 0 drives further up: freeze.
  const Frozen r = clamp_case(1.0F, 0.0F, 12.0F);
  EXPECT_TRUE(r.frozen);
  EXPECT_EQ(r.integrator_after, 0.0F);
}

TEST(L11Vertical, NoFreezeWhenEvPointsOutOfTheLimit) {
  // Low-side clamp (achieved 25 > T_req 18, diff = -7) with e_v = -2 < 0: asks for more thrust, out of the limit.
  // The increment 8 * (-2) * 0.25 = -4 is applied.
  Frozen r = clamp_case(1.0F, 0.0F, 25.0F);
  EXPECT_FALSE(r.frozen);
  EXPECT_EQ(r.integrator_after, -4.0F);
  // High-side clamp (achieved 1 < T_req 2, diff = 1) with e_v = +2: asks for less thrust, out of the limit. I = +4.
  r = clamp_case(-1.0F, 0.0F, 1.0F);
  EXPECT_FALSE(r.frozen);
  EXPECT_EQ(r.integrator_after, 4.0F);
}

TEST(L11Vertical, NoFreezeWhenNothingIsClamped) {
  // Achieved == requested (the mixer copies an unclamped request): integrates, I = 8 * (-2) * 0.25 = -4.
  const Frozen r = clamp_case(1.0F, 0.0F, 18.0F);
  EXPECT_FALSE(r.frozen);
  EXPECT_EQ(r.integrator_after, -4.0F);
}

TEST(L11Vertical, TheRoundingCaseIntegratesAndJustBeyondTheBoundFreezes) {
  // The bound b = (gamma_n + eps) (|achieved_thrust| + sum |achieved_torque|) here (column sums of |M-hat| are 1), n = 4,
  // recomputed in double: gamma_n = n eps / (1 - n eps).
  const double eps = std::numeric_limits<float>::epsilon();
  const double gamma = 4.0 * eps / (1.0 - 4.0 * eps);
  const double bound = (gamma + eps) * 18.0;
  // z = 1: e_v = -2, T_req = 18, a high-side clamp has achieved < 18. One ulp below 18 is far inside the bound.
  const float one_ulp_below = std::nextafter(18.0F, 0.0F);
  ASSERT_LT(static_cast<double>(18.0F - one_ulp_below), bound);
  Frozen r = clamp_case(1.0F, 0.0F, one_ulp_below);
  EXPECT_FALSE(r.frozen);
  EXPECT_EQ(r.integrator_after, -4.0F);
  // Beyond the bound: achieved = 18 - 2 b (rounded to float, still above the bound from 18): freezes.
  const float beyond = static_cast<float>(18.0 - 2.0 * bound);
  ASSERT_GT(static_cast<double>(18.0F - beyond), bound);
  r = clamp_case(1.0F, 0.0F, beyond);
  EXPECT_TRUE(r.frozen);
  EXPECT_EQ(r.integrator_after, 0.0F);
}

TEST(L11Vertical, TheTorquePartOfTheAllocationEntersTheBound) {
  // Achieved roll torque 1000 N m (scenario test value): b = (gamma_n + eps) (18 + 1000) with the achieved thrust 18.
  const double eps = std::numeric_limits<float>::epsilon();
  const double gamma = 4.0 * eps / (1.0 - 4.0 * eps);
  const double bound_thrust_only = (gamma + eps) * 18.0;
  const double bound = (gamma + eps) * (18.0 + 1000.0);
  // A shortfall of 1e-4 N (scenario test value) is beyond the thrust-only bound but inside the full bound: integrates.
  const float inside = static_cast<float>(18.0 - 1.0e-4);
  ASSERT_GT(static_cast<double>(18.0F - inside), bound_thrust_only);
  ASSERT_LT(static_cast<double>(18.0F - inside), bound);
  Frozen r = clamp_case(1.0F, 0.0F, inside, 1000.0F);
  EXPECT_FALSE(r.frozen);
  EXPECT_EQ(r.integrator_after, -4.0F);
  // Twice the full bound beyond: freezes.
  const float beyond = static_cast<float>(18.0 - 2.0 * bound);
  ASSERT_GT(static_cast<double>(18.0F - beyond), bound);
  r = clamp_case(1.0F, 0.0F, beyond, 1000.0F);
  EXPECT_TRUE(r.frozen);
  EXPECT_EQ(r.integrator_after, 0.0F);
}

TEST(L11Vertical, TheFreezeGatesOneIncrementOnly) {
  VerticalLaw<float> law = make();
  VerticalOutput<float> o = law.execute(state(1.0F, 0.0F), 0.0F);
  law.record_allocation(allocation(12.0F), o.thrust_request);
  ASSERT_TRUE(law.frozen());
  o = law.execute(state(1.0F, 0.0F), 0.0F);  // frozen: I stays 0
  EXPECT_EQ(law.integrator(), 0.0F);
  EXPECT_FALSE(law.frozen());  // no record_allocation for this execution
  (void)law.execute(state(1.0F, 0.0F), 0.0F);  // integrates: I = 8 * (-2) * 0.25 = -4
  EXPECT_EQ(law.integrator(), -4.0F);
}

TEST(L11Vertical, TheLawNeverClamps) {
  // z = 1e30 below: e_z = -1e30, e_v = k_z e_z = -2e30, u = k_v e_v = -8e30, T_req = T_h - u = 8e30 (float arithmetic).
  VerticalLaw<float> law = make();
  VerticalOutput<float> o = law.execute(state(1.0e30F, 0.0F), 0.0F);
  EXPECT_FALSE(o.fault);
  EXPECT_EQ(o.thrust_request, 10.0F - 4.0F * o.e_v);
  EXPECT_GT(o.thrust_request, 1.0e30F);
  law = make();
  o = law.execute(state(-1.0e30F, 0.0F), 0.0F);
  EXPECT_EQ(o.thrust_request, 10.0F - 4.0F * o.e_v);
  EXPECT_LT(o.thrust_request, -1.0e30F);  // negative thrust passes unchanged: the mixer alone owns the limits
}

class L11VerticalInvalid : public testing::TestWithParam<int> {};

VerticalState<float> invalid_state(int which) {
  switch (which) {
    case 0:
      return state(0.0F, 0.0F, false);  // valid bit clear
    case 1:
      return state(kNaN, 0.0F);
    case 2:
      return state(0.0F, kNaN);
    case 3:
      return state(kInf, 0.0F);
    default:
      return state(0.0F, -kInf);
  }
}

TEST_P(L11VerticalInvalid, OutputsExactlyTheHoverThrustResetsAndLatches) {
  VerticalLaw<float> law = make();
  (void)law.execute(state(0.0F, 0.0F), 1.0F);
  (void)law.execute(state(0.0F, 0.0F), 1.0F);  // I = 8 * 2 * 0.25 = 4
  ASSERT_EQ(law.integrator(), 4.0F);
  law.record_allocation(allocation(5.0F), 2.0F);
  const VerticalOutput<float> o = law.execute(invalid_state(GetParam()), 1.0F);
  EXPECT_EQ(o.thrust_request, config().hover_thrust);
  EXPECT_TRUE(o.fault);
  EXPECT_TRUE(law.fault());
  EXPECT_EQ(law.integrator(), 0.0F);
  EXPECT_FALSE(law.frozen());
  // The previous e_v is cleared too: a following valid execution adds nothing, and the latch stays set.
  const VerticalOutput<float> next = law.execute(state(0.0F, 0.0F), 1.0F);
  EXPECT_EQ(law.integrator(), 0.0F);
  EXPECT_TRUE(next.fault);
  EXPECT_TRUE(law.fault());
  law.reset();
  EXPECT_TRUE(law.fault());  // reset keeps the latch
  EXPECT_EQ(law.integrator(), 0.0F);
  law.init(config(), mixer_config());
  EXPECT_FALSE(law.fault());
  EXPECT_FALSE(law.execute(state(0.0F, 0.0F), 1.0F).fault);
}

INSTANTIATE_TEST_SUITE_P(Cases, L11VerticalInvalid, testing::Range(0, 5));

TEST(L11Vertical, ANonFiniteReferenceFaultsLikeAnInvalidState) {
  VerticalLaw<float> law = make();
  const VerticalOutput<float> o = law.execute(state(0.0F, 0.0F), kNaN);
  EXPECT_EQ(o.thrust_request, config().hover_thrust);
  EXPECT_TRUE(o.fault);
}

TEST(L11Vertical, ANonFiniteOutputFaultsWithTheHoverThrustAndAResetIntegrator) {
  // T_h = 3e38 N and z = 1.25e37 m (scenario test values, below FLT_MAX = 3.4e38): e_z = -1.25e37, e_v = -2.5e37,
  // u = -1e38 is finite, but T_req = 3e38 + 1e38 = 4e38 overflows float to +inf.
  VerticalConfig<float> c = config();
  c.hover_thrust = 3.0e38F;
  VerticalLaw<float> law;
  law.init(c, mixer_config());
  const VerticalOutput<float> o = law.execute(state(1.25e37F, 0.0F), 0.0F);
  EXPECT_EQ(o.thrust_request, 3.0e38F);
  EXPECT_TRUE(o.fault);
  EXPECT_TRUE(law.fault());
  EXPECT_EQ(law.integrator(), 0.0F);
  EXPECT_FALSE(law.frozen());
}

TEST(L11Vertical, ANonFiniteIntegratorFaultsWithTheHoverThrustAndAResetIntegrator) {
  // k_i = 3e38 N/m (scenario test value): z = -50, z_ref = 0: e_z = 50, e_v = 100, finite T_req = 10 - 400 = -390.
  // The second execution adds k_i e_v T = 3e38 * 100 * 0.25 = 7.5e39, which overflows float to +inf.
  VerticalConfig<float> c = config();
  c.ki_v = 3.0e38F;
  VerticalLaw<float> law;
  law.init(c, mixer_config());
  VerticalOutput<float> o = law.execute(state(-50.0F, 0.0F), 0.0F);
  ASSERT_FALSE(o.fault);
  ASSERT_EQ(o.thrust_request, -390.0F);
  o = law.execute(state(-50.0F, 0.0F), 0.0F);
  EXPECT_EQ(o.thrust_request, c.hover_thrust);
  EXPECT_TRUE(o.fault);
  EXPECT_TRUE(law.fault());
  EXPECT_EQ(law.integrator(), 0.0F);
  EXPECT_FALSE(law.frozen());
}

TEST(L11Vertical, ResetClearsTheIntegratorThePreviousErrorAndTheFreezeAndKeepsTheFault) {
  VerticalLaw<float> law = make();
  (void)law.execute(state(0.0F, 0.0F), 1.0F);
  (void)law.execute(state(0.0F, 0.0F), 1.0F);
  law.record_allocation(allocation(5.0F), 2.0F);
  ASSERT_EQ(law.integrator(), 4.0F);
  ASSERT_TRUE(law.frozen());
  (void)law.execute(state(kNaN, 0.0F), 1.0F);
  ASSERT_TRUE(law.fault());
  (void)law.execute(state(0.0F, 0.0F), 1.0F);  // e_v = 2 is now the previous error
  law.record_allocation(allocation(5.0F), 2.0F);
  law.reset();
  EXPECT_EQ(law.integrator(), 0.0F);
  EXPECT_FALSE(law.frozen());
  EXPECT_TRUE(law.fault());
  // The first execution after reset adds nothing (previous e_v = 0) although the last pre-reset e_v was 2.
  const VerticalOutput<float> o = law.execute(state(0.0F, 0.0F), 1.0F);
  EXPECT_EQ(law.integrator(), 0.0F);
  EXPECT_EQ(o.thrust_request, 2.0F);
}

TEST(L11Vertical, InitClearsTheStateAndTheLatch) {
  VerticalLaw<float> law = make();
  (void)law.execute(state(kNaN, 0.0F), 1.0F);
  ASSERT_TRUE(law.fault());
  law.init(config(), mixer_config());
  EXPECT_FALSE(law.fault());
  EXPECT_EQ(law.integrator(), 0.0F);
}

TEST(L11VerticalConfig, TheValidConfigurationAndItsBoundaries) {
  EXPECT_EQ(validate(config()), ConfigError::None);
  VerticalConfig<float> c = config();
  c.kp_z = 0.0F;  // an outer loop of zero gain is allowed
  EXPECT_EQ(validate(c), ConfigError::None);
  c = config();
  c.kp_v = 0.0F;
  c.ki_v = 0.0F;  // no velocity loop at all is allowed
  EXPECT_EQ(validate(c), ConfigError::None);
  c = config();
  c.ki_v = 0.0F;  // P only
  EXPECT_EQ(validate(c), ConfigError::None);
}

TEST(L11VerticalConfig, EveryRefusedCase) {
  const auto with = [](auto set) {
    VerticalConfig<float> c = config();
    set(c);
    return validate(c);
  };
  EXPECT_EQ(with([](auto& c) { c.kp_z = kNaN; }), ConfigError::NonFinite);
  EXPECT_EQ(with([](auto& c) { c.kp_v = kInf; }), ConfigError::NonFinite);
  EXPECT_EQ(with([](auto& c) { c.ki_v = kNaN; }), ConfigError::NonFinite);
  EXPECT_EQ(with([](auto& c) { c.period = kInf; }), ConfigError::NonFinite);
  EXPECT_EQ(with([](auto& c) { c.hover_thrust = kNaN; }), ConfigError::NonFinite);
  EXPECT_EQ(with([](auto& c) { c.kp_z = -1.0F; }), ConfigError::NegativeGain);
  EXPECT_EQ(with([](auto& c) { c.kp_v = -1.0F; }), ConfigError::NegativeGain);
  EXPECT_EQ(with([](auto& c) { c.ki_v = -1.0F; }), ConfigError::NegativeGain);
  EXPECT_EQ(with([](auto& c) { c.period = 0.0F; }), ConfigError::Period);
  EXPECT_EQ(with([](auto& c) { c.period = -0.25F; }), ConfigError::Period);
  EXPECT_EQ(with([](auto& c) { c.hover_thrust = 0.0F; }), ConfigError::HoverThrust);
  EXPECT_EQ(with([](auto& c) { c.hover_thrust = -10.0F; }), ConfigError::HoverThrust);
  EXPECT_EQ(with([](auto& c) { c.kp_v = 0.0F; }), ConfigError::IntegralWithoutProportional);
}

}  // namespace
