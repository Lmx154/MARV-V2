// L6 stage (b) T1, lines 3 and 4 (fault part): a rotor speed that is NaN, infinite, 0, negative, below the activation
// floor, flagged invalid, or whose harmonic is at or above f_s / 2 bypasses exactly the affected notches. The chain's
// output is bit-identical, tick for tick, to a test-local direct-form-I cascade of the remaining stages (the explicit
// reference: an absent notch), the bypass flag of that motor is set and its counter incremented once per update; the
// other motors' flags and counters are untouched. Controls: motors permuted, an unguarded notch above Nyquist.
// Also the configuration validator's refusals.
#include <limits>

#include "support.hpp"

namespace {

using namespace marv;
using namespace marv::gyro_chain;
using namespace marv::gyro_chain::test;

constexpr std::size_t kTicks = 4000;  // test value: ~ 20 notch time constants of the sharpest notch at 170 Hz (147 samples)

struct Expect {
  std::array<bool, kNotches> excluded{};
  std::uint32_t mask = 0;  // bit i: motor i + 1 has a bypassed notch
};

Expect expect_bypass(std::initializer_list<std::pair<std::size_t, std::size_t>> notches) {
  Expect e;
  for (const auto& [m, h] : notches) {
    e.excluded[notch_index(m, h)] = true;
    e.mask |= rotor_speed_valid_bit(m);
  }
  return e;
}

// A deterministic three-axis input: sums of tones at test frequencies (rad/tick) with test amplitudes, none zero.
Vec3f input(std::size_t n) {
  const double t = static_cast<double>(n);
  return Vec3f(static_cast<float>(1.0 + std::sin(0.3927 * t) + 0.5 * std::sin(0.0021 * t)),
               static_cast<float>(0.25 + std::sin(0.3043 * t + 0.5) + 0.5 * std::sin(0.7854 * t)),
               static_cast<float>(-1.0 + std::sin(1.1781 * t + 1.0) + 0.25 * std::sin(0.00137 * t)));
}

// True iff the chain given `sample` at every rate-loop execution is bit-identical to the reference without the excluded
// notches and its flags and counters are as expected.
bool matches(const Fixture& fx, const RotorSpeedSample& sample, const Expect& e,
             const std::array<float, kMotors>& reference_omega) {
  GyroChain<float> chain;
  chain.init(fx.config);
  RefChain ref(ref_stages(fx, reference_omega, [&] {
    std::array<bool, kNotches> include{};
    for (std::size_t i = 0; i < kNotches; ++i) {
      include[i] = !e.excluded[i];
    }
    return include;
  }()));
  bool same = true;
  std::uint32_t updates = 0;
  for (std::size_t n = 0; n < kTicks; ++n) {
    if (n % kDivisor == 0) {
      chain.update_notches(sample);
      ++updates;
    }
    same = same && bits_equal(chain.filter(input(n)), ref.filter(input(n)));
  }
  bool flags_ok = chain.bypass_flags() == e.mask;
  for (std::size_t m = 0; m < kMotors; ++m) {
    flags_ok = flags_ok && chain.bypass_count(m) == (((e.mask >> m) & 1U) != 0 ? updates : 0U);
  }
  return same && flags_ok;
}

TEST(L6GyroChainFaults, HealthySampleBypassesNothing) {
  const Fixture fx = make_fixture();
  const auto omega = rotor_omegas();
  EXPECT_TRUE(matches(fx, make_sample(omega), Expect{}, omega));
}

TEST(L6GyroChainFaults, EveryBadRotorSpeedBypassesItsNotchesBitIdentically) {
  const Fixture fx = make_fixture();
  const auto omega = rotor_omegas();
  const std::uint32_t bit0 = rotor_speed_valid_bit(0);
  const Expect m1 = expect_bypass({{0, 0}, {0, 1}, {0, 2}});
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float inf = std::numeric_limits<float>::infinity();

  struct Case {
    const char* name;
    float omega0;
    std::uint32_t flags;
    Expect e;
  };
  const std::vector<Case> cases{
      {"NaN", nan, kRotorSpeedFlagsDefined, m1},
      {"+inf", inf, kRotorSpeedFlagsDefined, m1},
      {"-inf", -inf, kRotorSpeedFlagsDefined, m1},
      {"zero", 0.0F, kRotorSpeedFlagsDefined, m1},
      {"negative", -omega[0], kRotorSpeedFlagsDefined, m1},
      {"below the activation floor", kOmegaThreshold / 2.0F, kRotorSpeedFlagsDefined, m1},
      {"invalid flag, speed left plausible", omega[0], kRotorSpeedFlagsDefined & ~bit0, m1},
      {"invalid flag, speed 0 as the convention says", 0.0F, kRotorSpeedFlagsDefined & ~bit0, m1},
      // 1700 Hz: the 1x harmonic is below f_s / 2 = 3200 Hz, 2x (3400) and 3x are not.
      {"2x and 3x at or above Nyquist", static_cast<float>(2.0 * kPi * 1700.0), kRotorSpeedFlagsDefined,
       expect_bypass({{0, 1}, {0, 2}})},
      // 1590 Hz: 1x and 2x (3180) are below, 3x (4770) is not.
      {"3x above Nyquist", static_cast<float>(2.0 * kPi * 1590.0), kRotorSpeedFlagsDefined, expect_bypass({{0, 2}})},
      // 3.23 kHz: even the 1x harmonic is above f_s / 2.
      {"1x above Nyquist", static_cast<float>(2.0 * kPi * 1.01 * fx.fs / 2.0), kRotorSpeedFlagsDefined, m1},
  };
  for (const Case& c : cases) {
    SCOPED_TRACE(c.name);
    auto w = omega;
    w[0] = c.omega0;
    // The reference's notch frequencies come from the same speeds, so the included notches match bit for bit.
    EXPECT_TRUE(matches(fx, make_sample(w, c.flags), c.e, w));
  }
}

TEST(L6GyroChainFaults, SeveralMotorsAndAllMotors) {
  const Fixture fx = make_fixture();
  const auto omega = rotor_omegas();
  auto w = omega;
  w[1] = std::numeric_limits<float>::quiet_NaN();
  w[3] = -omega[3];
  EXPECT_TRUE(matches(fx, make_sample(w), expect_bypass({{1, 0}, {1, 1}, {1, 2}, {3, 0}, {3, 1}, {3, 2}}), w));
  std::initializer_list<std::pair<std::size_t, std::size_t>> every = {
      {0, 0}, {0, 1}, {0, 2}, {1, 0}, {1, 1}, {1, 2}, {2, 0}, {2, 1}, {2, 2}, {3, 0}, {3, 1}, {3, 2}};
  EXPECT_TRUE(matches(fx, make_sample(omega, 0), expect_bypass(every), omega));
}

TEST(L6GyroChainFaults, FlagsFollowTheLatestUpdateAndCountersOnlyGrow) {
  const Fixture fx = make_fixture();
  const auto omega = rotor_omegas();
  GyroChain<float> chain;
  chain.init(fx.config);
  EXPECT_EQ(chain.bypass_flags(), 0U);
  auto bad = omega;
  bad[2] = std::numeric_limits<float>::quiet_NaN();
  chain.update_notches(make_sample(bad));
  chain.update_notches(make_sample(bad));
  EXPECT_EQ(chain.bypass_flags(), rotor_speed_valid_bit(2));
  EXPECT_EQ(chain.bypass_count(2), 2U);
  chain.update_notches(make_sample(omega));
  EXPECT_EQ(chain.bypass_flags(), 0U);
  EXPECT_EQ(chain.bypass_count(2), 2U);
  for (std::size_t m = 0; m < kMotors; ++m) {
    if (m != 2) {
      EXPECT_EQ(chain.bypass_count(m), 0U) << m;
    }
  }
  chain.update_notches(make_sample(bad));
  EXPECT_EQ(chain.bypass_count(2), 3U);
  // init clears flags and counters.
  chain.init(fx.config);
  EXPECT_EQ(chain.bypass_flags(), 0U);
  EXPECT_EQ(chain.bypass_count(2), 0U);
}

// Negative control: the same motor-1 fault, but the speeds of motors 1 and 2 swapped while the valid bits are not: the
// check against the fault's expectation must fail (flags land on the wrong motor, the wrong notches are bypassed).
TEST(L6GyroChainFaults, ControlMotorsPermutedFailsTheCheck) {
  const Fixture fx = make_fixture();
  const auto omega = rotor_omegas();
  auto w = omega;
  w[0] = 0.0F;  // motor 1 invalid, speed 0 by the convention
  const RotorSpeedSample good_order = make_sample(w, kRotorSpeedFlagsDefined & ~rotor_speed_valid_bit(0));
  const Expect e = expect_bypass({{0, 0}, {0, 1}, {0, 2}});
  ASSERT_TRUE(matches(fx, good_order, e, w));
  auto swapped = w;
  std::swap(swapped[0], swapped[1]);
  const RotorSpeedSample permuted = make_sample(swapped, good_order.flags);
  EXPECT_FALSE(matches(fx, permuted, e, w));
}

// Negative control: a notch computed for a frequency above Nyquist (what the guard prevents) is not the bypass. Its pole
// radius sqrt(a2) is above 1 (unstable), and a chain that included it differs from the guarded chain at once.
TEST(L6GyroChainFaults, ControlUnguardedNotchAboveNyquistFailsTheCheck) {
  const Fixture fx = make_fixture();
  const auto omega = rotor_omegas();
  auto w = omega;
  w[0] = static_cast<float>(2.0 * kPi * 3400.0);  // 3400 Hz > f_s / 2 = 3200 Hz
  const BiquadCoeffs<float> unguarded = notch_coeffs<float>(chain_f0(0, w[0]), fx.config.notch_q[0], fx.period);
  EXPECT_GT(unguarded.a2, 1.0F);
  // The expectation of an unguarded implementation: every motor-1 notch included in the reference.
  EXPECT_FALSE(matches(fx, make_sample(w), Expect{}, w));
  // The guarded expectation holds.
  EXPECT_TRUE(matches(fx, make_sample(w), expect_bypass({{0, 0}, {0, 1}, {0, 2}}), w));
}

TEST(L6GyroChainFaults, ValidatorRefusals) {
  const Fixture fx = make_fixture();
  const GyroChainConfig<float> ok = fx.config;
  EXPECT_EQ(validate(ok), ConfigError::None);
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float inf = std::numeric_limits<float>::infinity();
  const auto with = [&](auto mutate) {
    GyroChainConfig<float> c = ok;
    mutate(c);
    return validate(c);
  };
  EXPECT_EQ(with([&](auto& c) { c.period = nan; }), ConfigError::NonFinite);
  EXPECT_EQ(with([&](auto& c) { c.cutoff_hz = inf; }), ConfigError::NonFinite);
  EXPECT_EQ(with([&](auto& c) { c.notch_q[1] = nan; }), ConfigError::NonFinite);
  EXPECT_EQ(with([&](auto& c) { c.omega_threshold_rad_s = nan; }), ConfigError::NonFinite);
  EXPECT_EQ(with([&](auto& c) { c.period = 0.0F; }), ConfigError::Period);
  EXPECT_EQ(with([&](auto& c) { c.period = -c.period; }), ConfigError::Period);
  EXPECT_EQ(with([&](auto& c) { c.rate_divisor = 1; }), ConfigError::Divisor);
  EXPECT_EQ(with([&](auto& c) { c.rate_divisor = 0; }), ConfigError::Divisor);
  EXPECT_EQ(with([&](auto& c) { c.rate_divisor = 2; }), ConfigError::None);
  EXPECT_EQ(with([&](auto& c) { c.cutoff_hz = 0.0F; }), ConfigError::Cutoff);
  EXPECT_EQ(with([&](auto& c) { c.cutoff_hz = static_cast<float>(fx.fs / 2.0); }), ConfigError::Cutoff);
  EXPECT_EQ(with([&](auto& c) { c.cutoff_hz = static_cast<float>(fx.fs); }), ConfigError::Cutoff);
  EXPECT_EQ(with([&](auto& c) { c.cutoff_hz = static_cast<float>(0.99 * fx.fs / 2.0); }), ConfigError::None);
  EXPECT_EQ(with([&](auto& c) { c.notch_q[2] = 0.0F; }), ConfigError::NotchQ);
  EXPECT_EQ(with([&](auto& c) { c.notch_q[0] = -1.0F; }), ConfigError::NotchQ);
  EXPECT_EQ(with([&](auto& c) { c.omega_threshold_rad_s = 0.0F; }), ConfigError::Threshold);
  EXPECT_EQ(with([&](auto& c) { c.omega_threshold_rad_s = -1.0F; }), ConfigError::Threshold);
}

TEST(L6GyroChainFaults, RotorSpeedFlagBitsAreTheMotorIndexBits) {
  for (std::size_t i = 0; i < kMotors; ++i) {
    EXPECT_EQ(rotor_speed_valid_bit(i), rotor_speed_flag(static_cast<RotorSpeedFlag>(i))) << i;
  }
  EXPECT_EQ(kRotorSpeedFlagsDefined, (std::uint32_t{1} << kMotors) - 1U);
}

}  // namespace
