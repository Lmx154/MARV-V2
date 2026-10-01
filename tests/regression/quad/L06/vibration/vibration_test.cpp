// L6 stage (b), T1 (decision 0013): the plant's gyro vibration model.
//
// With the IMU noise off (N = B = 0, turn-on 0, latency 0) the gyro output is quantise(body rate + v), an exact function
// of the rotor speeds and angles, so the known-answer tests compare floats exactly (EXPECT_EQ). The rotor speeds and
// angles of the oracle come from a twin plant_model.hpp Model driven with the same calls as the plant (the angle itself
// is checked against the closed form in theta_test.cpp). The phases are recomputed from the noise stream (vib_support.hpp).
// Every metric check has a control that must break it.
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include "vib_support.hpp"

namespace {

using namespace marv::plant::vib_test;
using marv::plant::Model;
using marv::plant::Params;

constexpr double kStepDt = 0.012;  // s per marv_plant_step in these tests: 2.4 sub-steps of 5 ms, so a partial sub-step each call
constexpr std::uint64_t kSeed = 7;  // labelled: any seed; the tests also use kSeed + 1 as "another seed"
const std::array<std::uint16_t, 4> kDshots = {600, 900, 1200, 1500};  // four different mid-range speeds, all > 0
const std::array<double, 3> kAmp = {0.9, 0.5, 0.25};  // rad/s at hover: distinct per harmonic so a mix-up shows

Params<double> model_params(const marv_plant_config& c) {
  Params<double> p;
  p.tau = c.motor_tau_s;
  p.substep = c.motor_substep_s;
  p.omega_min = c.omega_min_rad_s;
  p.omega_max = c.omega_max_rad_s;
  for (std::size_t i = 0; i < 4; ++i) {
    p.omega0[i] = c.initial_omega_rad_s[i];
  }
  return p;
}

// A plant of the L1 fixture plus its twin Model. Every rotor starts at the speed of its DShot command in `cmd` (steady
// state: the speed stays bitwise constant, the angle advances), or at rest for command 0.
class Rig {
 public:
  explicit Rig(std::uint64_t seed, std::array<std::uint16_t, 4> cmd = kDshots) : seed_(seed), cmd_(cmd) {
    marv_plant_config c = fixture_config();
    c.rng_seed = seed;
    const Model<double> probe(model_params(c));
    for (std::size_t i = 0; i < 4; ++i) {
      c.initial_omega_rad_s[i] = probe.omega_cmd(cmd[i]);
    }
    cfg_ = c;
    twin_ = std::make_unique<Model<double>>(model_params(c));
    EXPECT_EQ(marv_plant_create(&c, &p_), MARV_PLANT_OK);
  }
  ~Rig() { marv_plant_destroy(p_); }
  Rig(const Rig&) = delete;
  Rig& operator=(const Rig&) = delete;

  marv_plant* get() const { return p_; }
  std::uint64_t seed() const { return seed_; }
  const Model<double>& twin() const { return *twin_; }

  void step(double dt = kStepDt) {
    const marv_plant_body b = identity_body();
    const marv_plant_cmd cmd = make_cmd(cmd_[0], cmd_[1], cmd_[2], cmd_[3]);
    marv_plant_out o = make_out();
    ASSERT_EQ(marv_plant_step(p_, &b, &cmd, dt, &o), MARV_PLANT_OK);
    twin_->advance(cmd_, dt);
    last_ = o;
  }
  const marv_plant_out& last_out() const { return last_; }

 private:
  std::uint64_t seed_;
  std::array<std::uint16_t, 4> cmd_;
  marv_plant_config cfg_{};
  marv_plant* p_ = nullptr;
  std::unique_ptr<Model<double>> twin_;
  marv_plant_out last_ = make_out();
};

void attach_both(Rig& rig, const marv_plant_imu_config& imu, const marv_plant_vibration_config& vib) {
  ASSERT_EQ(marv_plant_imu_attach(rig.get(), &imu), MARV_PLANT_OK);
  ASSERT_EQ(marv_plant_vibration_attach(rig.get(), &vib), MARV_PLANT_OK);
}

const marv_plant_body kBodyRate = [] {
  marv_plant_body b = identity_body();
  b.omega_frd_rad_s[0] = 0.5;    // binary-exact body rates, distinct per axis
  b.omega_frd_rad_s[1] = -0.25;
  b.omega_frd_rad_s[2] = 0.125;
  return b;
}();

std::array<double, 3> body_rate() {
  return {kBodyRate.omega_frd_rad_s[0], kBodyRate.omega_frd_rad_s[1], kBodyRate.omega_frd_rad_s[2]};
}

const marv_plant_imu_config kNoisyImu = noisy_imu();

// The oracle's gyro output for the rig's current state, amplitudes `amp`, exponent `p`.
std::array<float, 3> expected_gyro(const Rig& rig, std::array<double, 3> amp, int p, std::uint32_t* sat_mask = nullptr,
                                   std::uint64_t seed_override = ~std::uint64_t{0}) {
  const std::uint64_t seed = seed_override == ~std::uint64_t{0} ? rig.seed() : seed_override;
  const std::array<double, 3> v = expected_value(seed, rig.twin().omega(), rig.twin().theta(), amp, hover_omega(), p);
  const std::array<double, 3> w = body_rate();
  std::array<float, 3> out{};
  if (sat_mask != nullptr) {
    *sat_mask = 0;
  }
  for (std::size_t a = 0; a < 3; ++a) {
    bool sat = false;
    out[a] = quantise(w[a] + v[a], sat);
    if (sat && sat_mask != nullptr) {
      *sat_mask |= bit(MARV_PLANT_IMU_GYRO_SAT_X + static_cast<int>(a));
    }
  }
  return out;
}

marv_plant_imu_out sample(Rig& rig) { return sample_ok(rig.get(), kBodyRate); }

// ---------------------------------------------------------------------------------------------------------------------
// Known answers.

TEST(VibrationKat, FourRotorsThreeHarmonicsAfterSteps) {
  // Four rotors at different steady speeds, all three harmonics, p = 2, sampled after k = 0..5 steps (k = 0: the initial
  // state, theta = 0). The output equals quantise(body rate + v) on every axis, every k: the angle advances between
  // samples (so a stale or non-advancing angle fails) and the order of motors, harmonics and axes is pinned by the
  // distinct phases, amplitudes and speeds.
  Rig rig(kSeed);
  attach_both(rig, clean_imu(), vib_config(kAmp));
  std::array<float, 3> previous{};
  for (int k = 0; k <= 5; ++k) {  // labelled: six sampling points, the last after 5 steps = 60 ms
    if (k > 0) {
      rig.step();
    }
    const marv_plant_imu_out o = sample(rig);
    const std::array<float, 3> e = expected_gyro(rig, kAmp, 2);
    EXPECT_EQ(o.gyro_rad_s.x, e[0]) << "k " << k;
    EXPECT_EQ(o.gyro_rad_s.y, e[1]) << "k " << k;
    EXPECT_EQ(o.gyro_rad_s.z, e[2]) << "k " << k;
    EXPECT_EQ(o.flags, kValidBits);
    if (k > 0) {  // the output moved: the sample uses the state after the last step
      EXPECT_TRUE(o.gyro_rad_s.x != previous[0] || o.gyro_rad_s.y != previous[1] || o.gyro_rad_s.z != previous[2])
          << "k " << k;
    }
    previous = {o.gyro_rad_s.x, o.gyro_rad_s.y, o.gyro_rad_s.z};
    // Sampling again with no step in between reads the same state: the same bytes.
    EXPECT_TRUE(same_bits(sample(rig), o)) << "k " << k;
  }
}

TEST(VibrationKat, OracleControls) {
  // Each variant of the oracle that an implementation error would produce differs from the plant's output on some axis
  // after 5 steps: harmonic amplitudes reversed, motors' angles permuted, exponent 3, other seed's phases.
  Rig rig(kSeed);
  attach_both(rig, clean_imu(), vib_config(kAmp));
  for (int k = 0; k < 5; ++k) {
    rig.step();
  }
  const marv_plant_imu_out o = sample(rig);
  const std::array<float, 3> got = {o.gyro_rad_s.x, o.gyro_rad_s.y, o.gyro_rad_s.z};
  ASSERT_EQ(got, expected_gyro(rig, kAmp, 2));
  EXPECT_NE(got, expected_gyro(rig, {kAmp[2], kAmp[1], kAmp[0]}, 2));
  EXPECT_NE(got, expected_gyro(rig, kAmp, 3));
  EXPECT_NE(got, expected_gyro(rig, kAmp, 2, nullptr, kSeed + 1));
  // Motors permuted: the same sum with the rotors' (speed, angle) pairs rotated by one place.
  const std::array<double, 4> w = rig.twin().omega();
  const std::array<double, 4> t = rig.twin().theta();
  const std::array<double, 4> w2 = {w[1], w[2], w[3], w[0]};
  const std::array<double, 4> t2 = {t[1], t[2], t[3], t[0]};
  const std::array<double, 3> v = expected_value(kSeed, w2, t2, kAmp, hover_omega(), 2);
  std::array<float, 3> permuted{};
  for (std::size_t a = 0; a < 3; ++a) {
    bool sat = false;
    permuted[a] = quantise(body_rate()[a] + v[a], sat);
  }
  EXPECT_NE(got, permuted);
  // Phases are in the documented stream: the first of the 36 differs between stream 1 and stream 0 (ids are ABI).
  EXPECT_NE(noise::draw(kSeed, 1, 0), noise::draw(kSeed, 0, 0));
}

TEST(VibrationKat, SingleRotorSingleHarmonicClosedForm) {
  // One rotor (motor index 0, DShot 1200) spinning, every other rotor stopped: their ratio is 0, so their terms are
  // exactly 0. One harmonic at a time: the gyro equals quantise(body + A (w / w_hover)^2 sin(h theta + phi_{0,h,axis})),
  // written out directly. After 5 steps the angle is nonzero, so the harmonic multiplier h is visible.
  const std::array<std::uint16_t, 4> cmd = {1200, 0, 0, 0};
  for (std::size_t h = 0; h < 3; ++h) {
    Rig rig(kSeed, cmd);
    std::array<double, 3> amp{};
    amp[h] = 1.0;  // labelled: 1 rad/s at hover, large against the LSB (2^-10) so the sine's value is resolved
    attach_both(rig, clean_imu(), vib_config(amp));
    for (int k = 0; k < 5; ++k) {
      rig.step();
    }
    const double w = rig.twin().omega()[0];
    const double th = rig.twin().theta()[0];
    const double ratio = w / hover_omega();
    const marv_plant_imu_out o = sample(rig);
    const std::array<float, 3> got = {o.gyro_rad_s.x, o.gyro_rad_s.y, o.gyro_rad_s.z};
    for (std::size_t a = 0; a < 3; ++a) {
      const double y = body_rate()[a] + 1.0 * (ratio * ratio) * marv_musl_sin(static_cast<double>(h + 1) * th + phase(kSeed, 0, h, a));
      bool sat = false;
      EXPECT_EQ(got[a], quantise(y, sat)) << "h " << h + 1 << " axis " << a;
      // The same value from the platform's std::sin to within one count: the vendored sin is a sine.
      const double ref = body_rate()[a] + (ratio * ratio) * std::sin(static_cast<double>(h + 1) * th + phase(kSeed, 0, h, a));
      EXPECT_NEAR(static_cast<double>(got[a]), ref, kVibLsb) << "h " << h + 1 << " axis " << a;
      // Control: the wrong harmonic multiplier gives another value on at least one axis (checked after the loop).
    }
    std::array<float, 3> wrong{};
    for (std::size_t a = 0; a < 3; ++a) {
      bool sat = false;
      wrong[a] = quantise(body_rate()[a] + (ratio * ratio) * marv_musl_sin(static_cast<double>(h + 2) * th + phase(kSeed, 0, h, a)), sat);
    }
    EXPECT_NE(got, wrong) << "h " << h + 1;
  }
}

TEST(VibrationKat, FormulaAgainstIndependentLibmWithinDerivedBound) {
  // v of the formula against std::sin / std::pow / long double over all twelve terms, after 5 steps. Per term the model
  // differs from the reference by: the argument h theta + phi rounded twice in double (the product <= half an ulp of 3 * 2 pi
  // < 19, i.e. 1.8e-15, then the sum <= half an ulp of < 25.2 (3 * 2 pi + 2 pi), i.e. 1.8e-15; in all <= 3.6e-15, times |cos| <= 1),
  // the sine (musl <= 1 ulp ~ 1.1e-16, the platform's the same), the speed ratio squared (3 roundings, eps each at most) and
  // the two product roundings: <= 1e-14 A r per term, and the 12-term sum adds 12 eps/2 of its partial sums <= 1e-14 sum(A r)
  // again, so tol = 2e-14 * sum_h A_h * sum_i r_i.
  Rig rig(kSeed);
  attach_both(rig, clean_imu(), vib_config(kAmp));
  for (int k = 0; k < 5; ++k) {
    rig.step();
  }
  const std::array<double, 3> v = expected_value(kSeed, rig.twin().omega(), rig.twin().theta(), kAmp, hover_omega(), 2);
  const std::array<long double, 3> ref =
      reference_value(kSeed, rig.twin().omega(), rig.twin().theta(), kAmp, hover_omega(), 2.0);
  double sum_r = 0.0;
  for (double w : rig.twin().omega()) {
    sum_r += (w / hover_omega()) * (w / hover_omega());
  }
  const double tol = 2.0e-14 * (kAmp[0] + kAmp[1] + kAmp[2]) * sum_r;
  const std::array<long double, 3> wrong =
      reference_value(kSeed, rig.twin().omega(), rig.twin().theta(), kAmp, hover_omega(), 3.0);
  for (std::size_t a = 0; a < 3; ++a) {
    EXPECT_LT(std::fabs(static_cast<long double>(v[a]) - ref[a]), tol) << "axis " << a;
    EXPECT_GT(std::fabs(static_cast<long double>(v[a]) - wrong[a]), 1000.0L * tol) << "control, axis " << a;
  }
}

// ---------------------------------------------------------------------------------------------------------------------
// The step is not affected.

TEST(VibrationOff, StepOutputsBitIdenticalWithAndWithoutVibrationAndAcrossSeeds) {
  // Three plants stepped with the same commands: no vibration (seed 7), vibration with a large amplitude and IMU samples
  // between the steps (seed 7), vibration with another seed. Every marv_plant_out is bit-identical over 300 steps (labelled)
  // of varying commands.
  Rig plain(kSeed);
  Rig vibrating(kSeed);
  Rig other_seed(kSeed + 1);
  attach_both(vibrating, noisy_imu(), vib_config({5.0, 5.0, 5.0}));
  attach_both(other_seed, noisy_imu(), vib_config({5.0, 5.0, 5.0}));
  const marv_plant_body b = identity_body();
  for (int k = 0; k < 300; ++k) {
    const std::uint16_t d = static_cast<std::uint16_t>(48 + (k * 37) % 1900);  // varying, always within 48..1947
    const marv_plant_cmd cmd = make_cmd(d, static_cast<std::uint16_t>(d + 50), 0, static_cast<std::uint16_t>(2047 - (k % 100)));
    marv_plant_out a = make_out();
    marv_plant_out v = make_out();
    marv_plant_out s = make_out();
    ASSERT_EQ(marv_plant_step(plain.get(), &b, &cmd, kStepDt, &a), MARV_PLANT_OK);
    ASSERT_EQ(marv_plant_step(vibrating.get(), &b, &cmd, kStepDt, &v), MARV_PLANT_OK);
    ASSERT_EQ(marv_plant_step(other_seed.get(), &b, &cmd, kStepDt, &s), MARV_PLANT_OK);
    marv_plant_imu_out io{};
    ASSERT_EQ(marv_plant_imu_sample(vibrating.get(), &b, kStepDt, &io), MARV_PLANT_OK);
    ASSERT_EQ(marv_plant_imu_sample(other_seed.get(), &b, kStepDt, &io), MARV_PLANT_OK);
    ASSERT_TRUE(same_bits(a, v)) << "step " << k;
    ASSERT_TRUE(same_bits(a, s)) << "step " << k;
  }
  // Control: the comparison sees a difference when there is one (another command on one step).
  const marv_plant_cmd c1 = make_cmd(100, 100, 100, 100);
  const marv_plant_cmd c2 = make_cmd(101, 100, 100, 100);
  marv_plant_out x = make_out();
  marv_plant_out y = make_out();
  ASSERT_EQ(marv_plant_step(plain.get(), &b, &c1, kStepDt, &x), MARV_PLANT_OK);
  ASSERT_EQ(marv_plant_step(vibrating.get(), &b, &c2, kStepDt, &y), MARV_PLANT_OK);
  EXPECT_FALSE(same_bits(x, y));
}

// Runs the same step / sample sequence on two rigs; true iff every IMU output of the first equals the second's.
// `spin`: the rotors' commands (the rig's own).
bool same_imu_bytes(Rig& a, Rig& b, int n) {
  bool same = true;
  for (int k = 0; k < n; ++k) {
    a.step();
    b.step();
    const marv_plant_imu_out oa = sample(a);
    const marv_plant_imu_out ob = sample(b);
    same = same && same_bits(oa, ob);
  }
  return same;
}

constexpr int kStreamSamples = 400;  // labelled: 400 steps and samples, long enough for the bias and noise streams to act

TEST(VibrationOff, DetachedAndZeroAmplitudeGiveStageAImuBytes) {
  // Twin plants with the noisy IMU: one without the vibration entry (stage (a)), one with it.
  //  (i) all A = 0, rotors spinning, omega_hover = 1e-300 so that (omega / omega_hover)^2 overflows to inf: the 0 * inf must
  //      not reach the output (a zero amplitude contributes nothing);
  //  (ii) A != 0 but the rotors at rest: (0 / omega_hover)^2 = 0, so v = 0 exactly, and attaching the model (with its
  //      36 draws of stream 1) leaves stream 0 and the whole IMU unchanged.
  Rig plain(kSeed);
  ASSERT_EQ(marv_plant_imu_attach(plain.get(), &kNoisyImu), MARV_PLANT_OK);
  Rig zero(kSeed);
  marv_plant_vibration_config tiny_hover = vib_config({0.0, 0.0, 0.0});
  tiny_hover.omega_hover_rad_s = 1.0e-300;  // labelled: the smallest scale that overflows a ratio of 1e3 squared
  attach_both(zero, noisy_imu(), tiny_hover);
  EXPECT_TRUE(same_imu_bytes(plain, zero, kStreamSamples));

  const std::array<std::uint16_t, 4> rest = {0, 0, 0, 0};
  Rig plain_rest(kSeed, rest);
  ASSERT_EQ(marv_plant_imu_attach(plain_rest.get(), &kNoisyImu), MARV_PLANT_OK);
  Rig vib_rest(kSeed, rest);
  attach_both(vib_rest, noisy_imu(), vib_config({5.0, 5.0, 5.0}));
  EXPECT_TRUE(same_imu_bytes(plain_rest, vib_rest, kStreamSamples));

  // Controls: the same comparison with a nonzero amplitude and spinning rotors differs, and so does a single nonzero
  // harmonic (A_3 only) with the others zero.
  for (std::array<double, 3> amp : {std::array<double, 3>{1.0, 1.0, 1.0}, std::array<double, 3>{0.0, 0.0, 1.0}}) {
    Rig p2(kSeed);
    ASSERT_EQ(marv_plant_imu_attach(p2.get(), &kNoisyImu), MARV_PLANT_OK);
    Rig v2(kSeed);
    attach_both(v2, noisy_imu(), vib_config(amp));
    EXPECT_FALSE(same_imu_bytes(p2, v2, kStreamSamples));
  }
}

// ---------------------------------------------------------------------------------------------------------------------
// Determinism.

TEST(VibrationSeed, SameSeedSameBytesDifferentSeedDifferentPhases) {
  Rig a(kSeed);
  Rig b(kSeed);
  attach_both(a, noisy_imu(), vib_config(kAmp));
  attach_both(b, noisy_imu(), vib_config(kAmp));
  EXPECT_TRUE(same_imu_bytes(a, b, kStreamSamples));

  // Another seed: with the noise off the bytes are the vibration alone, so a differing byte is a differing phase.
  Rig c(kSeed);
  Rig d(kSeed + 1);
  attach_both(c, clean_imu(), vib_config(kAmp));
  attach_both(d, clean_imu(), vib_config(kAmp));
  c.step();
  d.step();
  const marv_plant_imu_out oc = sample(c);
  const marv_plant_imu_out od = sample(d);
  EXPECT_FALSE(same_bits(oc, od));
  // ... and each is its own seed's oracle (so the phases are those of the plant's seed, not some fixed table).
  EXPECT_EQ(oc.gyro_rad_s.x, expected_gyro(c, kAmp, 2)[0]);
  EXPECT_EQ(od.gyro_rad_s.x, expected_gyro(d, kAmp, 2)[0]);
  // All 36 phases of the two seeds differ and lie in [0, 2 pi).
  for (std::size_t i = 0; i < 4; ++i) {
    for (std::size_t h = 0; h < 3; ++h) {
      for (std::size_t ax = 0; ax < 3; ++ax) {
        EXPECT_NE(phase(kSeed, i, h, ax), phase(kSeed + 1, i, h, ax));
        EXPECT_GE(phase(kSeed, i, h, ax), 0.0);
        EXPECT_LT(phase(kSeed, i, h, ax), kTwoPi);
      }
    }
  }
}

// ---------------------------------------------------------------------------------------------------------------------
// Saturation.

TEST(VibrationSaturation, OverRangeAmplitudeSetsTheGyroSaturationFlags) {
  // Rotor 0 at DShot 1200 (ratio^2 ~ 0.85 at the fixture hover), A_1 = 1e4 rad/s, no steps (theta = 0): y_a = A r sin(phi_a),
  // far above the 100 rad/s full scale unless sin(phi_a) is below 100 / 8500 ~ 0.012. The flag of an axis is set iff the
  // oracle's count clamps; the clamped output is exactly +-100 (floor(100 / 2^-10) = 102400 counts). Accel flags stay
  // clear (the vibration is gyro only).
  const std::array<std::uint16_t, 4> cmd = {1200, 0, 0, 0};
  Rig rig(kSeed, cmd);
  const std::array<double, 3> amp = {1.0e4, 0.0, 0.0};
  attach_both(rig, clean_imu(), vib_config(amp));
  const marv_plant_imu_out o = sample(rig);
  std::uint32_t mask = 0;
  const std::array<float, 3> e = expected_gyro(rig, amp, 2, &mask);
  EXPECT_NE(mask, 0U);
  EXPECT_EQ(o.flags, kValidBits | mask);
  EXPECT_EQ(o.gyro_rad_s.x, e[0]);
  EXPECT_EQ(o.gyro_rad_s.y, e[1]);
  EXPECT_EQ(o.gyro_rad_s.z, e[2]);
  const std::array<float, 3> got = {o.gyro_rad_s.x, o.gyro_rad_s.y, o.gyro_rad_s.z};
  for (std::size_t a = 0; a < 3; ++a) {
    if ((mask >> a) & 1U) {
      EXPECT_EQ(std::fabs(got[a]), static_cast<float>(kFullScale)) << "axis " << a;
    }
  }
  // Control: a small amplitude sets no saturation flag and no clamp.
  Rig small(kSeed, cmd);
  attach_both(small, clean_imu(), vib_config({1.0, 0.0, 0.0}));
  EXPECT_EQ(sample(small).flags, kValidBits);
}

// ---------------------------------------------------------------------------------------------------------------------
// Controls and refusals.

// Counts of the x gyro axis for amplitude A_1 = `a`, single rotor 0 spinning, theta = 0 (no step), A_2 = A_3 = 0.
float x_gyro(double a, double exponent = 2.0) {
  const std::array<std::uint16_t, 4> cmd = {1200, 0, 0, 0};
  Rig rig(kSeed, cmd);
  marv_plant_vibration_config v = vib_config({a, 0.0, 0.0}, exponent);
  attach_both(rig, clean_imu(), v);
  return sample_ok(rig.get(), identity_body()).gyro_rad_s.x;
}

TEST(VibrationControls, OneUlpOfTheAmplitudeChangesTheBytes) {
  // The gyro is quantised to 2^-10, so a one-ulp change of A_1 changes a byte only where the value is at a rounding
  // tie. Construct that case: the x value is y(A) = (A r) sin(phi_x) with r = (w / w_hover)^2 (theta = 0, so the argument
  // is phi_x exactly). Start at the A that puts y at a tie (k + 1/2) LSB near 0.6 rad/s, then walk a few ulps of A until two
  // neighbours round to different counts (y is monotone in A). The plant must give different bytes for those two A's,
  // and identical bytes for the same A twice.
  const std::array<std::uint16_t, 4> cmd = {1200, 0, 0, 0};
  Rig probe(kSeed, cmd);
  const double ratio = probe.twin().omega()[0] / hover_omega();
  const double r = ratio * ratio;
  const double s = marv_musl_sin(phase(kSeed, 0, 0, 0));
  ASSERT_GT(std::fabs(s), 0.05);  // labelled: not near a zero of the sine, so A stays moderate
  const double tie = (std::floor(0.6 / kVibLsb) + 0.5) * kVibLsb * (s < 0.0 ? -1.0 : 1.0);
  auto count = [&](double a) { return std::round(((a * r) * s) / kVibLsb); };
  double a = tie / (r * s);
  for (int k = 0; k < 64; ++k) {  // labelled: far more ulps than the product's rounding can span
    a = std::nextafter(a, 0.0);
  }
  double lo = 0.0;
  bool found = false;
  for (int k = 0; k < 256 && !found; ++k) {
    const double up = std::nextafter(a, std::numeric_limits<double>::infinity());
    if (count(a) != count(up)) {
      lo = a;
      found = true;
    }
    a = up;
  }
  ASSERT_TRUE(found);
  const double hi = std::nextafter(lo, std::numeric_limits<double>::infinity());
  EXPECT_NE(x_gyro(lo), x_gyro(hi));
  EXPECT_EQ(x_gyro(lo), x_gyro(lo));
  EXPECT_EQ(static_cast<double>(x_gyro(lo)), (count(lo) * kVibLsb));
  EXPECT_EQ(static_cast<double>(x_gyro(hi)), (count(hi) * kVibLsb));
}

TEST(VibrationControls, SpeedExponentTwoAndThreeDiffer) {
  Rig a(kSeed);
  Rig b(kSeed);
  attach_both(a, clean_imu(), vib_config(kAmp, 2.0));
  attach_both(b, clean_imu(), vib_config(kAmp, 3.0));
  for (int k = 0; k < 5; ++k) {
    a.step();
    b.step();
  }
  const marv_plant_imu_out oa = sample(a);
  const marv_plant_imu_out ob = sample(b);
  EXPECT_FALSE(same_bits(oa, ob));
  // Each is its own exponent's oracle (the exponent reaches the sum), and exponent 0 is a constant amplitude.
  EXPECT_EQ(oa.gyro_rad_s.y, expected_gyro(a, kAmp, 2)[1]);
  EXPECT_EQ(ob.gyro_rad_s.y, expected_gyro(b, kAmp, 3)[1]);
  Rig z(kSeed);
  attach_both(z, clean_imu(), vib_config(kAmp, 0.0));
  z.step();
  EXPECT_EQ(sample(z).gyro_rad_s.z, expected_gyro(z, kAmp, 0)[2]);
}

TEST(VibrationRefusals, AttachRefusalsLeaveThePlantUnchanged) {
  const marv_plant_imu_config imu = noisy_imu();
  const marv_plant_vibration_config good = vib_config(kAmp);

  // Null and ABI.
  Rig r(kSeed);
  EXPECT_EQ(marv_plant_vibration_attach(nullptr, &good), MARV_PLANT_E_NULL);
  EXPECT_EQ(marv_plant_vibration_attach(r.get(), nullptr), MARV_PLANT_E_NULL);
  marv_plant_vibration_config bad = good;
  bad.struct_size = sizeof(good) - 1;
  EXPECT_EQ(marv_plant_vibration_attach(r.get(), &bad), MARV_PLANT_E_ABI);
  bad.struct_size = sizeof(good) + 1;
  EXPECT_EQ(marv_plant_vibration_attach(r.get(), &bad), MARV_PLANT_E_ABI);

  // Bad values: each of these is E_CONFIG.
  const double inf = std::numeric_limits<double>::infinity();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  std::vector<marv_plant_vibration_config> bads;
  auto with = [&](auto&& mutate) {
    marv_plant_vibration_config c = good;
    mutate(c);
    bads.push_back(c);
  };
  for (std::size_t h = 0; h < 3; ++h) {
    with([&](auto& c) { c.amplitude_rad_s[h] = -0.001; });
    with([&](auto& c) { c.amplitude_rad_s[h] = nan; });
    with([&](auto& c) { c.amplitude_rad_s[h] = inf; });
    with([&](auto& c) { c.amplitude_rad_s[h] = -inf; });
  }
  with([&](auto& c) { c.omega_hover_rad_s = 0.0; });
  with([&](auto& c) { c.omega_hover_rad_s = -1.0; });
  with([&](auto& c) { c.omega_hover_rad_s = nan; });
  with([&](auto& c) { c.omega_hover_rad_s = inf; });
  with([&](auto& c) { c.speed_exponent = 2.5; });
  with([&](auto& c) { c.speed_exponent = std::nextafter(2.0, 3.0); });
  with([&](auto& c) { c.speed_exponent = -1.0; });
  with([&](auto& c) { c.speed_exponent = nan; });
  with([&](auto& c) { c.speed_exponent = inf; });
  with([&](auto& c) { c.speed_exponent = static_cast<double>(MARV_PLANT_VIBRATION_MAX_EXPONENT) + 1.0; });
  for (const marv_plant_vibration_config& c : bads) {
    EXPECT_EQ(marv_plant_vibration_attach(r.get(), &c), MARV_PLANT_E_CONFIG);
  }
  // The bounds themselves are accepted on fresh plants: amplitude 0, exponent 0 and the maximum.
  for (double e : {0.0, static_cast<double>(MARV_PLANT_VIBRATION_MAX_EXPONENT)}) {
    Rig edge(kSeed);
    const marv_plant_vibration_config ok = vib_config({0.0, 0.0, 0.0}, e);
    EXPECT_EQ(marv_plant_vibration_attach(edge.get(), &ok), MARV_PLANT_OK);
  }

  // After all those refusals the plant is untouched: a valid attach succeeds and the IMU bytes equal a twin that only ever
  // attached the valid config. Then a second valid attach is E_STATE and changes nothing either.
  Rig twin(kSeed);
  ASSERT_EQ(marv_plant_imu_attach(r.get(), &imu), MARV_PLANT_OK);
  ASSERT_EQ(marv_plant_imu_attach(twin.get(), &imu), MARV_PLANT_OK);
  EXPECT_EQ(marv_plant_vibration_attach(r.get(), &good), MARV_PLANT_OK);
  EXPECT_EQ(marv_plant_vibration_attach(twin.get(), &good), MARV_PLANT_OK);
  marv_plant_vibration_config other = good;
  other.amplitude_rad_s[0] = 3.0;
  EXPECT_EQ(marv_plant_vibration_attach(r.get(), &other), MARV_PLANT_E_STATE);
  EXPECT_EQ(marv_plant_vibration_attach(r.get(), &good), MARV_PLANT_E_STATE);
  EXPECT_TRUE(same_imu_bytes(r, twin, 50));
  // Control: had the refused second config been applied, the bytes would differ.
  Rig applied(kSeed);
  ASSERT_EQ(marv_plant_imu_attach(applied.get(), &imu), MARV_PLANT_OK);
  ASSERT_EQ(marv_plant_vibration_attach(applied.get(), &other), MARV_PLANT_OK);
  Rig twin2(kSeed);
  ASSERT_EQ(marv_plant_imu_attach(twin2.get(), &imu), MARV_PLANT_OK);
  ASSERT_EQ(marv_plant_vibration_attach(twin2.get(), &good), MARV_PLANT_OK);
  EXPECT_FALSE(same_imu_bytes(applied, twin2, 50));
}

TEST(VibrationRefusals, AttachAfterTheFirstImuSampleIsRefusedAndOrderIsFree) {
  const marv_plant_vibration_config good = vib_config(kAmp);
  // After the first IMU sample: E_STATE, and the plant is as if the call never happened.
  Rig late(kSeed);
  Rig twin(kSeed);
  ASSERT_EQ(marv_plant_imu_attach(late.get(), &kNoisyImu), MARV_PLANT_OK);
  ASSERT_EQ(marv_plant_imu_attach(twin.get(), &kNoisyImu), MARV_PLANT_OK);
  EXPECT_TRUE(same_bits(sample(late), sample(twin)));
  EXPECT_EQ(marv_plant_vibration_attach(late.get(), &good), MARV_PLANT_E_STATE);
  EXPECT_TRUE(same_imu_bytes(late, twin, 50));
  // Before the IMU attach is allowed (the sample then needs the IMU attached, as before), and equals attaching after the IMU.
  Rig first(kSeed);
  Rig second(kSeed);
  ASSERT_EQ(marv_plant_vibration_attach(first.get(), &good), MARV_PLANT_OK);
  ASSERT_EQ(marv_plant_imu_attach(first.get(), &kNoisyImu), MARV_PLANT_OK);
  ASSERT_EQ(marv_plant_imu_attach(second.get(), &kNoisyImu), MARV_PLANT_OK);
  ASSERT_EQ(marv_plant_vibration_attach(second.get(), &good), MARV_PLANT_OK);
  EXPECT_TRUE(same_imu_bytes(first, second, 50));
  // Vibration alone does not make the IMU sample: it stays E_STATE until the IMU is attached.
  Rig only(kSeed);
  ASSERT_EQ(marv_plant_vibration_attach(only.get(), &good), MARV_PLANT_OK);
  marv_plant_imu_out o{};
  EXPECT_EQ(marv_plant_imu_sample(only.get(), &kBodyRate, kStepDt, &o), MARV_PLANT_E_STATE);
}

}  // namespace
