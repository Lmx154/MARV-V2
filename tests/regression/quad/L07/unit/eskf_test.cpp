// T1 for the attitude and gyro-bias ESKF in shadow (fw/estimator eskf.hpp, M4, decision 0055): known answers derived below, in
// double, against the float build. Conventions (core section 3) as mahony_test.cpp: FRD body, NED world, Hamilton quaternion
// [w, x, y, z] body -> NED, w >= 0; at rest the accelerometer reads the specific force, level (0, 0, -g).
//
// Numbers here are labelled scenario values of this test, not flight numbers:
//   - the tick, 130 us, as mahony_test.cpp (the whole-microsecond value of the board's 7680 Hz tick);
//   - kRegister: the values of fw/estimator/params/eskf_register.yaml (sources there; the gravity fusion window of 75 samples
//     included) and the 0.9 g / 1.1 g health window of the quad_fc register (Betaflight 4.5.1, decision 0042);
//   - the attitudes, rates, biases, durations and the noise of the long run are chosen to exercise every axis.
// g is kStandardGravity (constants.hpp). Tolerances are in float epsilon (kEps = 2^-23) or stated where used.
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <random>

#include "marv/estimator/eskf.hpp"
#include "marv/prim/constants.hpp"
#include "marv/prim/quat.hpp"
#include "marv/prim/vec.hpp"

namespace {

using marv::estimator::Eskf;
using marv::estimator::EskfAccel;
using marv::estimator::EskfConfig;
using marv::estimator::EskfCov;
using marv::estimator::kEskfAngle;
using marv::estimator::kEskfBias;
using marv::estimator::kEskfStates;
using marv::prim::Quat;
using marv::prim::Vec3;
using Filter = Eskf<float>;

constexpr std::uint32_t kTickUs = 130;
constexpr double kDt = static_cast<double>(kTickUs) / 1.0e6;
constexpr double kEps = std::numeric_limits<float>::epsilon();
constexpr double kG = marv::prim::kStandardGravity;
constexpr std::uint64_t kResetUs = 7000000;
constexpr std::uint32_t kDivisor = 75;  // eskf_accel_fusion_divisor
constexpr double kWindow = kDivisor * kDt;  // the span of a window of healthy samples, s

EskfConfig<float> register_config() {
  EskfConfig<float> c;
  c.gyro_noise_density = 4.8869219055841225e-05F;
  c.gyro_bias_random_walk = 5.985232540755365e-05F;
  c.accel_noise_density = 0.000588399F;
  c.angle_sigma_init = 0.1F;
  c.gyro_bias_sigma_init = 5.782283977442576e-05F;
  c.accel_gate = 5.0F;
  c.accel_reset_us = kResetUs;
  c.angle_var_min = 1.0e-9F;
  c.angle_var_max = 1.0F;
  c.gyro_bias_var_min = 1.0e-9F;
  c.gyro_bias_var_max = 1.0F;
  c.accel_healthy_min = 0.9F;
  c.accel_healthy_max = 1.1F;
  c.accel_fusion_divisor = kDivisor;
  return c;
}

// The register's noise with the bias states switched off (no random walk, zero initial bias variance) and no variance limits:
// B and C of P stay exactly 0, and the angle block evolves as stated in each test.
EskfConfig<float> no_bias_config() {
  EskfConfig<float> c = register_config();
  c.gyro_bias_random_walk = 0.0F;
  c.gyro_bias_sigma_init = 0.0F;
  c.angle_var_min = 0.0F;
  c.gyro_bias_var_min = 0.0F;
  return c;
}

Filter make(const EskfConfig<float>& c) {
  Filter f;
  f.init(c);
  return f;
}

int steps_for(double seconds) { return static_cast<int>(std::lround(seconds / kDt)); }

// Whole windows: the steps of the windows that fit in `seconds`, so a run after the initialising sample ends on a fusion.
int window_steps_for(double seconds) { return static_cast<int>(std::floor(seconds / kWindow)) * static_cast<int>(kDivisor); }

Vec3<float> v3(double x, double y, double z) {
  return Vec3<float>(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z));
}

Vec3<float> nan3() {
  const float n = std::numeric_limits<float>::quiet_NaN();
  return Vec3<float>(n, n, n);
}

// The specific force at rest for R = Ry(pitch) Rx(roll) (mahony_test.cpp): g (sin(pitch), -cos(pitch) sin(roll),
// -cos(pitch) cos(roll)).
Vec3<float> specific_force(double roll, double pitch, double scale = 1.0) {
  return v3(scale * kG * std::sin(pitch), -scale * kG * std::cos(pitch) * std::sin(roll),
            -scale * kG * std::cos(pitch) * std::cos(roll));
}

// q = qy(pitch) (x) qx(roll), yaw 0 (mahony_test.cpp).
Quat<double> tilt_quat(double roll, double pitch) {
  const double cr = std::cos(roll / 2.0);
  const double sr = std::sin(roll / 2.0);
  const double cp = std::cos(pitch / 2.0);
  const double sp = std::sin(pitch / 2.0);
  return Quat<double>(cr * cp, sr * cp, cr * sp, -sr * sp).canonical();
}

double max_diff(const Quat<float>& a, const Quat<double>& b) {
  const Quat<double> bc = b.canonical();
  return std::max({std::fabs(static_cast<double>(a.w) - bc.w), std::fabs(static_cast<double>(a.x) - bc.x),
                   std::fabs(static_cast<double>(a.y) - bc.y), std::fabs(static_cast<double>(a.z) - bc.z)});
}

Quat<double> to_double(const Quat<float>& q) {
  return Quat<double>(static_cast<double>(q.w), static_cast<double>(q.x), static_cast<double>(q.y), static_cast<double>(q.z));
}

// The angle between the estimate's and the truth's down directions in body axes (R^T e_z): the tilt error, heading excluded.
double tilt_error(const Quat<float>& est, const Quat<double>& truth) {
  const auto down = [](const Quat<double>& q) {
    return Vec3<double>(2.0 * (q.x * q.z - q.w * q.y), 2.0 * (q.y * q.z + q.w * q.x), 1.0 - 2.0 * (q.x * q.x + q.y * q.y));
  };
  const Vec3<double> a = down(to_double(est));
  const Vec3<double> b = down(truth);
  return std::atan2(a.cross(b).norm(), a.dot(b));
}

bool bitwise_equal(const Filter& a, const Filter& b) {
  const auto same = [](float x, float y) { return std::memcmp(&x, &y, sizeof(float)) == 0; };
  if (!(same(a.quat().w, b.quat().w) && same(a.quat().x, b.quat().x) && same(a.quat().y, b.quat().y) &&
        same(a.quat().z, b.quat().z))) {
    return false;
  }
  for (std::size_t i = 0; i < 3; ++i) {
    if (!same(a.bias()[i], b.bias()[i])) {
      return false;
    }
  }
  for (std::size_t i = 0; i < kEskfStates * kEskfStates; ++i) {
    if (!same(a.covariance().e[i], b.covariance().e[i])) {
      return false;
    }
  }
  return true;
}

// Cholesky in double of the float covariance: true iff every pivot is positive (P positive definite).
bool positive_definite(const EskfCov<float>& p) {
  std::array<std::array<double, kEskfStates>, kEskfStates> l{};
  for (std::size_t j = 0; j < kEskfStates; ++j) {
    double d = static_cast<double>(p(j, j));
    for (std::size_t k = 0; k < j; ++k) {
      d -= l[j][k] * l[j][k];
    }
    if (!(d > 0.0)) {
      return false;
    }
    l[j][j] = std::sqrt(d);
    for (std::size_t i = j + 1; i < kEskfStates; ++i) {
      double s = static_cast<double>(p(i, j));
      for (std::size_t k = 0; k < j; ++k) {
        s -= l[i][k] * l[j][k];
      }
      l[i][j] = s / l[j][j];
    }
  }
  return true;
}

bool symmetric(const EskfCov<float>& p) {
  for (std::size_t i = 0; i < kEskfStates; ++i) {
    for (std::size_t j = i + 1; j < kEskfStates; ++j) {
      if (std::memcmp(&p(i, j), &p(j, i), sizeof(float)) != 0) {
        return false;
      }
    }
  }
  return true;
}

void run(Filter& f, const Vec3<float>& gyro, const Vec3<float>& accel, int n) {
  for (int i = 0; i < n; ++i) {
    f.step(gyro, accel, kTickUs);
  }
}

// ---------------------------------------------------------------------------------------------------------------------
// a. At rest, level: converges.
// (1) The covariance, from a level start (every sample level and exact, so every residual and every injected error is exactly 0
// and the reset Jacobian G is the identity). With the bias states off (no_bias_config) B = C = 0 and P stays diagonal: each
// tilt axis is a scalar Kalman filter of a random walk (process variance q = N_g^2 dt / 2 per step, Q = n q over a window of
// n = kDivisor steps) observed once per window with variance var = (N_a / g)^2 / (2 n dt) (the window's mean); the heading is
// not observed (its gain is inhibited, and H does not read it). Closed forms: the steady-state prior P- at a fusion solves
// P-^2 - Q P- - Q var = 0, P- = (Q + sqrt(Q^2 + 4 Q var)) / 2, the posterior P+ = P- var / (P- + var); the heading variance is
// sigma0^2 + N q after N steps. Bounds: after 20 s of whole windows (P converges at about 2 N_g g / N_a = 1.6 1/s, so 32 time
// constants), ending on a fusion, P_xx and P_yy within 1e-3 relative of P+ (float rounding at a fixed point), the heading
// variance within N eps relative (one rounding per add), every off-diagonal entry exactly 0.
// (2) The attitude (kRegister), from a first sample rolled by 0.02 rad (an initialisation error) and level after it: the
// estimate reaches level, bound 1e-6 rad after 20 s (the first fusion, with P at its initial 0.1 rad, takes almost all of the
// error; the rest decays at the filter's bandwidth, about N_g g / N_a = 0.8 1/s).
TEST(Eskf, AtRestLevelConvergesToTheScalarSteadyState) {
  const EskfConfig<float> c = no_bias_config();
  Filter f = make(c);
  f.step(v3(0, 0, 0), specific_force(0.0, 0.0), kTickUs);
  ASSERT_TRUE(f.valid());
  const int n = window_steps_for(20.0);
  run(f, v3(0, 0, 0), specific_force(0.0, 0.0), n);
  ASSERT_TRUE(f.valid());
  ASSERT_EQ(f.accel_status(), EskfAccel::Fused);
  EXPECT_LE(tilt_error(f.quat(), Quat<double>::identity()), kEps);

  const double ng = static_cast<double>(c.gyro_noise_density);
  const double na = static_cast<double>(c.accel_noise_density) / kG;
  const double q = ng * ng * kDt / 2.0;
  const double big_q = kDivisor * q;
  const double var = na * na / (2.0 * kWindow);
  const double prior = (big_q + std::sqrt(big_q * big_q + 4.0 * big_q * var)) / 2.0;
  const double post = prior * var / (prior + var);
  const EskfCov<float>& p = f.covariance();
  EXPECT_NEAR(static_cast<double>(p(0, 0)), post, 1.0e-3 * post);
  EXPECT_NEAR(static_cast<double>(p(1, 1)), post, 1.0e-3 * post);
  const double heading = static_cast<double>(c.angle_sigma_init) * static_cast<double>(c.angle_sigma_init) + n * q;
  EXPECT_NEAR(static_cast<double>(p(2, 2)), heading, static_cast<double>(n) * kEps * heading);
  for (std::size_t i = 0; i < kEskfStates; ++i) {
    for (std::size_t j = 0; j < kEskfStates; ++j) {
      if (i != j) {
        EXPECT_EQ(p(i, j), 0.0F) << i << ", " << j;
      }
    }
  }

  Filter g = make(register_config());
  g.step(v3(0, 0, 0), specific_force(0.02, 0.0), kTickUs);
  ASSERT_TRUE(g.valid());
  EXPECT_NEAR(tilt_error(g.quat(), Quat<double>::identity()), 0.02, 16.0 * kEps);
  run(g, v3(0, 0, 0), specific_force(0.0, 0.0), n);
  ASSERT_TRUE(g.valid());
  EXPECT_LE(tilt_error(g.quat(), Quat<double>::identity()), 1.0e-6);
  EXPECT_EQ(g.counts().gated, 0U);
}

// ---------------------------------------------------------------------------------------------------------------------
// b. The gyro bias (kRegister): the part perpendicular to gravity is learned, the part along it is not. Nothing measures heading,
// so the bias along the body's down direction d = R^T e_z is unobservable (level, -R db dt maps it onto the heading alone); the
// filter removes that direction from every bias gain and zeroes the heading gain (eskf.hpp, inhibited gains). The defect this
// pins (the M4 T2 replay, eskf.hpp): without the inhibition the tilt updates moved the heading and, through the heading's
// correlation with b . d, that bias: level and still, b_z wandered to 5 deg/s in 47 s on the bench log.
// The bound on b . d: each step's bias change db_k is orthogonal to the estimate's down direction d^_k at its update, up to the
// rounding of the projection (a few eps of |db_k|), so its part along the true d is at most |db_k| (|d^_k - d| + 4 eps), and
// |d^_k - d| <= the tilt error e_k. Summed: |b . d| <= sum_k |db_k| (e_k + 4 eps), evaluated from the run itself.
struct BiasRun {
  double along = 0.0;  // b . d, d the truth's down direction in body axes
  double bound = 0.0;  // sum_k |db_k| (e_k + 4 eps)
};

BiasRun run_bias(Filter& f, const Quat<double>& truth, const Vec3<double>& bias, double gyro_sd, double accel_sd, int n,
                 std::mt19937* rng) {
  std::normal_distribution<double> unit(0.0, 1.0);
  const auto noise = [&](double sd) { return rng == nullptr ? 0.0 : sd * unit(*rng); };
  const Vec3<double> down(2.0 * (truth.x * truth.z - truth.w * truth.y), 2.0 * (truth.y * truth.z + truth.w * truth.x),
                          1.0 - 2.0 * (truth.x * truth.x + truth.y * truth.y));
  BiasRun r;
  for (int i = 0; i < n; ++i) {
    const Vec3<float> before = f.bias();
    f.step(v3(bias[0] + noise(gyro_sd), bias[1] + noise(gyro_sd), bias[2] + noise(gyro_sd)),
           v3(-kG * down[0] + noise(accel_sd), -kG * down[1] + noise(accel_sd), -kG * down[2] + noise(accel_sd)), kTickUs);
    const Vec3<double> db(static_cast<double>(f.bias()[0]) - static_cast<double>(before[0]),
                          static_cast<double>(f.bias()[1]) - static_cast<double>(before[1]),
                          static_cast<double>(f.bias()[2]) - static_cast<double>(before[2]));
    r.bound += db.norm() * (tilt_error(f.quat(), truth) + 4.0 * kEps);
  }
  r.along = static_cast<double>(f.bias()[0]) * down[0] + static_cast<double>(f.bias()[1]) * down[1] +
            static_cast<double>(f.bias()[2]) * down[2];
  return r;
}

// (1) Level and noise-free, the gyro reads the constant b = (3, 0, 3) sigma_b0 (sigma_b0 = gyro_bias_sigma_init, the boot
// calibration's residual): b_x converges and b_z, along gravity, stays at its initial 0. Known answer for b_x: level, the pair
// (dtheta_x, db_x) is a two-state linear system, predicted per step with F = [[1, -dt], [0, 1]] and the process noise
// (Theta_i, Omega_i), fused once per window by the row of the y residual with the window's variance; its error from
// (0, b_x) is propagated below in double with the same gains, and after 60 s it is below 1e-20 rad/s (bias time constant about
// 1 s). Bounds: b_x within 1e-4 of b_x relative (float rounding of the injected corrections at a fixed point), b_y within
// 1e-4 b_x of 0, |b_z| within the derived bound above, the tilt within 1e-6 rad.
TEST(Eskf, AConstantBiasPerpendicularToGravityConvergesAndTheBiasAlongGravityIsNotLearned) {
  const EskfConfig<float> c = register_config();
  const double s0 = static_cast<double>(c.gyro_bias_sigma_init);
  const Vec3<double> b(3.0 * s0, 0.0, 3.0 * s0);
  Filter f = make(c);
  f.step(v3(b[0], b[1], b[2]), specific_force(0.0, 0.0), kTickUs);
  ASSERT_TRUE(f.valid());
  const int n = window_steps_for(60.0);
  const BiasRun r = run_bias(f, Quat<double>::identity(), b, 0.0, 0.0, n, nullptr);
  ASSERT_TRUE(f.valid());

  const double ng = static_cast<double>(c.gyro_noise_density);
  const double kg = static_cast<double>(c.gyro_bias_random_walk);
  const double na = static_cast<double>(c.accel_noise_density) / kG;
  const double theta_i = ng * ng * kDt / 2.0;
  const double omega_i = kg * kg * kDt;
  const double var = na * na / (2.0 * kWindow);
  double paa = static_cast<double>(c.angle_sigma_init) * static_cast<double>(c.angle_sigma_init);
  double pab = 0.0;
  double pbb = s0 * s0;
  double ea = 0.0;
  double eb = b[0];
  for (int i = 1; i <= n; ++i) {
    ea -= eb * kDt;
    paa += -2.0 * pab * kDt + pbb * kDt * kDt + theta_i;
    pab -= pbb * kDt;
    pbb += omega_i;
    if (i % static_cast<int>(kDivisor) == 0) {
      const double ka = paa / (paa + var);
      const double kb = pab / (paa + var);
      const double r0 = ea;
      ea -= ka * r0;
      eb -= kb * r0;
      const double a = (1.0 - ka) * (1.0 - ka) * paa + ka * ka * var;
      const double ab = (1.0 - ka) * (pab - kb * paa) + ka * kb * var;
      const double bb = pbb - 2.0 * kb * pab + kb * kb * paa + kb * kb * var;
      paa = a;
      pab = ab;
      pbb = bb;
    }
  }
  EXPECT_LE(std::fabs(eb), 1.0e-20);
  EXPECT_NEAR(static_cast<double>(f.bias()[0]), b[0] - eb, 1.0e-4 * b[0]);
  EXPECT_LE(std::fabs(static_cast<double>(f.bias()[1])), 1.0e-4 * b[0]);
  EXPECT_LE(std::fabs(r.along), r.bound);
  EXPECT_LE(tilt_error(f.quat(), Quat<double>::identity()), 1.0e-6);
  EXPECT_EQ(f.counts().gated, 0U);
  EXPECT_EQ(f.counts().resets, 0U);
}

// (2) The defect itself: still, tilted as the bench board was (the 30 s still log's mean specific force (-0.1983, 0.0347,
// -9.7793) m/s^2, tests/regression/quad/L07/results/README.md: pitch asin(-0.1983 / 9.7814) = -0.0203 rad, roll
// atan2(-0.0347, 9.7793) = -0.0035 rad), the gyro and the accelerometer with white noise of the register's densities at this
// tick (per-sample sd N / sqrt(2 dt), the plant's rule; zero bias), 60 s, seeded std::mt19937 (any seed must pass). Bounds:
// |b . d| within the derived bound above, and |b_z| within sigma_b0, its initial standard deviation.
TEST(Eskf, StillAndNoisyTheBiasAlongGravityIsNotLearned) {
  const EskfConfig<float> c = register_config();
  const double pitch = std::asin(-0.1983 / 9.7814);
  const double roll = std::atan2(-0.0347, 9.7793);
  const Quat<double> truth = tilt_quat(roll, pitch);
  Filter f = make(c);
  f.step(v3(0, 0, 0), specific_force(roll, pitch), kTickUs);
  ASSERT_TRUE(f.valid());
  std::mt19937 rng(20261018U);
  const double gyro_sd = static_cast<double>(c.gyro_noise_density) / std::sqrt(2.0 * kDt);
  const double accel_sd = static_cast<double>(c.accel_noise_density) / std::sqrt(2.0 * kDt);
  const BiasRun r = run_bias(f, truth, Vec3<double>(), gyro_sd, accel_sd, window_steps_for(60.0), &rng);
  ASSERT_TRUE(f.valid());
  std::printf("b . d %.3e rad/s (bound %.3e), b (%.3e, %.3e, %.3e) rad/s, fused %u, gated %u\n", r.along, r.bound,
              static_cast<double>(f.bias()[0]), static_cast<double>(f.bias()[1]), static_cast<double>(f.bias()[2]),
              f.counts().fused, f.counts().gated);
  EXPECT_LE(std::fabs(r.along), r.bound);
  EXPECT_LE(std::fabs(static_cast<double>(f.bias()[2])), static_cast<double>(c.gyro_bias_sigma_init));
  EXPECT_EQ(f.counts().resets, 0U);
}

// ---------------------------------------------------------------------------------------------------------------------
// c. At rest, tilted (kRegister). The attitudes of mahony_test.cpp, an inverted roll among them. The first sample sets q to its
// tilt with yaw 0 (bound 16 eps, as Mahony's initialisation), and 1 s of the same sample holds it: the residual is rounding only,
// so the bound is the worst-case accumulation, n eps.
struct Tilt {
  double roll;
  double pitch;
};
constexpr Tilt kTilts[] = {{0.3, -0.2}, {-1.0, 0.5}, {3.0, 0.1}};

TEST(Eskf, AtRestTiltedInitialisesAndHoldsTheTilt) {
  for (const Tilt& t : kTilts) {
    SCOPED_TRACE(testing::Message() << "roll " << t.roll << " pitch " << t.pitch);
    const Vec3<float> a = specific_force(t.roll, t.pitch);
    Filter f = make(register_config());
    f.step(v3(0, 0, 0), a, kTickUs);
    ASSERT_TRUE(f.valid());
    EXPECT_LE(max_diff(f.quat(), tilt_quat(t.roll, t.pitch)), 16.0 * kEps);
    EXPECT_GE(f.quat().w, 0.0F);
    const int n = steps_for(1.0);
    run(f, v3(0, 0, 0), a, n);
    ASSERT_TRUE(f.valid());
    EXPECT_LE(tilt_error(f.quat(), tilt_quat(t.roll, t.pitch)), static_cast<double>(n) * kEps);
    EXPECT_GE(f.quat().w, 0.0F);
    EXPECT_LE(std::fabs(static_cast<double>(f.quat().norm()) - 1.0), 4.0 * kEps);
    EXPECT_EQ(f.counts().gated, 0U);
  }
}

// ---------------------------------------------------------------------------------------------------------------------
// d. Gyro-only dead reckoning between accelerometer updates (no_bias_config). After the initialising sample the accelerometer is
// absent (NaN, no update) for N = 1920 steps (0.2496 s) of a constant body rate omega = (12, -9, 15) rad/s, |omega| = 21.2 rad/s,
// a turn of 5.29 rad. Nominal: q(t) = q0 (x) exp(1/2 omega t) (mahony_test.cpp c; the bias estimate is 0), bound N eps.
// Covariance: with B = C = 0 the prediction is A <- A + Theta_i I each step, so A = (sigma0^2 + N N_g^2 dt / 2) I exactly
// and every other entry stays 0; bound N eps relative.
TEST(Eskf, GyroOnlyBetweenAccelUpdatesMatchesTheClosedForm) {
  const EskfConfig<float> c = no_bias_config();
  for (const Tilt& t : kTilts) {
    SCOPED_TRACE(testing::Message() << "roll " << t.roll << " pitch " << t.pitch);
    Filter f = make(c);
    f.step(v3(0, 0, 0), specific_force(t.roll, t.pitch), kTickUs);
    ASSERT_TRUE(f.valid());
    const Quat<double> q0 = to_double(f.quat());
    const Vec3<double> omega(12.0, -9.0, 15.0);
    const int n = 1920;
    run(f, v3(omega[0], omega[1], omega[2]), nan3(), n);
    ASSERT_TRUE(f.valid());
    const double tt = n * kDt;
    const Quat<double> expect = q0 * Quat<double>::from_axis_angle(omega * (1.0 / omega.norm()), omega.norm() * tt);
    EXPECT_LE(max_diff(f.quat(), expect), static_cast<double>(n) * kEps);
    EXPECT_EQ(f.accel_status(), EskfAccel::NotUsed);
    const double ng = static_cast<double>(c.gyro_noise_density);
    const double s0 = static_cast<double>(c.angle_sigma_init);
    const double a = s0 * s0 + n * ng * ng * kDt / 2.0;
    const EskfCov<float>& p = f.covariance();
    for (std::size_t i = 0; i < kEskfStates; ++i) {
      for (std::size_t j = 0; j < kEskfStates; ++j) {
        if (i == j && i < kEskfBias) {
          EXPECT_NEAR(static_cast<double>(p(i, j)), a, static_cast<double>(n) * kEps * a) << i;
        } else {
          EXPECT_EQ(p(i, j), 0.0F) << i << ", " << j;
        }
      }
    }
  }
}

// ---------------------------------------------------------------------------------------------------------------------
// e. The accelerometer gates (kRegister), after 5 s of whole windows level at rest. Each candidate is held for one whole window
// and compared with a twin stepped with no accelerometer (NaN) over it: a rejected window leaves the filter bitwise equal to its
// twin.
//   - health: |f| = 1.5 g (outside (0.9 g, 1.1 g)), tilted 0.3 rad: no sample enters the window, not used;
//   - innovation: |f| = g tilted 0.1 rad in roll. Its residual is about 0.1, against 5 sqrt(s) with s = var + H P H^T, var =
//     (N_a / g)^2 / (2 T) = 1.8e-7 for the window's T = 75 x 130 us (sqrt 4.3e-4) and H P H^T of order 1e-9 after convergence:
//     5 sqrt(s) = 2.1e-3 < 0.1, gated;
//   - the timeout: the same sample, held, is gated window after window until the spans of the rejected windows reach
//     accel_reset_us (7 s, kResetUs): the window where the sum of their spans (75 x 130 us = 9750 us each) reaches it,
//     ceil(7e6 / 9750) = 718 windows, resets the tilt to the window's direction (roll 0.1, yaw kept 0: bound 16 eps) and the next
//     window fuses.
TEST(Eskf, TheAccelGatesRejectAHighGAndAnInconsistentWindowAndTheTimeoutResetsTheTilt) {
  Filter f = make(register_config());
  f.step(v3(0, 0, 0), specific_force(0.0, 0.0), kTickUs);
  run(f, v3(0, 0, 0), specific_force(0.0, 0.0), window_steps_for(5.0));
  ASSERT_TRUE(f.valid());
  ASSERT_EQ(f.accel_status(), EskfAccel::Fused);
  const int window = static_cast<int>(kDivisor);

  Filter twin = f;
  run(f, v3(0, 0, 0), specific_force(0.3, 0.0, 1.5), window);
  run(twin, v3(0, 0, 0), nan3(), window);
  EXPECT_TRUE(bitwise_equal(f, twin));
  EXPECT_EQ(f.accel_status(), EskfAccel::NotUsed);
  EXPECT_EQ(f.counts().gated, 0U);

  const Vec3<float> off = specific_force(0.1, 0.0);
  run(f, v3(0, 0, 0), off, window);
  run(twin, v3(0, 0, 0), nan3(), window);
  EXPECT_TRUE(bitwise_equal(f, twin));
  EXPECT_EQ(f.accel_status(), EskfAccel::Gated);
  EXPECT_EQ(f.counts().gated, 1U);

  const std::uint64_t span_us = static_cast<std::uint64_t>(kDivisor) * kTickUs;
  const std::uint64_t needed = (kResetUs + span_us - 1) / span_us;
  std::uint64_t k = 1;
  while (f.accel_status() == EskfAccel::Gated && k < 2 * needed) {
    run(f, v3(0, 0, 0), off, window);
    ++k;
  }
  EXPECT_EQ(k, needed);
  EXPECT_EQ(f.accel_status(), EskfAccel::Reset);
  EXPECT_EQ(f.counts().resets, 1U);
  EXPECT_LE(max_diff(f.quat(), tilt_quat(0.1, 0.0)), 16.0 * kEps);
  const float a0 = register_config().angle_sigma_init * register_config().angle_sigma_init;
  for (std::size_t i = 0; i < 3; ++i) {
    EXPECT_EQ(f.covariance()(kEskfAngle + i, kEskfAngle + i), a0);
  }
  run(f, v3(0, 0, 0), off, window);
  EXPECT_EQ(f.accel_status(), EskfAccel::Fused);
}

// ---------------------------------------------------------------------------------------------------------------------
// f. The covariance stays symmetric positive definite over 10^6 float steps (kRegister): about 130 s of piecewise-constant body
// rates (a new rate every 10^4 steps, each axis uniform in +-5 rad/s), white gyro noise of 0.1 rad/s and accelerometer noise of
// 0.5 m/s^2 per axis (both far above the register's densities, so the gates and the resets are exercised), and every seventh
// segment a specific force of 2 g (unhealthy). The truth is integrated in double. Checked at every step: P bitwise symmetric,
// every diagonal within its floor and ceiling, valid(); every 1000 steps and at the end: P positive definite (Cholesky in
// double). Seeded std::mt19937 (the test's own draw; any seed must pass).
TEST(Eskf, TheCovarianceStaysSymmetricPositiveDefiniteOverAMillionFloatSteps) {
  const EskfConfig<float> c = register_config();
  Filter f = make(c);
  std::mt19937 rng(20261018U);
  std::uniform_real_distribution<double> rate(-5.0, 5.0);
  std::normal_distribution<double> gyro_noise(0.0, 0.1);
  std::normal_distribution<double> accel_noise(0.0, 0.5);
  constexpr int kSteps = 1000000;
  constexpr int kSegment = 10000;
  constexpr int kUnhealthyEvery = 7;
  constexpr int kCheckEvery = 1000;
  Quat<double> truth = tilt_quat(0.2, -0.1);
  Vec3<double> omega;
  bool unhealthy = false;
  int failures = 0;
  for (int i = 0; i < kSteps && failures < 10; ++i) {
    if (i % kSegment == 0) {
      omega = Vec3<double>(rate(rng), rate(rng), rate(rng));
      unhealthy = (i / kSegment) % kUnhealthyEvery == kUnhealthyEvery - 1;
    }
    if (i > 0) {
      truth = (truth * Quat<double>::from_axis_angle(omega * (1.0 / omega.norm()), omega.norm() * kDt)).normalized();
    }
    const Vec3<double> down(2.0 * (truth.x * truth.z - truth.w * truth.y), 2.0 * (truth.y * truth.z + truth.w * truth.x),
                            1.0 - 2.0 * (truth.x * truth.x + truth.y * truth.y));
    const double scale = unhealthy ? 2.0 : 1.0;
    const Vec3<float> accel = v3(-scale * kG * down[0] + accel_noise(rng), -scale * kG * down[1] + accel_noise(rng),
                                 -scale * kG * down[2] + accel_noise(rng));
    const Vec3<float> gyro = v3(omega[0] + gyro_noise(rng), omega[1] + gyro_noise(rng), omega[2] + gyro_noise(rng));
    f.step(gyro, accel, kTickUs);
    const EskfCov<float>& p = f.covariance();
    bool ok = f.valid() && symmetric(p);
    for (std::size_t s = 0; s < 3; ++s) {
      const float ang = p(kEskfAngle + s, kEskfAngle + s);
      const float bia = p(kEskfBias + s, kEskfBias + s);
      ok = ok && ang >= c.angle_var_min && ang <= c.angle_var_max && bia >= c.gyro_bias_var_min && bia <= c.gyro_bias_var_max;
    }
    if (i % kCheckEvery == 0 || i == kSteps - 1) {
      ok = ok && positive_definite(p);
    }
    if (!ok) {
      ++failures;
      ADD_FAILURE() << "step " << i;
    }
  }
  EXPECT_EQ(failures, 0);
  EXPECT_GT(f.counts().fused, 0U);
  EXPECT_GT(f.counts().gated, 0U);
  std::printf("1e6 steps: fused %u, gated %u, resets %u\n", f.counts().fused, f.counts().gated, f.counts().resets);
}

// ---------------------------------------------------------------------------------------------------------------------
// g. Non-finite input (kRegister), as Mahony's validity (decision 0042 item 5). Before initialisation a non-finite gyro or
// accelerometer changes nothing (no initialisation). After it: a NaN, infinite or zero accelerometer vector skips only the
// update (the filter equals its gyro-only twin bitwise and stays valid); a NaN or infinite gyro clears the state (identity, bias
// 0, P 0, not initialised, not valid), and the next healthy sample initialises it again.
TEST(Eskf, NonFiniteInputIsRefusedAndReportedInvalid) {
  const float inf = std::numeric_limits<float>::infinity();
  Filter f = make(register_config());
  f.step(nan3(), specific_force(0.0, 0.0), kTickUs);
  EXPECT_FALSE(f.initialised());
  f.step(v3(0, 0, 0), nan3(), kTickUs);
  EXPECT_FALSE(f.initialised());
  EXPECT_FALSE(f.valid());

  f.step(v3(0, 0, 0), specific_force(0.3, -0.2), kTickUs);
  run(f, v3(0.1, 0.2, -0.3), specific_force(0.3, -0.2), steps_for(0.5));
  ASSERT_TRUE(f.valid());
  for (const Vec3<float>& bad : {nan3(), v3(inf, 0, 0), v3(0, 0, 0)}) {
    Filter twin = f;
    f.step(v3(0.1, 0.2, -0.3), bad, kTickUs);
    twin.step(v3(0.1, 0.2, -0.3), nan3(), kTickUs);
    EXPECT_TRUE(bitwise_equal(f, twin));
    EXPECT_TRUE(f.valid());
  }
  for (const Vec3<float>& bad : {nan3(), v3(0, -inf, 0)}) {
    Filter g = f;
    g.step(bad, specific_force(0.3, -0.2), kTickUs);
    EXPECT_FALSE(g.initialised());
    EXPECT_FALSE(g.valid());
    EXPECT_EQ(g.quat().w, 1.0F);
    EXPECT_EQ(g.bias()[0], 0.0F);
    EXPECT_EQ(g.covariance()(0, 0), 0.0F);
    g.step(v3(0, 0, 0), specific_force(0.3, -0.2), kTickUs);
    EXPECT_TRUE(g.valid());
    EXPECT_LE(max_diff(g.quat(), tilt_quat(0.3, -0.2)), 16.0 * kEps);
  }
}

}  // namespace
