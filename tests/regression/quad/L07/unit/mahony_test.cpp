// T1 for the Mahony attitude filter (fw/estimator, M3 P1): known answers derived by hand below, in double, against the float
// build. Conventions (core section 3): FRD body, NED world, Hamilton quaternion [w, x, y, z] body -> NED, w >= 0. At rest the
// accelerometer reads the specific force, the reaction to gravity: level it is (0, 0, -g) in FRD.
//
// Numbers here are labelled scenario values of this test, not flight numbers:
//   - the tick, 130 us, is the whole-microsecond value of the board's 7680 Hz tick (1e6 / 7680 = 130.2 us,
//     fc3v2_scenario_values.yaml); the filter takes the step in whole microseconds;
//   - Kp = 2 1/s and Ki = 1 1/s^2: critical damping, Kp = 2 sqrt(Ki), the closed-loop poles are a double pole at -Kp / 2 = -1 1/s;
//   - the health bounds, 0.9 g and 1.1 g, are Betaflight 4.5.1's imuIsAccelerometerHealthy window (imu.c:326-338), the value of
//     the quad_fc register's estimator_accel_healthy_min and _max (decision 0042);
//   - the attitudes, rates and biases are chosen to exercise every axis and a rotation past half a turn.
// g is kStandardGravity (constants.hpp). Tolerances are in float epsilon (kEps = 2^-23), each stated where used.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

#include "marv/estimator/mahony.hpp"
#include "marv/prim/constants.hpp"
#include "marv/prim/quat.hpp"
#include "marv/prim/vec.hpp"

namespace {

using marv::estimator::Mahony;
using marv::estimator::MahonyConfig;
using marv::prim::Quat;
using marv::prim::Vec3;
using Filter = Mahony<float>;

constexpr std::uint32_t kTickUs = 130;
constexpr double kDt = static_cast<double>(kTickUs) / 1.0e6;
constexpr double kEps = std::numeric_limits<float>::epsilon();
constexpr double kG = marv::prim::kStandardGravity;
constexpr float kKp = 2.0F;
constexpr float kKi = 1.0F;
constexpr float kHealthyMin = 0.9F;  // g
constexpr float kHealthyMax = 1.1F;  // g

int steps_for(double seconds) { return static_cast<int>(std::lround(seconds / kDt)); }

Filter make(float kp, float ki) {
  Filter f;
  f.init(MahonyConfig<float>{kp, ki, kHealthyMin, kHealthyMax});
  return f;
}

Vec3<float> v3(double x, double y, double z) {
  return Vec3<float>(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z));
}

// The specific force at rest for R = Ry(pitch) Rx(roll): f_body = R^T (0, 0, -g) = -g (R20, R21, R22), with
// R20 = -sin(pitch), R21 = cos(pitch) sin(roll), R22 = cos(pitch) cos(roll), so
// f = g (sin(pitch), -cos(pitch) sin(roll), -cos(pitch) cos(roll)). Level: (0, 0, -g).
Vec3<float> specific_force(double roll, double pitch) {
  return v3(kG * std::sin(pitch), -kG * std::cos(pitch) * std::sin(roll), -kG * std::cos(pitch) * std::cos(roll));
}

// q = qy(pitch) (x) qx(roll), yaw 0: (cp + j sp)(cr + i sr) = cp cr + cp sr i + sp cr j + sp sr (j i), and j i = -k, with
// cp = cos(pitch / 2), sp = sin(pitch / 2), cr = cos(roll / 2), sr = sin(roll / 2).
Quat<double> tilt_quat(double roll, double pitch) {
  const double cr = std::cos(roll / 2.0);
  const double sr = std::sin(roll / 2.0);
  const double cp = std::cos(pitch / 2.0);
  const double sp = std::sin(pitch / 2.0);
  return Quat<double>(cr * cp, sr * cp, cr * sp, -sr * sp).canonical();
}

// q0 (x) exp(1/2 omega t): the body-rate integral of a constant omega, (cos(|omega| t / 2), omega / |omega| sin(|omega| t / 2)).
Quat<double> gyro_only(const Quat<double>& q0, const Vec3<double>& omega, double t) {
  const double n = omega.norm();
  if (n == 0.0) {
    return q0.canonical();
  }
  return (q0 * Quat<double>::from_axis_angle(omega * (1.0 / n), n * t)).canonical();
}

double max_diff(const Quat<float>& a, const Quat<double>& b) {
  const Quat<double> bc = b.canonical();
  return std::max({std::fabs(static_cast<double>(a.w) - bc.w), std::fabs(static_cast<double>(a.x) - bc.x),
                   std::fabs(static_cast<double>(a.y) - bc.y), std::fabs(static_cast<double>(a.z) - bc.z)});
}

void run(Filter& f, const Vec3<float>& gyro, const Vec3<float>& accel, int n) {
  for (int i = 0; i < n; ++i) {
    f.step(gyro, accel, kTickUs);
  }
}

// ---------------------------------------------------------------------------------------------------------------------
// a. At rest, level.
// v = f / |f| = (0, 0, -1) exactly along z and vhat = -(R20, R21, R22) = (0, 0, -1) for the identity, so e = v x vhat = 0 exactly
// (every term of the cross product has a zero factor), the corrected rate is 0, exp(0) is the identity and the normalisation
// divides by 1: q stays [1, 0, 0, 0] and the bias stays 0 in exact arithmetic. The bound used is the worst case of one rounding
// of epsilon per step, steps x epsilon: 10 s at the tick is 76923 steps, 9.2e-3.
TEST(Mahony, AtRestLevelStaysIdentityFor10Seconds) {
  Filter f = make(kKp, kKi);
  EXPECT_FALSE(f.initialised());
  const int n = steps_for(10.0);
  run(f, v3(0, 0, 0), specific_force(0, 0), n);
  ASSERT_TRUE(f.initialised());
  const double tol = static_cast<double>(n) * kEps;
  EXPECT_LE(max_diff(f.quat(), Quat<double>::identity()), tol);
  EXPECT_LE(std::fabs(static_cast<double>(f.quat().norm()) - 1.0), 4.0 * kEps);
  EXPECT_EQ(f.bias()[0], 0.0F);
  EXPECT_EQ(f.bias()[1], 0.0F);
  EXPECT_EQ(f.bias()[2], 0.0F);
}

// ---------------------------------------------------------------------------------------------------------------------
// b. At rest, tilted. The initial q is the tilt_quat above. Three attitudes: both axes with opposite signs, a steeper pair, and
// an inverted roll (3 rad, 172 deg, cos(roll) < 0). Initialisation: atan2, sqrt, sin, cos and two products, each within a
// couple of ulp: bound 16 epsilon (1.9e-6). Convention check independent of the quaternion formula: rotating the specific force
// by q must give (0, 0, -g) in NED, bound g x 16 epsilon. After N = 1 s (7692 steps) the filter holds the attitude: the
// correction is restoring (time constant 1/Kp), so the bound is the worst-case rounding accumulation, N x epsilon (9.2e-4).
struct Tilt {
  double roll;
  double pitch;
};
constexpr Tilt kTilts[] = {{0.3, -0.2}, {-1.0, 0.5}, {3.0, 0.1}};

TEST(Mahony, AtRestTiltedInitialisesAndHoldsTheTilt) {
  for (const Tilt& t : kTilts) {
    SCOPED_TRACE(testing::Message() << "roll " << t.roll << " pitch " << t.pitch);
    const Vec3<float> a = specific_force(t.roll, t.pitch);
    Filter f = make(kKp, kKi);
    f.step(v3(0, 0, 0), a, kTickUs);
    ASSERT_TRUE(f.initialised());
    EXPECT_LE(max_diff(f.quat(), tilt_quat(t.roll, t.pitch)), 16.0 * kEps);
    const Vec3<float> down = f.quat().rotate(a);
    EXPECT_LE(std::fabs(static_cast<double>(down[0])), kG * 16.0 * kEps);
    EXPECT_LE(std::fabs(static_cast<double>(down[1])), kG * 16.0 * kEps);
    EXPECT_LE(std::fabs(static_cast<double>(down[2]) + kG), kG * 16.0 * kEps);
    EXPECT_GE(f.quat().w, 0.0F);

    const int n = steps_for(1.0);
    run(f, v3(0, 0, 0), a, n);
    EXPECT_LE(max_diff(f.quat(), tilt_quat(t.roll, t.pitch)), static_cast<double>(n) * kEps);
    EXPECT_GE(f.quat().w, 0.0F);
    EXPECT_LE(std::fabs(static_cast<double>(f.quat().norm()) - 1.0), 4.0 * kEps);
  }
}

// ---------------------------------------------------------------------------------------------------------------------
// c. Constant rotation, no accelerometer correction (Kp = Ki = 0). With a constant body rate omega, q(t) = q0 (x) exp(1/2 omega t)
// with exp(1/2 omega t) = (cos(|omega| t / 2), omega / |omega| sin(|omega| t / 2)) (right product: omega is in the body
// frame). The first sample initialises q0 (no integration), then N = 1920 steps (t = 0.2496 s) integrate. |omega| = 20 rad/s
// (1146 deg/s, a fast flip), so the angle is 4.99 rad, past a half turn: w of the product changes sign for the level start,
// which exercises the canonical sign. Cases: each body axis from level, the combined axis (12, -9, 8) rad/s from level, and the
// combined axis from the tilt (0.3, -0.2). Error bound: each step does one Hamilton product, a sum of four products per
// component, at most 4 epsilon of relative rounding, plus the sin/cos/division of the increment (a few ulp, within the 4):
// 4 N epsilon = 9.1e-4 in the worst case, linear accumulation. The step length dt = 130 / 1e6 in float is off by at most
// epsilon / 2 relative, an angle error of 5 x 6e-8 = 3e-7, which is inside the bound.
struct Rotation {
  Vec3<double> omega;
  Tilt start;
};

TEST(Mahony, ConstantRotationWithoutCorrectionFollowsTheExponential) {
  const Rotation cases[] = {
      {Vec3<double>(20.0, 0.0, 0.0), {0.0, 0.0}},   {Vec3<double>(0.0, 20.0, 0.0), {0.0, 0.0}},
      {Vec3<double>(0.0, 0.0, 20.0), {0.0, 0.0}},   {Vec3<double>(12.0, -9.0, 8.0), {0.0, 0.0}},
      {Vec3<double>(12.0, -9.0, 8.0), {0.3, -0.2}},
  };
  const int n = steps_for(0.25);
  const double t = static_cast<double>(n) * kDt;
  const double tol = 4.0 * static_cast<double>(n) * kEps;
  for (const Rotation& c : cases) {
    SCOPED_TRACE(testing::Message() << "omega " << c.omega[0] << " " << c.omega[1] << " " << c.omega[2] << " start "
                                    << c.start.roll << " " << c.start.pitch);
    Filter f = make(0.0F, 0.0F);
    const Vec3<float> a = specific_force(c.start.roll, c.start.pitch);
    f.step(v3(0, 0, 0), a, kTickUs);
    run(f, v3(c.omega[0], c.omega[1], c.omega[2]), a, n);
    const Quat<double> expected = gyro_only(tilt_quat(c.start.roll, c.start.pitch), c.omega, t);
    EXPECT_LE(max_diff(f.quat(), expected), tol);
    EXPECT_GE(f.quat().w, 0.0F);
    EXPECT_LE(std::fabs(static_cast<double>(f.quat().norm()) - 1.0), 4.0 * kEps);
    EXPECT_EQ(f.bias()[0], 0.0F);
    EXPECT_EQ(f.bias()[1], 0.0F);
    EXPECT_EQ(f.bias()[2], 0.0F);
  }
}

// Hand check of the exponential for the first case, independent of any helper: omega = (20, 0, 0), t = 1920 x 130e-6 = 0.2496 s,
// angle 4.992 rad, half 2.496 rad: q = (cos 2.496, sin 2.496, 0, 0) = (-0.798743, 0.601672, 0, 0), canonical (0.798743, -0.601672, 0, 0)
// (the digits are rounded to 6 places: 5e-7; the float error here is far below the 1e-5 used).
TEST(Mahony, ConstantRotationAboutXKnownAnswer) {
  Filter f = make(0.0F, 0.0F);
  f.step(v3(0, 0, 0), specific_force(0, 0), kTickUs);
  run(f, v3(20.0, 0, 0), specific_force(0, 0), 1920);
  EXPECT_NEAR(static_cast<double>(f.quat().w), 0.798743, 1.0e-5);
  EXPECT_NEAR(static_cast<double>(f.quat().x), -0.601672, 1.0e-5);
  EXPECT_NEAR(static_cast<double>(f.quat().y), 0.0, 4.0 * 1920.0 * kEps);
  EXPECT_NEAR(static_cast<double>(f.quat().z), 0.0, 4.0 * 1920.0 * kEps);
}

// ---------------------------------------------------------------------------------------------------------------------
// d. Gyro bias at rest, level. The gyro reads omega + b = b (omega = 0). The error dynamics of the small tilt error phi and
// the bias error beta = bhat - b (only the part perpendicular to gravity is observable, so x and y here):
//   phi_dot = -beta - Kp phi,   beta_dot = Ki phi,   so   s^2 + Kp s + Ki = 0.
// For Kp = 2, Ki = 1 the double pole is -1 1/s (time constant tau = 2 / Kp = 1 s), phi(0) = 0 and beta(0) = -b give
//   phi(t) = b t e^{-t},   beta(t) = -b (1 + t) e^{-t}.
// The residual fraction (1 + t) e^{-t} is 4.0e-2 at 5 s, 5.0e-4 at 10 s and 4.3e-8 at 20 s: the convergence time to 1 permille
// is about 9.2 s. The check at 5 s is the law itself: |beta| equals the analytic value within 2 % of it (the discretisation of the
// bias integral is first order in Kp dt = 2.6e-4 per second, the linearisation of the cross product is second order in
// phi_max = b / e = 0.011 rad, 1e-4; 2 % is a margin of ten on both together) plus the float floor below. From 20 s on, the
// analytic residual is negligible and the error is the float floor: bhat accumulates Ki e dt per step into a value of
// size b, and an increment below half an ulp of bhat is lost, so the tilt error stalls at phi_floor = ulp(b) / (2 Ki dt) and
// the bias error at beta_floor = Kp phi_floor (ulp(0.03) = 1.86e-9, phi_floor = 7.2e-6 rad, beta_floor = 1.4e-5 rad/s). The
// tilt error is read from q as 2 x and 2 y (small angle: q ~ (1, phi / 2)).
TEST(Mahony, GyroBiasConvergesAtTheRateTheGainsGive) {
  const double bx = 0.02;   // rad/s, labelled scenario value (0.02 rad/s is 1.1 deg/s)
  const double by = -0.03;  // rad/s
  Filter f = make(kKp, kKi);
  const Vec3<float> a = specific_force(0, 0);
  const Vec3<float> gyro = v3(bx, by, 0);
  f.step(gyro, a, kTickUs);
  const double ulp = static_cast<double>(std::nextafter(0.03F, 1.0F)) - static_cast<double>(0.03F);
  const double phi_floor = ulp / (2.0 * static_cast<double>(kKi) * kDt);
  const double beta_floor = static_cast<double>(kKp) * phi_floor;

  const int n5 = steps_for(5.0);
  run(f, gyro, a, n5);
  const double t5 = static_cast<double>(n5) * kDt;
  const double residual5 = (1.0 + t5) * std::exp(-t5);
  EXPECT_NEAR(static_cast<double>(f.bias()[0]) - bx, -bx * residual5, 0.02 * std::fabs(bx) * residual5 + beta_floor);
  EXPECT_NEAR(static_cast<double>(f.bias()[1]) - by, -by * residual5, 0.02 * std::fabs(by) * residual5 + beta_floor);
  const double phi5 = t5 * std::exp(-t5);
  EXPECT_NEAR(2.0 * static_cast<double>(f.quat().x), bx * phi5, 0.02 * std::fabs(bx) * phi5 + phi_floor);

  run(f, gyro, a, steps_for(15.0));
  const double t20 = static_cast<double>(steps_for(20.0)) * kDt;
  const double residual20 = (1.0 + t20) * std::exp(-t20);
  EXPECT_LE(std::fabs(static_cast<double>(f.bias()[0]) - bx), residual20 * std::fabs(bx) + beta_floor);
  EXPECT_LE(std::fabs(static_cast<double>(f.bias()[1]) - by), residual20 * std::fabs(by) + beta_floor);
  EXPECT_LE(std::fabs(2.0 * static_cast<double>(f.quat().x)), phi_floor + residual20);
  EXPECT_LE(std::fabs(2.0 * static_cast<double>(f.quat().y)), phi_floor + residual20);
  EXPECT_EQ(f.bias()[2], 0.0F);
}

// ---------------------------------------------------------------------------------------------------------------------
// e. A zero or non-finite specific force skips the correction: the step follows the gyro alone, the bias is untouched. With
// large gains (Kp = 2, Ki = 1 and a 0.36 rad tilt error against the level vector the control uses) a correction would move q
// by far more than the tolerance. Reference as in c: q0 (x) exp(1/2 omega t), omega = (1, -2, 0.5) rad/s, N = 1920 steps,
// bound 4 N epsilon. The control feeds a finite level vector instead; q must then leave the gyro-only path by more than 1e-2
// (the bound is 9.1e-4, the effect is of the order of Kp x 0.36 rad x 0.25 s = 0.18 rad), so the test would catch a skip that
// does nothing.
Vec3<float> with_vec(float x, float y, float z) { return Vec3<float>(x, y, z); }

TEST(Mahony, ZeroOrNonFiniteAccelerationSkipsTheCorrection) {
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float inf = std::numeric_limits<float>::infinity();
  const float huge = 1.0e30F;  // |f|^2 overflows float: the norm is not finite
  const Vec3<float> bad[] = {with_vec(0, 0, 0),   with_vec(nan, nan, nan), with_vec(0, 0, nan),  with_vec(nan, 0, -9.8F),
                             with_vec(0, 0, inf), with_vec(-inf, 0, 0),     with_vec(huge, huge, huge)};
  const Tilt start{0.3, -0.2};
  const Vec3<double> omega(1.0, -2.0, 0.5);
  const int n = steps_for(0.25);
  const double t = static_cast<double>(n) * kDt;
  const double tol = 4.0 * static_cast<double>(n) * kEps;
  const Quat<double> expected = gyro_only(tilt_quat(start.roll, start.pitch), omega, t);
  const Vec3<float> gyro = v3(omega[0], omega[1], omega[2]);

  for (const Vec3<float>& a : bad) {
    SCOPED_TRACE(testing::Message() << "accel " << a[0] << " " << a[1] << " " << a[2]);
    Filter f = make(kKp, kKi);
    f.step(v3(0, 0, 0), specific_force(start.roll, start.pitch), kTickUs);
    run(f, gyro, a, n);
    EXPECT_LE(max_diff(f.quat(), expected), tol);
    EXPECT_EQ(f.bias()[0], 0.0F);
    EXPECT_EQ(f.bias()[1], 0.0F);
    EXPECT_EQ(f.bias()[2], 0.0F);
  }

  Filter control = make(kKp, kKi);
  control.step(v3(0, 0, 0), specific_force(start.roll, start.pitch), kTickUs);
  run(control, gyro, specific_force(0, 0), n);
  EXPECT_GT(max_diff(control.quat(), expected), 1.0e-2);
}

// Before the first healthy sample nothing is initialised and the gyro is not integrated; the first healthy sample then sets the tilt.
TEST(Mahony, InvalidFirstSamplesDoNotInitialise) {
  Filter f = make(kKp, kKi);
  const float nan = std::numeric_limits<float>::quiet_NaN();
  for (int i = 0; i < 10; ++i) {
    f.step(v3(1, 2, 3), with_vec(0, 0, 0), kTickUs);
    f.step(v3(1, 2, 3), with_vec(nan, 0, -9.8F), kTickUs);
  }
  EXPECT_FALSE(f.initialised());
  EXPECT_EQ(f.quat().w, 1.0F);
  EXPECT_EQ(f.quat().x, 0.0F);
  EXPECT_EQ(f.quat().y, 0.0F);
  EXPECT_EQ(f.quat().z, 0.0F);
  EXPECT_FALSE(f.valid());
  f.step(v3(1, 2, 3), specific_force(0.3, -0.2), kTickUs);
  EXPECT_TRUE(f.initialised());
  EXPECT_TRUE(f.valid());
  EXPECT_LE(max_diff(f.quat(), tilt_quat(0.3, -0.2)), 16.0 * kEps);
}

// ---------------------------------------------------------------------------------------------------------------------
// f. Validity (decision 0042 item 5). q is initialised, and valid() true, only from a sample whose specific-force norm is in the
// open interval (0.9 g, 1.1 g), at any tilt. The first healthy sample gives the exact tilt (the bound of b above, 16 epsilon)
// for the three tilts of b, an inverted one (roll pi, pitch 0: f = (0, 0, +g)) and a steep one near the singular pitch. For the
// inverted tilt w is 0 in exact arithmetic and the sign of the float w (6e-17 against the float rounding of pi, -4e-8) picks the
// canonical sign, q or -q, the same rotation: the comparison there is up to that sign; the down check below is not.
TEST(Mahony, FirstHealthySampleGivesTheTiltAndIsValidAtAnyTilt) {
  const Tilt tilts[] = {{0.0, 0.0}, {0.3, -0.2}, {-1.0, 0.5}, {3.0, 0.1}, {3.14159265358979, 0.0}, {0.2, 1.5}};
  for (const Tilt& t : tilts) {
    SCOPED_TRACE(testing::Message() << "roll " << t.roll << " pitch " << t.pitch);
    Filter f = make(kKp, kKi);
    EXPECT_FALSE(f.valid());
    f.step(v3(0, 0, 0), specific_force(t.roll, t.pitch), kTickUs);
    EXPECT_TRUE(f.initialised());
    EXPECT_TRUE(f.valid());
    const Quat<double> ref = tilt_quat(t.roll, t.pitch);
    const Quat<double> q(f.quat().w, f.quat().x, f.quat().y, f.quat().z);
    const double plus = std::max({std::fabs(q.w - ref.w), std::fabs(q.x - ref.x), std::fabs(q.y - ref.y), std::fabs(q.z - ref.z)});
    const double minus = std::max({std::fabs(q.w + ref.w), std::fabs(q.x + ref.x), std::fabs(q.y + ref.y), std::fabs(q.z + ref.z)});
    EXPECT_LE(std::min(plus, minus), 16.0 * kEps);
    const Vec3<float> a = specific_force(t.roll, t.pitch);
    const Vec3<float> down = f.quat().rotate(a);
    EXPECT_LE(std::fabs(static_cast<double>(down[0])), kG * 16.0 * kEps);
    EXPECT_LE(std::fabs(static_cast<double>(down[1])), kG * 16.0 * kEps);
    EXPECT_LE(std::fabs(static_cast<double>(down[2]) + kG), kG * 16.0 * kEps);
  }
}

// Free fall (|f| = 0), norms outside (0.9 g, 1.1 g) and the bounds themselves (open interval) leave the filter uninitialised and
// invalid, however many samples arrive, and the gyro is not integrated. Norms: 0.5 g, 0.8 g, 1.2 g, 3 g, 0.9 g x (1 - 1e-3) and
// 1.1 g x (1 + 1e-3) (inside float rounding of the bound is not tested; a thousandth is far above it). Control: 0.9 g x (1 + 1e-3)
// and 1.1 g x (1 - 1e-3) initialise, so the bounds are what the test exercises.
TEST(Mahony, UnhealthyNormsLeaveItInvalidAndUninitialised) {
  const double outside[] = {0.0, 0.5, 0.8, 1.2, 3.0, 0.9 * (1.0 - 1.0e-3), 1.1 * (1.0 + 1.0e-3)};
  for (const double norm : outside) {
    SCOPED_TRACE(testing::Message() << "|f| = " << norm << " g");
    Filter f = make(kKp, kKi);
    const Vec3<float> a = specific_force(0.3, -0.2) * static_cast<float>(norm);
    run(f, v3(1, 2, 3), a, 100);
    EXPECT_FALSE(f.initialised());
    EXPECT_FALSE(f.valid());
    EXPECT_EQ(f.quat().w, 1.0F);
    EXPECT_EQ(f.quat().x, 0.0F);
  }
  const double inside[] = {0.9 * (1.0 + 1.0e-3), 1.1 * (1.0 - 1.0e-3)};
  for (const double norm : inside) {
    SCOPED_TRACE(testing::Message() << "|f| = " << norm << " g");
    Filter f = make(kKp, kKi);
    f.step(v3(0, 0, 0), specific_force(0.3, -0.2) * static_cast<float>(norm), kTickUs);
    EXPECT_TRUE(f.valid());
  }
}

// A non-finite gyro (NaN or infinite, any axis) makes the estimate invalid and uninitialised and it stays so while the input
// lasts, even with a healthy accelerometer; the next healthy sample initialises it again from its own tilt. A NaN gyro is not
// taken for zero rotation: the attitude after recovery is the new sample's tilt, not the old estimate.
TEST(Mahony, NonFiniteGyroInvalidatesUntilTheNextHealthySample) {
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float inf = std::numeric_limits<float>::infinity();
  const Vec3<float> bad[] = {with_vec(nan, 0, 0), with_vec(0, nan, 0), with_vec(0, 0, nan), with_vec(inf, 0, 0),
                             with_vec(0, -inf, 0)};
  for (const Vec3<float>& g : bad) {
    SCOPED_TRACE(testing::Message() << "gyro " << g[0] << " " << g[1] << " " << g[2]);
    Filter f = make(kKp, kKi);
    f.step(v3(0, 0, 0), specific_force(0.3, -0.2), kTickUs);
    run(f, v3(0, 0, 0), specific_force(0.3, -0.2), 10);
    ASSERT_TRUE(f.valid());
    f.step(g, specific_force(0.3, -0.2), kTickUs);
    EXPECT_FALSE(f.valid());
    EXPECT_FALSE(f.initialised());
    run(f, g, specific_force(0.3, -0.2), 10);
    EXPECT_FALSE(f.valid());
    f.step(v3(0, 0, 0), specific_force(-0.5, 0.4), kTickUs);
    EXPECT_TRUE(f.valid());
    EXPECT_LE(max_diff(f.quat(), tilt_quat(-0.5, 0.4)), 16.0 * kEps);
  }
}

// A finite gyro so large that the rotation angle overflows (the largest float on every axis: the norm of the increment is
// infinite and the sine of an infinite half angle is NaN) makes the step's q non-finite: invalid and uninitialised, and valid
// again from the next healthy sample.
TEST(Mahony, NonFiniteQuaternionInvalidatesUntilTheNextHealthySample) {
  const float huge = std::numeric_limits<float>::max();
  Filter f = make(kKp, kKi);
  f.step(v3(0, 0, 0), specific_force(0.3, -0.2), kTickUs);
  ASSERT_TRUE(f.valid());
  f.step(v3(huge, huge, huge), specific_force(0.3, -0.2), kTickUs);
  EXPECT_FALSE(f.valid());
  EXPECT_FALSE(f.initialised());
  f.step(v3(0, 0, 0), specific_force(0.3, -0.2), kTickUs);
  EXPECT_TRUE(f.valid());
  EXPECT_LE(max_diff(f.quat(), tilt_quat(0.3, -0.2)), 16.0 * kEps);
}

}  // namespace
