// The L5 angle-mode setpoint generator (quad spec 4 L5, decision 0006 D), T1.
//   tilt          tilt(q_sp) <= theta_max over a stick sweep; on-axis full stick gives theta_max
//   deadband      strict comparison; the rescale gives full rate at full stick for every d in [0, 1)
//   yaw-rate mode C's split of the setpoint error has psi = 0; the yaw-rate command
//   braking       the lock at the first crossing, psi_lock = psi_m of that execution (control: a lock at the release
//                 heading or at the heading where the yaw input started)
//   guard (a)     one lock per release (control: the stick leaving the deadband gives a second, different lock)
//   guard (b)     the fallback at the first execution with dt >= t_cross AND dt alpha_min >= omega_r (controls: the
//                 execution before, each single condition alone, a sequence that crosses earlier)
//   re-arm, initialisation (including the singular set), faults
//
// Numbers in this file are one of: derived (the rule is stated), or a "scenario test value" with its reason.
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <random>
#include <vector>

#include <marv/attitude/angle_mode.hpp>
#include <marv/attitude/attitude_law.hpp>
#include <marv/attitude/config.hpp>
#include <marv/prim/constants.hpp>
#include <marv/prim/quat.hpp>
#include <marv/prim/vec.hpp>
#include <marv/types/attitude_state.hpp>

namespace {

using namespace marv;
using attitude::AngleMode;
using attitude::AngleOutput;
using attitude::AnglePhase;
using attitude::AngleSticks;
using attitude::AttitudeConfig;
using attitude::AttitudeLaw;
using Q = prim::Quat<float>;
using V3 = prim::Vec3<float>;

constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
constexpr float kInf = std::numeric_limits<float>::infinity();
constexpr float kMaxFloat = std::numeric_limits<float>::max();
constexpr double kEps = std::numeric_limits<float>::epsilon();
constexpr double kUnitRoundoff = kEps / 2;
constexpr double kPiD = prim::kPi;
constexpr double kMicro = prim::kMicrosecondsPerSecond;

double d(float x) { return static_cast<double>(x); }
float f(double x) { return static_cast<float>(x); }

struct Qd {
  double w = 1;
  double x = 0;
  double y = 0;
  double z = 0;
};
Qd to_d(const Q& q) { return Qd{d(q.w), d(q.x), d(q.y), d(q.z)}; }
Q to_f(const Qd& q) { return Q(f(q.w), f(q.x), f(q.y), f(q.z)); }
Qd mul(const Qd& a, const Qd& b) {
  return Qd{a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z, a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x, a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
}
Qd inv(const Qd& a) { return Qd{a.w, -a.x, -a.y, -a.z}; }
Qd normalized(const Qd& a) {
  const double n = std::sqrt(a.w * a.w + a.x * a.x + a.y * a.y + a.z * a.z);
  return Qd{a.w / n, a.x / n, a.y / n, a.z / n};
}
Qd canonical(const Qd& a) { return a.w < 0 ? Qd{-a.w, -a.x, -a.y, -a.z} : a; }
Qd about(double ax, double ay, double az, double angle) {
  const double s = std::sin(angle / 2);
  return Qd{std::cos(angle / 2), ax * s, ay * s, az * s};
}
// R^T e_z: the world-down axis in body axes.
std::array<double, 3> down_in_body(const Qd& q) {
  return {2 * (q.x * q.z - q.w * q.y), 2 * (q.y * q.z + q.w * q.x), q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z};
}

// The heading of decision 0006 D in double: psi = 2 atan2(z, w) with (w, z) signed into (-pi, pi]; 0 on the singular set.
double heading_ref(const Qd& q) {
  if (q.w == 0 && q.z == 0) {
    return 0;
  }
  const bool flip = q.w < 0 || (q.w == 0 && q.z < 0);
  return 2 * (flip ? std::atan2(-q.z, -q.w) : std::atan2(q.z, q.w));
}
double heading_of(const Q& q) { return heading_ref(to_d(q)); }

double wrap(double a) { return a - 2 * kPiD * std::round(a / (2 * kPiD)); }

// The tilt of a setpoint: the angle between its body z axis and world down.
double tilt_of(const Q& q) { return 2 * std::atan2(std::hypot(d(q.x), d(q.y)), std::hypot(d(q.w), d(q.z))); }

// The yaw angle psi of C's split of q^* (x) q_sp^: the yaw error the attitude law would see (unweighted).
double yaw_error_of(const Q& q, const Q& q_sp) {
  const Qd e = canonical(mul(inv(normalized(to_d(q))), normalized(to_d(q_sp))));
  if (e.w == 0 && e.z == 0) {
    return 0;
  }
  const double rho = std::hypot(e.w, e.z);
  return 2 * std::atan2(e.z / rho, e.w / rho);
}

// ---- the derived tolerances -------------------------------------------------------------------------------------

// The float error of the attitude law's closed form for k = 1 (decision 0006 C, rounding_bound of the attitude suite):
// 512 u (1 + 1 / rho) with u = epsilon / 2. The angle-mode setpoint is built with two products and one split of the
// same kind, so the error of an angle derived from q_sp is bounded by twice that, over rho (atan2).
constexpr double kRoundingUnits = 512;
double angle_tol(double rho) { return 2 * kRoundingUnits * kUnitRoundoff * (1 + 1 / rho) / rho; }
// Scenario test value: the largest current tilt (rad) of the random attitudes, so rho of q^* (x) q_des stays at or
// above cos((theta_max + this) / 2) = cos(0.92) = 0.6 (theta_max = pi / 3 below).
constexpr double kMaxCurrentTilt = 0.8;
constexpr double kTiltMaxD = kPiD / 3;

// ---- the scenario configuration ---------------------------------------------------------------------------------

// Scenario test value: the attitude period, 1 ms: a whole number of microseconds (the half-microsecond period is a
// separate rig).
constexpr std::uint64_t kPeriodUsNum = 1000;
constexpr std::uint64_t kPeriodUsDen = 1;
// Scenario test value: the first stamp, nonzero so a stamp of 0 is not mistaken for "no stamp".
constexpr std::uint64_t kT0Us = 5000;
// Scenario test values: rate limits distinct per axis, so an axis mix-up shows.
constexpr float kRateMaxRoll = 8;
constexpr float kRateMaxPitch = 6;
constexpr float kRateMaxYaw = 4;
// Scenario test values: a deadband of 10 % of the stick (the default is 0; a nonzero one exercises the rescale), and the
// fallback terms of the main braking tests: alpha_min = 10 rad/s^2 and t_cross = 0.2504 s, off the 1 ms stamp grid and
// the 312.5 us one (no ties).
constexpr float kDeadband = 0.1F;
constexpr float kAlphaMin = 10;
constexpr float kTCross = 0.2504F;
// The law entries the generator does not read; they must only pass validate().
constexpr float kGain = 3;
constexpr float kYawWeight = 0.5F;

AttitudeConfig<float> test_config() {
  AttitudeConfig<float> c;
  c.kp = kGain;
  c.yaw_weight = kYawWeight;
  c.period = static_cast<float>(kPeriodUsNum) / static_cast<float>(kPeriodUsDen) / static_cast<float>(kMicro);
  c.tilt_max = f(kTiltMaxD);
  c.yaw_deadband = kDeadband;
  c.yaw_alpha_min = kAlphaMin;
  c.yaw_t_cross = kTCross;
  c.rate_max = V3(kRateMaxRoll, kRateMaxPitch, kRateMaxYaw);
  return c;
}

// Scenario test value: the seed of every random stream.
constexpr std::uint64_t kSeed = 20260930;

class Rng {
 public:
  explicit Rng(std::uint64_t seed) : g_(seed) {}
  double uniform() {
    constexpr int kDigits = std::numeric_limits<double>::digits;
    return std::ldexp(static_cast<double>(g_() >> (64 - kDigits)), -kDigits);
  }
  double uniform(double lo, double hi) { return lo + (hi - lo) * uniform(); }

 private:
  std::mt19937_64 g_;
};

// ---- planted attitudes and the rig ------------------------------------------------------------------------------

struct Pose {
  double psi = 0;         // heading, rad
  double roll = 0;        // tilt about the heading frame's x, rad
  double pitch = 0;       // tilt about y, rad
  double psi_dot = 0;     // world-down rate, rad/s
};

// q = q_z(psi) (x) q_x(roll) (x) q_y(pitch); the body rates are the world-down rate in body axes, so that
// rotate(q, omega).z = psi_dot.
AttitudeState<float> make_state(std::uint64_t t_us, const Pose& p) {
  const Qd q = mul(mul(about(0, 0, 1, p.psi), about(1, 0, 0, p.roll)), about(0, 1, 0, p.pitch));
  const Q qf = to_f(q);
  const std::array<double, 3> zb = down_in_body(normalized(to_d(qf)));
  return AttitudeState<float>{t_us, qf, V3(f(zb[0] * p.psi_dot), f(zb[1] * p.psi_dot), f(zb[2] * p.psi_dot)), true};
}

struct Rig {
  AttitudeConfig<float> cfg;
  AngleMode<float> mode;
  std::uint64_t n = 0;
  std::uint64_t period_num;
  std::uint64_t period_den;

  explicit Rig(const AttitudeConfig<float>& c = test_config(), std::uint64_t num = kPeriodUsNum,
               std::uint64_t den = kPeriodUsDen)
      : cfg(c), period_num(num), period_den(den) {
    mode.init(cfg);
  }

  std::uint64_t stamp(std::uint64_t index) const { return kT0Us + index * period_num / period_den; }
  std::uint64_t last_stamp() const { return stamp(n - 1); }

  AngleOutput<float> step(float sr, float sp, float sy, const Pose& p) {
    return step_state(sr, sp, sy, make_state(stamp(n), p));
  }
  AngleOutput<float> step_state(float sr, float sp, float sy, const AttitudeState<float>& a) {
    ++n;
    return mode.execute(AngleSticks<float>{sr, sp, sy}, a);
  }
};

bool bits_equal(const Q& a, const Q& b) { return std::memcmp(&a, &b, sizeof(Q)) == 0; }

// ---- tilt -------------------------------------------------------------------------------------------------------

TEST(L5Angle, TheSetpointTiltNeverExceedsTheLimitAndOnAxisFullStickGivesIt) {
  const double tol = angle_tol(std::cos((kTiltMaxD + kMaxCurrentTilt) / 2));
  Rng rng(kSeed);
  // Scenario test value: a 21 x 21 grid of stick values in steps of 0.1 covers the disc, its boundary and the corners
  // of the square (where the norm is sqrt(2)).
  constexpr int kGrid = 10;
  for (int i = -kGrid; i <= kGrid; ++i) {
    for (int j = -kGrid; j <= kGrid; ++j) {
      const float sr = static_cast<float>(i) / static_cast<float>(kGrid);
      const float sp = static_cast<float>(j) / static_cast<float>(kGrid);
      const double n = std::hypot(d(sr), d(sp));
      const double want = kTiltMaxD * std::min(1.0, n);
      // Level and moving (yaw-rate mode), then centred (locked), then a tilted random attitude.
      const std::array<float, 2> yaw_sticks{0.0F, 1.0F};
      for (const float sy : yaw_sticks) {
        Rig rig;
        const Pose p{rng.uniform(-kPiD, kPiD), rng.uniform(-kMaxCurrentTilt, kMaxCurrentTilt) / 2,
                     rng.uniform(-kMaxCurrentTilt, kMaxCurrentTilt) / 2, 0};
        const AngleOutput<float> out = rig.step(sr, sp, sy, p);
        ASSERT_TRUE(out.valid);
        EXPECT_LE(tilt_of(out.q_sp), kTiltMaxD + tol) << i << "," << j << " sy " << d(sy);
        EXPECT_NEAR(tilt_of(out.q_sp), want, tol) << i << "," << j << " sy " << d(sy);
      }
    }
  }
  // On-axis full stick gives theta_max, in both senses; and the sign convention: positive roll stick rolls right side
  // down (positive rotation about x), positive pitch stick pitches nose up (positive rotation about y).
  const std::array<std::array<float, 2>, 4> axes{{{1, 0}, {-1, 0}, {0, 1}, {0, -1}}};
  for (const auto& a : axes) {
    Rig rig;
    const AngleOutput<float> out = rig.step(a[0], a[1], 0, Pose{});
    EXPECT_NEAR(tilt_of(out.q_sp), kTiltMaxD, tol);
    const auto sign = [](float v) { return (v > 0 ? 1 : 0) - (v < 0 ? 1 : 0); };
    EXPECT_EQ(sign(out.q_sp.x), sign(a[0]));
    EXPECT_EQ(sign(out.q_sp.y), sign(a[1]));
  }
  // Level sticks give exactly the heading quaternion of the lock: no tilt.
  Rig level;
  const AngleOutput<float> out = level.step(0, 0, 0, Pose{});
  EXPECT_EQ(out.q_sp.x, 0.0F);
  EXPECT_EQ(out.q_sp.y, 0.0F);
}

// ---- the deadband ------------------------------------------------------------------------------------------------

TEST(L5Angle, TheDeadbandComparisonIsStrictAndTheRescaleGivesFullRateAtFullStick) {
  // With d = 0: s = 0 locks, the smallest positive float stick is yaw-rate mode.
  {
    AttitudeConfig<float> c = test_config();
    c.yaw_deadband = 0.0F;
    Rig zero(c);
    const AngleOutput<float> o0 = zero.step(0, 0, 0.0F, Pose{});
    EXPECT_EQ(zero.mode.phase(), AnglePhase::Locked);
    EXPECT_EQ(o0.yaw_rate_cmd, 0.0F);
    Rig tiny(c);
    const float smallest = std::numeric_limits<float>::denorm_min();
    (void)tiny.step(0, 0, smallest, Pose{});
    EXPECT_EQ(tiny.mode.phase(), AnglePhase::YawRate);
    Rig tiny_neg(c);
    (void)tiny_neg.step(0, 0, -smallest, Pose{});
    EXPECT_EQ(tiny_neg.mode.phase(), AnglePhase::YawRate);
  }
  // A nonzero d: |s| = d is inside the band, the next float up is outside (Betaflight rc.c:700, strict).
  const std::array<float, 5> bands{0.0F, 0.05F, 0.1F, 0.3F, std::nextafter(1.0F, 0.0F)};
  for (const float band : bands) {
    AttitudeConfig<float> c = test_config();
    c.yaw_deadband = band;
    for (const float sign : {1.0F, -1.0F}) {
      Rig at(c);
      (void)at.step(0, 0, sign * band, Pose{});
      EXPECT_EQ(at.mode.phase(), AnglePhase::Locked) << "d " << d(band);
      Rig above(c);
      const AngleOutput<float> o = above.step(0, 0, sign * std::nextafter(band, 1.0F), Pose{});
      EXPECT_EQ(above.mode.phase(), AnglePhase::YawRate) << "d " << d(band);
      EXPECT_EQ(o.yaw_rate_cmd > 0, sign > 0);
      // Full stick is full rate, exactly, for every d in [0, 1): (1 - d) / (1 - d) = 1.
      Rig full(c);
      EXPECT_EQ(full.step(0, 0, sign, Pose{}).yaw_rate_cmd, sign * kRateMaxYaw) << "d " << d(band);
      // The rescale: sign (|s| - d) / (1 - d) rate_max_yaw, to the rounding of one subtraction, one quotient and one
      // product (3u relative) at a stick halfway between the band edge and full.
      const float mid = band + (1.0F - band) / 2;
      Rig half(c);
      const double want = d(sign) * (d(mid) - d(band)) / (1.0 - d(band)) * d(kRateMaxYaw);
      EXPECT_NEAR(d(half.step(0, 0, sign * mid, Pose{}).yaw_rate_cmd), want, 4 * kUnitRoundoff * d(kRateMaxYaw) + 1e-30)
          << "d " << d(band);
    }
  }
}

// ---- yaw-rate mode -----------------------------------------------------------------------------------------------

TEST(L5Angle, YawRateModeHasNoYawErrorAndCommandsTheRescaledRate) {
  const double tol = angle_tol(std::cos((kTiltMaxD + kMaxCurrentTilt) / 2));
  Rng rng(kSeed);
  constexpr int kSamples = 400;
  for (int n = 0; n < kSamples; ++n) {
    const Pose p{rng.uniform(-kPiD, kPiD), rng.uniform(-kMaxCurrentTilt, kMaxCurrentTilt) / 2,
                 rng.uniform(-kMaxCurrentTilt, kMaxCurrentTilt) / 2, rng.uniform(-3, 3)};
    const double rad = rng.uniform();
    const double az = rng.uniform(-kPiD, kPiD);
    const float sr = f(rad * std::cos(az));
    const float sp = f(rad * std::sin(az));
    const float sy = f(rng.uniform(d(kDeadband), 1.0) * (rng.uniform() < 0.5 ? -1 : 1));
    if (std::fabs(d(sy)) <= d(kDeadband)) {
      continue;
    }
    Rig rig;
    const AttitudeState<float> a = make_state(kT0Us, p);
    const AngleOutput<float> out = rig.step_state(sr, sp, sy, a);
    ASSERT_TRUE(out.valid);
    EXPECT_EQ(rig.mode.phase(), AnglePhase::YawRate);
    // ω_ff = (0, 0, psi_dot_cmd): the command is the rescaled stick times rate_max_yaw.
    const double want = std::copysign(1.0, d(sy)) * (std::fabs(d(sy)) - d(kDeadband)) / (1.0 - d(kDeadband)) * d(kRateMaxYaw);
    EXPECT_NEAR(d(out.yaw_rate_cmd), want, 4 * kUnitRoundoff * d(kRateMaxYaw));
    // The yaw error of C's split vanishes within the derived bound, and the heading setpoint is the current heading.
    EXPECT_NEAR(yaw_error_of(a.q, out.q_sp), 0.0, tol) << n;
    // q_sp is q tilted by the shortest rotation to the desired thrust axis: its thrust axis is the heading-frame
    // tilt's, q_z(psi_m) (x) q_xy(t) (the world-first heading of q_sp itself is not constrained).
    const double n_stick = std::hypot(d(sr), d(sp));
    if (n_stick > 0) {
      const double half = kTiltMaxD * std::min(1.0, n_stick);
      const Qd q_des = mul(about(0, 0, 1, heading_of(a.q)), about(d(sr) / n_stick, d(sp) / n_stick, 0, half));
      const Qd qs = normalized(to_d(out.q_sp));
      const std::array<double, 3> axis_sp{2 * (qs.x * qs.z + qs.w * qs.y), 2 * (qs.y * qs.z - qs.w * qs.x),
                                          qs.w * qs.w - qs.x * qs.x - qs.y * qs.y + qs.z * qs.z};
      const std::array<double, 3> axis_des{2 * (q_des.x * q_des.z + q_des.w * q_des.y),
                                           2 * (q_des.y * q_des.z - q_des.w * q_des.x),
                                           q_des.w * q_des.w - q_des.x * q_des.x - q_des.y * q_des.y +
                                               q_des.z * q_des.z};
      const double cross = std::hypot(axis_sp[1] * axis_des[2] - axis_sp[2] * axis_des[1],
                                      axis_sp[2] * axis_des[0] - axis_sp[0] * axis_des[2],
                                      axis_sp[0] * axis_des[1] - axis_sp[1] * axis_des[0]);
      EXPECT_LE(cross, tol) << n;
    }
    EXPECT_NEAR(tilt_of(out.q_sp), kTiltMaxD * std::min(1.0, n_stick), tol) << n;
  }
}

// ---- braking and the crossing -----------------------------------------------------------------------------------

// A run: `hold` executions with the yaw stick at full (psi advancing at the rate omega0), then the release with the
// planted world-down rates `rates` (rates[0] at the release execution n_r), the stick centred.
struct Braking {
  std::vector<AnglePhase> phases;   // after each release execution
  std::vector<double> headings;     // psi_m of each release execution
  std::vector<AngleOutput<float>> outs;
  double heading_start = 0;         // psi_m at the first yaw-stick execution
  double lock = 0;
  Rig rig;
};

Braking run_release(const std::vector<double>& rates, double omega0, float tilt_roll_stick = 0.0F) {
  Braking b;
  constexpr int kHold = 20;  // scenario test value: a sustained yaw input of 20 executions
  double psi = 0.3;          // scenario test value: the starting heading
  const double dt = static_cast<double>(kPeriodUsNum) / static_cast<double>(kPeriodUsDen) / kMicro;
  b.heading_start = psi;
  for (int i = 0; i < kHold; ++i) {
    (void)b.rig.step(tilt_roll_stick, 0, 1.0F, Pose{psi, 0, 0, omega0});
    psi += omega0 * dt;
  }
  for (std::size_t i = 0; i < rates.size(); ++i) {
    const Pose p{psi, 0, 0, rates[i]};
    b.outs.push_back(b.rig.step(tilt_roll_stick, 0, 0.0F, p));
    b.phases.push_back(b.rig.mode.phase());
    b.headings.push_back(heading_of(make_state(0, p).q));
    psi += rates[i] * dt;
  }
  b.lock = d(b.rig.mode.heading_lock());
  return b;
}

constexpr double kHeadingTol = 4 * kEps * kPiD;  // atan2 and the quaternion cast: a few rounding units of pi

TEST(L5Angle, TheHeadingLocksAtTheFirstCrossingAndIsTheHeadingOfThatExecution) {
  // Scenario test values: a positive release that decelerates and crosses zero at the fifth release execution, and a
  // negative release that crosses at the fourth.
  const std::vector<double> positive{2.0, 1.2, 0.5, 0.1, -0.2, -0.3};
  const Braking b = run_release(positive, 3.0);
  const std::array<AnglePhase, 6> expect{AnglePhase::Braking, AnglePhase::Braking, AnglePhase::Braking,
                                         AnglePhase::Braking, AnglePhase::Locked,  AnglePhase::Locked};
  for (std::size_t i = 0; i < expect.size(); ++i) {
    EXPECT_EQ(b.phases[i], expect[i]) << i;
  }
  // psi_lock is the heading of the locking execution.
  EXPECT_NEAR(b.lock, b.headings[4], kHeadingTol);
  // Control: a lock at the release heading (decision 14's first form) or at the heading where the yaw input started
  // (the catch-up behaviour) is not the heading of the locking execution.
  const double margin = 4 * kHeadingTol;
  EXPECT_GT(std::fabs(wrap(b.lock - b.headings[0])), margin);
  EXPECT_GT(std::fabs(wrap(b.lock - b.heading_start)), margin);
  // While braking the yaw-rate command is zero and the yaw error is zero.
  const double tol = angle_tol(std::cos(kTiltMaxD / 2));
  for (std::size_t i = 0; i < 4; ++i) {
    EXPECT_EQ(b.outs[i].yaw_rate_cmd, 0.0F) << i;
  }
  // At the locking execution the setpoint heading is the lock.
  EXPECT_NEAR(wrap(heading_of(b.outs[4].q_sp) - b.lock), 0.0, tol);

  // A negative release (sigma_r = -1): the crossing is a positive rate.
  const std::vector<double> negative{-2.5, -1.0, -0.4, 0.3, 0.6};
  const Braking n = run_release(negative, -3.0);
  EXPECT_EQ(n.phases[2], AnglePhase::Braking);
  EXPECT_EQ(n.phases[3], AnglePhase::Locked);
  EXPECT_NEAR(n.lock, n.headings[3], kHeadingTol);

  // A rate of exactly zero crosses: sigma_r psi_dot_m <= 0.
  const Braking z = run_release({1.5, 0.7, 0.0, 0.4}, 3.0);
  EXPECT_EQ(z.phases[1], AnglePhase::Braking);
  EXPECT_EQ(z.phases[2], AnglePhase::Locked);
  EXPECT_NEAR(z.lock, z.headings[2], kHeadingTol);
}

TEST(L5Angle, AReleaseWithZeroRateLocksAtTheReleaseExecution) {
  const Braking b = run_release({0.0, 0.5}, 3.0);
  EXPECT_EQ(b.phases[0], AnglePhase::Locked);
  EXPECT_NEAR(b.lock, b.headings[0], kHeadingTol);
  EXPECT_EQ(b.outs[0].yaw_rate_cmd, 0.0F);
}

// ---- guard (a) --------------------------------------------------------------------------------------------------

TEST(L5Angle, GuardAOneLockPerReleaseEvenWhenTheRateChangesSignRepeatedly) {
  // The release crosses at its second execution; the stick then stays centred while psi_dot_m flips sign repeatedly
  // (scenario test values: +-0.5 rad/s for 8 executions, the gyro-noise case of L6).
  const std::vector<double> rates{1.0, -0.2, 0.5, -0.5, 0.5, -0.5, 0.5, -0.5, 0.5, -0.5};
  Braking b = run_release(rates, 3.0);
  EXPECT_EQ(b.phases[0], AnglePhase::Braking);
  for (std::size_t i = 1; i < rates.size(); ++i) {
    EXPECT_EQ(b.phases[i], AnglePhase::Locked) << i;
  }
  const double lock = b.lock;
  EXPECT_NEAR(lock, b.headings[1], kHeadingTol);
  // The lock and the setpoint are unchanged through every later sign change (the heading still moves: the
  // setpoint must not follow it).
  for (std::size_t i = 2; i < rates.size(); ++i) {
    EXPECT_TRUE(bits_equal(b.outs[i].q_sp, b.outs[2].q_sp)) << i;
  }
  EXPECT_EQ(d(b.rig.mode.heading_lock()), lock);

  // Control: the same sequence with the stick leaving the deadband and returning between two sign changes makes a
  // second, different lock: the test can see a re-lock.
  Braking c = run_release({1.0, -0.2, 0.5, -0.5}, 3.0);
  const double first_lock = c.lock;
  double psi = c.headings.back() + 0.5;  // the vehicle has moved on (scenario test value: half a radian)
  (void)c.rig.step(0, 0, 1.0F, Pose{psi, 0, 0, 2.0});    // leaves the deadband
  EXPECT_EQ(c.rig.mode.phase(), AnglePhase::YawRate);
  psi += 0.5;
  (void)c.rig.step(0, 0, 0.0F, Pose{psi, 0, 0, 1.0});    // back: a new release
  EXPECT_EQ(c.rig.mode.phase(), AnglePhase::Braking);
  psi += 0.5;
  (void)c.rig.step(0, 0, 0.0F, Pose{psi, 0, 0, -0.3});   // a crossing: a second lock
  EXPECT_EQ(c.rig.mode.phase(), AnglePhase::Locked);
  EXPECT_GT(std::fabs(wrap(d(c.rig.mode.heading_lock()) - first_lock)), 0.25);
  EXPECT_NEAR(d(c.rig.mode.heading_lock()), wrap(psi), kHeadingTol);
}

// ---- guard (b) --------------------------------------------------------------------------------------------------

struct Fallback {
  std::size_t lock_index = 0;  // index (from n_r = 0) of the first Locked execution, SIZE_MAX if none in `count`
  std::vector<AnglePhase> phases;
};

// A release at psi_dot = omega_r that never crosses (same sign, constant), on the stamp grid num / den us.
Fallback run_no_crossing(double omega_r, float alpha_min, float t_cross, std::uint64_t num, std::uint64_t den,
                         std::size_t count) {
  AttitudeConfig<float> c = test_config();
  c.yaw_alpha_min = alpha_min;
  c.yaw_t_cross = t_cross;
  Rig rig(c, num, den);
  constexpr int kHold = 3;  // scenario test value
  for (int i = 0; i < kHold; ++i) {
    (void)rig.step(0, 0, 1.0F, Pose{0.3, 0, 0, omega_r});
  }
  Fallback fb;
  fb.lock_index = static_cast<std::size_t>(-1);
  for (std::size_t i = 0; i < count; ++i) {
    (void)rig.step(0, 0, 0.0F, Pose{0.3, 0, 0, omega_r});
    fb.phases.push_back(rig.mode.phase());
    if (fb.lock_index == static_cast<std::size_t>(-1) && rig.mode.phase() == AnglePhase::Locked) {
      fb.lock_index = i;
    }
  }
  return fb;
}

// The first execution index i >= 0 (i = 0 is n_r) with dt >= t_cross AND dt alpha_min >= omega_r, from the stamps, in
// double, independently of the firmware's float evaluation; the parameters keep every test off a tie by at least the
// rounding of a float quotient.
std::size_t expected_fallback_index(double omega_r, double alpha_min, double t_cross, std::uint64_t num,
                                    std::uint64_t den, std::size_t count) {
  for (std::size_t i = 0; i < count; ++i) {
    const double dt = static_cast<double>(i * num / den) / kMicro;
    if (dt >= t_cross && dt * alpha_min >= omega_r) {
      return i;
    }
  }
  return static_cast<std::size_t>(-1);
}

TEST(L5Angle, GuardBFallbackLocksAtTheFirstExecutionWhereBothConditionsHold) {
  // Scenario test values: omega_r = 0.4 rad/s (omega_r / alpha_min = 0.04 s < t_cross: t_cross binds) and
  // omega_r = 6.005 rad/s (0.6005 s > t_cross: the alpha term binds), alpha_min = 10 rad/s^2, t_cross = 0.2504 s; both
  // boundaries lie off the 1 ms stamp grid.
  struct Regime {
    double omega_r;
    const char* name;
  };
  const std::array<Regime, 2> regimes{{{0.4, "t_cross binds"}, {6.005, "alpha binds"}}};
  // Scenario test value: the stamp grids, 1 ms and 312.5 us (the half-microsecond period: stamps floor(n 312.5)).
  const std::array<std::array<std::uint64_t, 2>, 2> grids{{{1000, 1}, {625, 2}}};
  for (const Regime& r : regimes) {
    for (const auto& g : grids) {
      const std::size_t count = static_cast<std::size_t>(0.7 * kMicro / (static_cast<double>(g[0]) / static_cast<double>(g[1])));
      const std::size_t want = expected_fallback_index(r.omega_r, d(kAlphaMin), d(kTCross), g[0], g[1], count);
      ASSERT_NE(want, static_cast<std::size_t>(-1)) << r.name;
      const Fallback fb = run_no_crossing(r.omega_r, kAlphaMin, kTCross, g[0], g[1], count);
      EXPECT_EQ(fb.lock_index, want) << r.name << " grid " << g[0] << "/" << g[1];
      // The execution before is still braking (and every earlier one).
      for (std::size_t i = 0; i < want; ++i) {
        EXPECT_EQ(fb.phases[i], AnglePhase::Braking) << r.name << " " << i;
      }
      // AND, not OR: the first execution past the single condition is still braking.
      std::size_t only_t_cross = static_cast<std::size_t>(-1);
      std::size_t only_alpha = static_cast<std::size_t>(-1);
      for (std::size_t i = 0; i < count; ++i) {
        const double dt = static_cast<double>(i * g[0] / g[1]) / kMicro;
        if (only_t_cross == static_cast<std::size_t>(-1) && dt >= d(kTCross)) {
          only_t_cross = i;
        }
        if (only_alpha == static_cast<std::size_t>(-1) && dt * d(kAlphaMin) >= r.omega_r) {
          only_alpha = i;
        }
      }
      ASSERT_NE(only_t_cross, static_cast<std::size_t>(-1));
      ASSERT_NE(only_alpha, static_cast<std::size_t>(-1));
      EXPECT_EQ(want, std::max(only_t_cross, only_alpha));  // the fallback is at max(t_cross, omega_r / alpha_min)
      if (r.omega_r > 1) {
        // omega_r large: the first execution past t_cross alone (the alpha term is not yet satisfied) is braking.
        EXPECT_LT(only_t_cross, want);
        EXPECT_EQ(fb.phases[only_t_cross], AnglePhase::Braking);
      } else {
        // omega_r small: the first execution past omega_r / alpha_min alone is braking.
        EXPECT_LT(only_alpha, want);
        EXPECT_EQ(fb.phases[only_alpha], AnglePhase::Braking);
      }
    }
  }
}

TEST(L5Angle, AnEarlierCrossingLocksAtTheCrossingNotAtTheFallback) {
  // The same release as the t_cross-binding regime, but the rate crosses zero at the 101st execution (0.1 s, well before
  // t_cross = 0.2504 s): scenario test values.
  constexpr std::size_t kCross = 100;
  Rig rig;
  constexpr double kOmegaR = 0.4;
  (void)rig.step(0, 0, 1.0F, Pose{0.3, 0, 0, kOmegaR});
  std::size_t lock_index = static_cast<std::size_t>(-1);
  for (std::size_t i = 0; i < 400; ++i) {
    (void)rig.step(0, 0, 0.0F, Pose{0.3, 0, 0, i < kCross ? kOmegaR : -0.05});
    if (rig.mode.phase() == AnglePhase::Locked) {
      lock_index = i;
      break;
    }
  }
  EXPECT_EQ(lock_index, kCross);
  // Control: without the crossing the lock is at the fallback, far later.
  const Fallback fb = run_no_crossing(kOmegaR, kAlphaMin, kTCross, kPeriodUsNum, kPeriodUsDen, 400);
  EXPECT_GT(fb.lock_index, kCross);
}

// ---- re-arm -----------------------------------------------------------------------------------------------------

TEST(L5Angle, LeavingTheDeadbandReturnsToYawRateModeFromBrakingAndFromTheLock) {
  // From braking.
  {
    Rig rig;
    (void)rig.step(0, 0, 1.0F, Pose{0.3, 0, 0, 2.0});
    (void)rig.step(0, 0, 0.0F, Pose{0.3, 0, 0, 2.0});
    EXPECT_EQ(rig.mode.phase(), AnglePhase::Braking);
    const AngleOutput<float> o = rig.step(0, 0, -1.0F, Pose{0.3, 0, 0, 2.0});
    EXPECT_EQ(rig.mode.phase(), AnglePhase::YawRate);
    EXPECT_EQ(o.yaw_rate_cmd, -kRateMaxYaw);
    // A later release starts a new braking phase with its own sigma_r (negative now): a positive rate is a crossing
    // only after sigma_r is re-recorded; a negative rate at the new release does not lock.
    (void)rig.step(0, 0, 0.0F, Pose{0.3, 0, 0, -2.0});
    EXPECT_EQ(rig.mode.phase(), AnglePhase::Braking);
    (void)rig.step(0, 0, 0.0F, Pose{0.3, 0, 0, 0.4});
    EXPECT_EQ(rig.mode.phase(), AnglePhase::Locked);
  }
  // From the lock.
  {
    Rig rig;
    (void)rig.step(0, 0, 0.0F, Pose{0.3, 0, 0, 0});
    EXPECT_EQ(rig.mode.phase(), AnglePhase::Locked);
    const AngleOutput<float> o = rig.step(0, 0, 0.5F, Pose{0.3, 0, 0, 0});
    EXPECT_EQ(rig.mode.phase(), AnglePhase::YawRate);
    EXPECT_GT(o.yaw_rate_cmd, 0.0F);
  }
}

// ---- initialisation ---------------------------------------------------------------------------------------------

TEST(L5Angle, TheFirstValidExecutionLocksAtOnceWithACentredStickAndTracksWithAnActiveOne) {
  const Pose p{1.1, 0.2, -0.1, 0.9};
  {
    Rig rig;
    const AttitudeState<float> a = make_state(kT0Us, p);
    const AngleOutput<float> o = rig.step_state(0, 0, 0.0F, a);
    EXPECT_EQ(rig.mode.phase(), AnglePhase::Locked);
    EXPECT_NEAR(d(rig.mode.heading_lock()), heading_of(a.q), kHeadingTol);
    EXPECT_NEAR(wrap(heading_of(o.q_sp) - heading_of(a.q)), 0.0, angle_tol(1));
    EXPECT_EQ(o.yaw_rate_cmd, 0.0F);
  }
  {
    Rig rig;
    const AngleOutput<float> o = rig.step(0, 0, 0.6F, p);
    EXPECT_EQ(rig.mode.phase(), AnglePhase::YawRate);
    EXPECT_GT(o.yaw_rate_cmd, 0.0F);
  }
  // reset() re-initialises: a locked mode locks again at the new heading.
  {
    Rig rig;
    (void)rig.step(0, 0, 0.0F, Pose{0.3, 0, 0, 0});
    const double first = d(rig.mode.heading_lock());
    rig.mode.reset();
    EXPECT_EQ(rig.mode.phase(), AnglePhase::Uninit);
    (void)rig.step(0, 0, 0.0F, Pose{-1.2, 0, 0, 0});
    EXPECT_EQ(rig.mode.phase(), AnglePhase::Locked);
    EXPECT_GT(std::fabs(d(rig.mode.heading_lock()) - first), 1.0);
  }
}

TEST(L5Angle, TheSingularSetOfTheHeadingGivesZero) {
  // q = (0, 1, 0, 0) (a roll of 180 degrees, so q.w = q.z = 0): psi_m = 0 and the same identity q_z branch as in C. A tilt
  // stick on top still gives a finite, valid setpoint.
  Rig rig;
  const AttitudeState<float> a{kT0Us, Q(0, 1, 0, 0), V3(), true};
  const AngleOutput<float> o = rig.step_state(0, 0, 0.0F, a);
  ASSERT_TRUE(o.valid);
  EXPECT_EQ(rig.mode.phase(), AnglePhase::Locked);
  EXPECT_EQ(rig.mode.heading_lock(), 0.0F);
  // Control: a neighbouring attitude (w small but nonzero) has a heading that is not forced to zero.
  Rig near;
  const AttitudeState<float> b{kT0Us, Q(0.0F, 0.6F, 0.0F, 0.8F), V3(), true};  // w = 0, z != 0: heading = pi (canonical)
  (void)near.step_state(0, 0, 0.0F, b);
  EXPECT_NEAR(std::fabs(d(near.mode.heading_lock())), kPiD, kHeadingTol);
}

// ---- faults -----------------------------------------------------------------------------------------------------

struct FaultCase {
  const char* name;
  AngleSticks<float> sticks;
  AttitudeState<float> a;
  AttitudeState<float> twin;
};

TEST(L5Angle, EachFaultInputDropsTheBrakingStateAndTheLockAndItsTwinPasses) {
  const AttitudeState<float> ok = make_state(kT0Us, Pose{0.4, 0.1, 0.1, 0.3});
  const AngleSticks<float> sticks_ok{0.2F, -0.3F, 0.0F};
  const auto with_a = [&](auto&& edit) {
    AttitudeState<float> a = ok;
    edit(a);
    return a;
  };
  const auto with_s = [&](auto&& edit) {
    AngleSticks<float> s = sticks_ok;
    edit(s);
    return s;
  };
  struct Case {
    const char* name;
    AngleSticks<float> s;
    AttitudeState<float> a;
  };
  const std::vector<Case> cases{
      {"valid flag clear", sticks_ok, with_a([](AttitudeState<float>& a) { a.valid = false; })},
      {"q NaN", sticks_ok, with_a([](AttitudeState<float>& a) { a.q.x = kNaN; })},
      {"q zero", sticks_ok, with_a([](AttitudeState<float>& a) { a.q = Q(0, 0, 0, 0); })},
      {"q norm overflows", sticks_ok, with_a([](AttitudeState<float>& a) { a.q = Q(kMaxFloat, kMaxFloat, 0, 0); })},
      {"omega NaN", sticks_ok, with_a([](AttitudeState<float>& a) { a.omega_frd[1] = kNaN; })},
      {"omega infinite", sticks_ok, with_a([](AttitudeState<float>& a) { a.omega_frd[0] = kInf; })},
      {"omega overflows the rotation", sticks_ok,
       with_a([](AttitudeState<float>& a) {
         a.q = Q(0, 1, 0, 0);
         a.omega_frd = V3(0, kMaxFloat, kMaxFloat);
       })},
      {"roll stick NaN", with_s([](AngleSticks<float>& s) { s.roll = kNaN; }), ok},
      {"pitch stick infinite", with_s([](AngleSticks<float>& s) { s.pitch = kInf; }), ok},
      {"yaw stick NaN", with_s([](AngleSticks<float>& s) { s.yaw = kNaN; }), ok},
      {"roll stick above 1", with_s([](AngleSticks<float>& s) { s.roll = std::nextafter(1.0F, 2.0F); }), ok},
      {"pitch stick below -1", with_s([](AngleSticks<float>& s) { s.pitch = -std::nextafter(1.0F, 2.0F); }), ok},
      {"yaw stick above 1", with_s([](AngleSticks<float>& s) { s.yaw = 2.0F; }), ok},
  };
  for (const Case& c : cases) {
    // The valid twin passes from a fresh generator (the same sticks and attitude without the fault).
    Rig twin;
    EXPECT_TRUE(twin.step_state(sticks_ok.roll, sticks_ok.pitch, sticks_ok.yaw, ok).valid) << c.name;

    // From the lock: a fault drops it; the next valid execution re-initialises at the then-current heading.
    Rig locked;
    (void)locked.step(0, 0, 0.0F, Pose{0.3, 0, 0, 0});
    ASSERT_EQ(locked.mode.phase(), AnglePhase::Locked);
    const AngleOutput<float> bad = locked.step_state(c.s.roll, c.s.pitch, c.s.yaw, c.a);
    EXPECT_FALSE(bad.valid) << c.name;
    EXPECT_EQ(bad.yaw_rate_cmd, 0.0F) << c.name;
    EXPECT_EQ(bad.q_sp.w, 0.0F) << c.name;  // the zero quaternion: an invalid setpoint for the attitude law
    EXPECT_EQ(bad.q_sp.x, 0.0F) << c.name;
    EXPECT_EQ(bad.q_sp.y, 0.0F) << c.name;
    EXPECT_EQ(bad.q_sp.z, 0.0F) << c.name;
    EXPECT_EQ(locked.mode.phase(), AnglePhase::Uninit) << c.name;
    const Pose later{-1.4, 0, 0, 0};
    (void)locked.step(0, 0, 0.0F, later);
    EXPECT_EQ(locked.mode.phase(), AnglePhase::Locked) << c.name;
    EXPECT_NEAR(d(locked.mode.heading_lock()), later.psi, kHeadingTol) << c.name;

    // From braking: a fault drops it; the next valid execution with a centred stick locks at once (no stale braking).
    Rig braking;
    (void)braking.step(0, 0, 1.0F, Pose{0.3, 0, 0, 2.0});
    (void)braking.step(0, 0, 0.0F, Pose{0.3, 0, 0, 2.0});
    ASSERT_EQ(braking.mode.phase(), AnglePhase::Braking);
    EXPECT_FALSE(braking.step_state(c.s.roll, c.s.pitch, c.s.yaw, c.a).valid) << c.name;
    EXPECT_EQ(braking.mode.phase(), AnglePhase::Uninit) << c.name;
    (void)braking.step(0, 0, 0.0F, Pose{0.35, 0, 0, 2.0});
    EXPECT_EQ(braking.mode.phase(), AnglePhase::Locked) << c.name;

    // Straight into the attitude law the invalid setpoint is the fault path (zero rate setpoint, fault_active).
    AttitudeLaw<float> law;
    law.init(test_config());
    const AttitudeState<float> meas = ok;
    const attitude::AttitudeOutput<float> r = law.execute(meas, bad.q_sp, bad.yaw_rate_cmd);
    EXPECT_TRUE(r.fault_active) << c.name;
    EXPECT_EQ(r.rate_setpoint[0], 0.0F) << c.name;
  }
}

}  // namespace
