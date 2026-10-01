// L6 stage (b), T1 (decision 0013): the rotor mechanical angle of the plant's motor model.
//
// Oracle. Over a time t from speed w0 under a held command c the speed is c + (w0 - c) exp(-t/tau), so the angle turned
// is  c t + (w0 - c) tau (1 - exp(-t/tau)),  evaluated DIRECTLY at t = n h in long double (never by iterating), reduced
// mod 2 pi in long double, and compared to the model's wrapped angle by circular distance.
//
// Bound (n = sub-steps taken, scale = max(c, w0), h = sub-step, eps = DBL_EPSILON; every angle increment is below
// 2 pi, so the wrap subtracts one exact 2 pi at most). Per sub-step the model's own rounding is
//  - the increment c s + (w - c) tau (1 - e^{-s/tau}): c s one product (eps/2), (w - c) one subtraction (eps/2),
//    1 - e^{-s/tau} from expm1 (the division eps/2 of |x| <= 1, expm1 one ulp: 1.5 eps), two products and the sum
//    (eps/2 each) -> at most 4.5 eps scale h, with tau (1 - e^{-s/tau}) <= s;
//  - the addition theta + increment, below 4 pi: half an ulp of a value in [8, 16), which is 4 eps;
//  - the wrap: the double 2 pi is 2.45e-16 rad = 1.1 eps below the real 2 pi (the subtraction itself is exact,
//    Sterbenz), once per sub-step at most.
//  That is eps (4.5 scale h + 6) per sub-step (4 + 1.1 <= 6), added over n.
//  Carried speed error (a transient run; none in steady state, where w = c exactly and (w - c) = 0): motor_test.cpp
//  derives |d omega| <= 3 k eps scale after k sub-steps, and d theta/d omega <= h per sub-step, so
//  sum_k 3 k eps scale h = 1.5 h n^2 eps scale.
//  A partial sub-step (dt not a multiple of h; `rest` = dt - whole h): the time carries an error <= eps/2 dt, which
//  turns the angle by <= scale dt eps/2 and the speed by 2 eps scale (motor_test.cpp), integrated over at most n
//  sub-steps: per call scale eps (dt/2 + 2 h n).
// The long-double reference adds at most 2^-64 of its magnitude (n scale h): charged as n scale h 1e-19 (2^-64 = 5.4e-20,
// doubled).
//
// Negative controls: the same run against an oracle that is wrong in the way an implementation could be (the speed
// scaled, the transient term dropped, a sub-step of time late) must fall outside the bound.
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

#include "plant_fixture.hpp"
#include "plant_model.hpp"

namespace {

using namespace marv::plant::test;
using marv::plant::Model;
using marv::plant::Params;

constexpr double kEpsD = std::numeric_limits<double>::epsilon();
constexpr long double kTwoPiLd = 2.0L * 3.141592653589793238462643383279502884L;  // pi to 36 digits (long double has 19)
constexpr double kTwoPiD = 0x1.921fb54442d18p+2;                                  // the double nearest 2 pi

Params<double> params(double h, double w0) {
  Params<double> p;
  p.tau = kTau;
  p.substep = h;
  p.omega_min = kOmegaMin;
  p.omega_max = kOmegaMax;
  p.omega0 = {w0, w0, w0, w0};
  return p;
}

double bound(double n, double n_calls, double scale, double h, double dt, bool carried, bool partial) {
  double b = n * kEpsD * (4.5 * scale * h + 6.0) + n * scale * h * 1.0e-19;
  if (carried) {
    b += 1.5 * h * n * n * kEpsD * scale;
  }
  if (partial) {
    b += n_calls * scale * kEpsD * (0.5 * dt + 2.0 * h * n);
  }
  return b;
}

// Circular distance on [0, 2 pi) between a model angle and an unwrapped long-double angle.
long double circ(double theta, long double unwrapped) {
  const long double r = std::fmod(unwrapped, kTwoPiLd);
  const long double d = std::fabs(static_cast<long double>(theta) - r);
  return std::min(d, kTwoPiLd - d);
}

struct RunResult {
  double worst_ratio = 0.0;  // max over steps and motors of circular error / bound
  bool in_range = true;      // every angle in [0, 2 pi) at every step
  bool monotone_start = true;
};

// Runs the model for `calls` calls of advance(dt) under the held `dshot`, from speed w0, comparing after every call to
// the oracle  c t + (w0 - c) tau (1 - e^{-t/tau}) with t = k dt and the oracle's variant:
//   speed_scale: the oracle's c and w0 are multiplied by it (control: 1 + 1e-9),
//   drop_transient: the oracle omits the (w0 - c) term (control),
//   late: the oracle's time is (k - 1) dt (control, one call late).
RunResult run(double h, double w0, int dshot, double dt, int calls, bool partial, double speed_scale = 1.0,
        bool drop_transient = false, bool late = false) {
  Model<double> model(params(h, w0));
  const double c = model.omega_cmd(static_cast<std::uint16_t>(dshot));
  const double scale = std::max(c, w0);
  const bool carried = w0 != c;
  RunResult r;
  for (int k = 1; k <= calls; ++k) {
    model.advance({static_cast<std::uint16_t>(dshot), static_cast<std::uint16_t>(dshot),
                   static_cast<std::uint16_t>(dshot), static_cast<std::uint16_t>(dshot)}, dt);
    const long double t = static_cast<long double>(late ? k - 1 : k) * static_cast<long double>(dt);
    const long double cc = static_cast<long double>(c) * speed_scale;
    const long double ww = static_cast<long double>(w0) * speed_scale;
    long double turned = cc * t;
    if (!drop_transient) {
      turned += (ww - cc) * static_cast<long double>(kTau) * (-std::expm1(-t / static_cast<long double>(kTau)));
    }
    const double n_sub = static_cast<double>(k) * (partial ? std::ceil(dt / h) : 1.0);
    const double b = bound(n_sub, static_cast<double>(k), scale, h, dt, carried, partial);
    for (double th : model.theta()) {
      r.in_range = r.in_range && th >= 0.0 && th < kTwoPiD;
      r.worst_ratio = std::max(r.worst_ratio, static_cast<double>(circ(th, turned)) / b);
    }
  }
  return r;
}

// Labelled test values.
//  kSteadyCalls: 2000 sub-steps (10 s at h = 5 ms), so the angle wraps about 2000 * 6.15 / 6.28 ~ 1960 times.
//  kTransientH: 1 ms, so w h <= 3.8 < 2 pi for every speed the runs use (the increment-below-2-pi precondition).
//  kTransientCalls: 600 sub-steps = 0.6 s = 20 tau, the transient has decayed to 2e-9 of its start.
//  kPartialDtFactor: 2.5 sub-steps per call, so each call is two whole sub-steps and a half one.
constexpr int kSteadyCalls = 2000;
constexpr double kTransientH = 0.001;
constexpr int kTransientCalls = 600;
constexpr double kPartialDtFactor = 2.5;
constexpr int kPartialCalls = 200;
// A control must exceed the bound by a clear factor (the bound is a worst case, the real error is far below it).
constexpr double kControlRatio = 10.0;

TEST(RotorAngle, SteadySpeedAdvancesExactlyOmegaDtPerSubstep) {
  // Steady state: the speed starts at its command, so every sub-step turns c h (the second term is exactly 0). DShot 48
  // gives c = 250 (c h = 1.25 rad); DShot 600 gives c ~ 1230 (c h = 6.15 rad, just below 2 pi, wrapping at almost every step).
  for (int dshot : {48, 600}) {
    Model<double> probe(params(kSubstep, 0.0));
    const double c = probe.omega_cmd(static_cast<std::uint16_t>(dshot));
    const RunResult r = run(kSubstep, c, dshot, kSubstep, kSteadyCalls, false);
    EXPECT_TRUE(r.in_range) << "dshot " << dshot;
    EXPECT_LT(r.worst_ratio, 1.0) << "dshot " << dshot;
    // Reality check that the bound is not vacuous: the model is well inside it, and far from the controls' error.
    EXPECT_GT(r.worst_ratio, 0.0) << "dshot " << dshot;
    // Control: an oracle whose speed is 1e-9 high is out of bound.
    EXPECT_GT(run(kSubstep, c, dshot, kSubstep, kSteadyCalls, false, 1.0 + 1.0e-9).worst_ratio, kControlRatio)
        << "dshot " << dshot;
  }
}

TEST(RotorAngle, SteadyFirstStepIsExactlyOmegaDt) {
  // One sub-step from steady state: theta = fl(c h), bitwise (0 + fl(c h), and c h < 2 pi so the wrap leaves it). DShot 600
  // gives c h = 6.15 rad.
  Model<double> model(params(kSubstep, 0.0));
  const double c = model.omega_cmd(600);
  Model<double> steady(params(kSubstep, c));
  steady.advance({600, 600, 600, 600}, kSubstep);
  EXPECT_EQ(steady.omega()[0], c);
  EXPECT_EQ(steady.theta()[0], c * kSubstep);
}

TEST(RotorAngle, TransientMatchesClosedFormIntegral) {
  // Down from 2000 rad/s to the DShot-48 command 250, and up from rest to the DShot-2047 command 3800 rad/s: both signs
  // of (w0 - c), at every sub-step.
  struct Case { double w0; int dshot; };
  for (const Case& cs : {Case{2000.0, 48}, Case{0.0, 2047}}) {
    const RunResult r = run(kTransientH, cs.w0, cs.dshot, kTransientH, kTransientCalls, false);
    EXPECT_TRUE(r.in_range);
    EXPECT_LT(r.worst_ratio, 1.0) << "w0 " << cs.w0;
    EXPECT_GT(run(kTransientH, cs.w0, cs.dshot, kTransientH, kTransientCalls, false, 1.0 + 1.0e-9).worst_ratio, kControlRatio);
    EXPECT_GT(run(kTransientH, cs.w0, cs.dshot, kTransientH, kTransientCalls, false, 1.0, true).worst_ratio,
              kControlRatio);
    EXPECT_GT(run(kTransientH, cs.w0, cs.dshot, kTransientH, kTransientCalls, false, 1.0, false, true).worst_ratio,
              kControlRatio);
  }
}

TEST(RotorAngle, PartialSubstepsMatchClosedFormIntegral) {
  // Each call advances 2.5 sub-steps (two whole, one partial), so the partial path is on every call.
  const double dt = kPartialDtFactor * kTransientH;
  const RunResult r = run(kTransientH, 0.0, 2047, dt, kPartialCalls, true);
  EXPECT_TRUE(r.in_range);
  EXPECT_LT(r.worst_ratio, 1.0);
  EXPECT_GT(run(kTransientH, 0.0, 2047, dt, kPartialCalls, true, 1.0 + 1.0e-9).worst_ratio, kControlRatio);
  EXPECT_GT(run(kTransientH, 0.0, 2047, dt, kPartialCalls, true, 1.0, true).worst_ratio, kControlRatio);
}

TEST(RotorAngle, RotorsAreIndependentAndStartAtZero) {
  // Four different commands: each rotor's angle follows its own speed (a swapped index would not match the oracle),
  // and before any step every angle is 0.
  Model<double> model(params(kTransientH, 0.0));
  for (double th : model.theta()) {
    EXPECT_EQ(th, 0.0);
  }
  const std::array<std::uint16_t, 4> cmd = {300, 800, 1400, 2047};
  const int calls = 50;  // labelled: 50 sub-steps, the transient is still strongly active
  for (int k = 0; k < calls; ++k) {
    model.advance(cmd, kTransientH);
  }
  for (std::size_t i = 0; i < 4; ++i) {
    const long double c = model.omega_cmd(cmd[i]);
    const long double t = static_cast<long double>(calls) * kTransientH;
    const long double turned = c * t - c * kTau * (-std::expm1(-t / static_cast<long double>(kTau)));
    const double b = bound(calls, calls, static_cast<double>(c), kTransientH, kTransientH, true, false);
    EXPECT_LT(static_cast<double>(circ(model.theta()[i], turned)), b) << "rotor " << i;
    const long double other = model.omega_cmd(cmd[(i + 1) % 4]);
    const long double swapped = other * t - other * kTau * (-std::expm1(-t / static_cast<long double>(kTau)));
    EXPECT_GT(static_cast<double>(circ(model.theta()[i], swapped)), b) << "control, rotor " << i;
  }
}

}  // namespace
