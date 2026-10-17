// Frozen path: tests/regression/quad/L01/unit/plant/motor_test.cpp
// T1 unit test of the marv_plant v0 motor model: first-order step response against the closed form at every sub-step.
//
// Oracle. omega(t) = c + (omega0 - c) exp(-t / tau), evaluated DIRECTLY at t = n h, never by iterating.
//
// Tolerance rule (relative to scale = max(|c|, |omega0|); n = number of sub-steps taken so far, whole or partial).
//  - Plant, per sub-step: a = exp(-h/tau) has relative error <= 1.5 eps (division eps/2 of |x| <= 1, exp 1 ulp); the
//    subtraction, product and sum add eps/2 each; with |omega - c| <= scale that is <= 3 eps scale per sub-step, and
//    the carried error is only multiplied by a <= 1, so the plant deviates by <= 3 n eps scale from the exact
//    recursion. A partial sub-step adds a time error <= eps/2 * dt (the product whole * h), i.e. <= 2 eps scale
//    for dt <= 4 h <= 4 tau; that is charged as 2 more.
//  - Oracle: t = n h and t / tau round (eps/2 each) so the exponent x has absolute error <= x eps; exp(-x) then has
//    error <= e^-x (x + 1) eps <= eps; the subtraction, product and sum add 1.5 eps: <= 2.5 eps scale, charged as 3.
// tol(n) = (3 n + 3 + 2) eps scale.
//
// Negative control: the same sequence with the command applied one sub-step late must fall outside tol(n).

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "marv_plant.h"
#include "plant_fixture.hpp"

namespace marv::plant::test {
namespace {

double closed_form(double c, double w0, double t) { return c + (w0 - c) * std::exp(-t / kTau); }
double tol_of(double n, double scale) { return (3.0 * n + 3.0 + 2.0) * kEps * scale; }

marv_plant* make_plant(double h) {
  marv_plant_config cfg = fixture_config();
  cfg.motor_substep_s = h;
  marv_plant* p = nullptr;
  EXPECT_EQ(marv_plant_create(&cfg, &p), MARV_PLANT_OK);
  return p;
}

// Halving sequence h = tau, tau/2, ..., tau/64.
std::vector<double> substeps() {
  std::vector<double> hs;
  double h = kTau;
  for (int k = 0; k < 7; ++k) {
    hs.push_back(h);
    h *= 0.5;
  }
  return hs;
}

constexpr std::uint16_t kCmdA[MARV_PLANT_N_MOTORS] = {1000, 2047, 48, 1500};
constexpr std::uint16_t kCmdB[MARV_PLANT_N_MOTORS] = {0, 300, 1800, 48};

// Worst ratio |plant - oracle| / tol over every sub-step and motor; `shift` moves the oracle time by that many
// sub-steps (shift 0 is the real comparison). Phase A: from rest under kCmdA for 3 tau. Phase B: from the state
// reached, under kCmdB for 3 tau.
double worst_ratio_step_response(double h, int shift) {
  marv_plant* p = make_plant(h);
  const marv_plant_config cfg = fixture_config();
  const marv_plant_body body = identity_body();
  marv_plant_out out = make_out();
  const std::size_t steps = static_cast<std::size_t>(std::llround(3.0 * kTau / h));
  double worst = 0.0;
  double w0[MARV_PLANT_N_MOTORS] = {};
  const std::uint16_t* cmds[2] = {kCmdA, kCmdB};
  for (const std::uint16_t* d : cmds) {
    const marv_plant_cmd cmd = make_cmd(d[0], d[1], d[2], d[3]);
    for (std::size_t n = 1; n <= steps; ++n) {
      EXPECT_EQ(marv_plant_step(p, &body, &cmd, h, &out), MARV_PLANT_OK);
      for (std::size_t i = 0; i < MARV_PLANT_N_MOTORS; ++i) {
        const double c = esc_omega(cfg, d[i]);
        const double t = static_cast<double>(static_cast<long>(n) - shift) * h;
        const double expected = closed_form(c, w0[i], t);
        const double tol = tol_of(static_cast<double>(n), std::max(std::fabs(c), std::fabs(w0[i])));
        worst = std::max(worst, std::fabs(out.rotor_speed_rad_s[i] - expected) / tol);
      }
    }
    for (std::size_t i = 0; i < MARV_PLANT_N_MOTORS; ++i) {
      w0[i] = out.rotor_speed_rad_s[i];
    }
  }
  marv_plant_destroy(p);
  return worst;
}

TEST(PlantMotor, StepResponseMatchesClosedFormAtEverySubstep) {
  for (double h : substeps()) {
    EXPECT_LE(worst_ratio_step_response(h, 0), 1.0) << "h = " << h;
  }
}

// One call whose dt is not a multiple of h: whole sub-steps of h plus one partial sub-step; and the same call
// repeated, whose result is the closed form at the cumulative time (exact ZOH composes).
TEST(PlantMotor, PartialFinalSubstepMatchesClosedForm) {
  const marv_plant_config cfg = fixture_config();
  const marv_plant_body body = identity_body();
  const marv_plant_cmd cmd = make_cmd(kCmdA[0], kCmdA[1], kCmdA[2], kCmdA[3]);
  const double ratios[] = {3.5, 2.3, 0.3, 1.0000001};
  for (double h : substeps()) {
    for (double r : ratios) {
      const double dt = r * h;
      const double whole = std::floor(dt / h);
      marv_plant* p = make_plant(h);
      marv_plant_out out = make_out();
      const int calls = 40;
      for (int m = 1; m <= calls; ++m) {
        ASSERT_EQ(marv_plant_step(p, &body, &cmd, dt, &out), MARV_PLANT_OK);
        const double t = static_cast<double>(m) * dt;
        const double n = static_cast<double>(m) * (whole + 1.0);
        for (std::size_t i = 0; i < MARV_PLANT_N_MOTORS; ++i) {
          const double c = esc_omega(cfg, kCmdA[i]);
          // The per-call time error (partial-step product) also accumulates over m calls: charged in tol_of via n.
          EXPECT_NEAR(out.rotor_speed_rad_s[i], closed_form(c, 0.0, t), tol_of(n, c))
              << "h = " << h << " ratio = " << r << " call = " << m << " motor = " << i;
        }
      }
      marv_plant_destroy(p);
    }
  }
}

// Negative control: an independent implementation of the model with the command applied one sub-step late
// (motor sees command 0 during the first sub-step, then the real command) must be rejected by the comparison.
TEST(PlantMotor, NegativeControlLateCommandFailsTheComparison) {
  const marv_plant_config cfg = fixture_config();
  for (double h : substeps()) {
    const double a = std::exp(-h / kTau);
    const std::size_t steps = static_cast<std::size_t>(std::llround(3.0 * kTau / h));
    double worst = 0.0;
    for (std::size_t i = 0; i < MARV_PLANT_N_MOTORS; ++i) {
      const double c = esc_omega(cfg, kCmdA[i]);
      double w = 0.0;
      double applied = 0.0;
      for (std::size_t n = 1; n <= steps; ++n) {
        w = applied + (w - applied) * a;
        applied = c;
        const double tol = tol_of(static_cast<double>(n), std::fabs(c));
        worst = std::max(worst, std::fabs(w - closed_form(c, 0.0, static_cast<double>(n) * h)) / tol);
      }
    }
    EXPECT_GT(worst, 1.0e3) << "h = " << h;
  }
  // The same fault injected as a time shift of the oracle against the real plant.
  for (double h : substeps()) {
    EXPECT_GT(worst_ratio_step_response(h, 1), 1.0e3) << "h = " << h;
  }
}

// Known answer for omega_min = 0 (decision 0048 item 1): the ESC map's low end is 0 rad/s, so DShot 48 settles to rotor
// speed 0 (from omega_max, 50 tau: the residual exp(-50) is below eps) and DShot 2047 settles to omega_max. Tolerance
// tol_of(n, omega_max) as above, the closed form evaluated at t = n h.
TEST(PlantMotor, ZeroMinimumSpeedMapsDshot48ToZeroAndDshot2047ToMaximum) {
  marv_plant_config cfg = fixture_config();
  cfg.omega_min_rad_s = 0.0;
  const std::size_t steps = static_cast<std::size_t>(std::llround(50.0 * kTau / cfg.motor_substep_s));
  const marv_plant_body body = identity_body();
  for (const std::uint16_t dshot : {static_cast<std::uint16_t>(kDshotMin), static_cast<std::uint16_t>(kDshotMax)}) {
    const bool low = dshot == kDshotMin;
    marv_plant_config c = cfg;
    for (double& w : c.initial_omega_rad_s) {
      w = low ? c.omega_max_rad_s : 0.0;
    }
    marv_plant* p = nullptr;
    ASSERT_EQ(marv_plant_create(&c, &p), MARV_PLANT_OK);
    const marv_plant_cmd cmd = make_cmd(dshot, dshot, dshot, dshot);
    marv_plant_out out = make_out();
    for (std::size_t n = 0; n < steps; ++n) {
      ASSERT_EQ(marv_plant_step(p, &body, &cmd, c.motor_substep_s, &out), MARV_PLANT_OK);
    }
    const double expected = low ? 0.0 : c.omega_max_rad_s;
    for (std::size_t i = 0; i < MARV_PLANT_N_MOTORS; ++i) {
      EXPECT_NEAR(out.rotor_speed_rad_s[i], expected, tol_of(static_cast<double>(steps), c.omega_max_rad_s))
          << "dshot = " << dshot << " motor = " << i;
    }
    marv_plant_destroy(p);
  }
}

}  // namespace
}  // namespace marv::plant::test
