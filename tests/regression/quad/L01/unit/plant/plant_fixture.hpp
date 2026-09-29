// Frozen path: tests/regression/quad/L01/unit/plant/plant_fixture.hpp
// Shared fixture of the marv_plant v0 T1 tests. Every numeric value in this file is a scenario value (L1 test
// fixture): a labelled choice that gives a plausible 5-inch-class quad, not a measurement and not the vehicle card.
// The card-based hover test is a later packet.
#pragma once

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

#include "marv_plant.h"

namespace marv::plant::test {

constexpr double kEps = std::numeric_limits<double>::epsilon();

// Restated DShot range (Betaflight DShot notes): 0 = stop, 48..2047 = throttle.
constexpr int kDshotMin = 48;
constexpr int kDshotMax = 2047;

// scenario value (L1 test fixture)
constexpr double kMass = 0.75;             // kg
constexpr double kArmX = 0.11;             // m, |x_FRD| of every rotor
constexpr double kArmY = 0.09;             // m, |y_FRD| of every rotor (unequal to kArmX so x/y mix-ups show)
constexpr double kThrustCoeff = 2.0e-7;    // N / (rad/s)^2
constexpr double kTorqueRatio = 0.016;     // m
constexpr double kOmegaMin = 250.0;        // rad/s at DShot 48
constexpr double kOmegaMax = 3800.0;       // rad/s at DShot 2047
constexpr double kTau = 0.03;              // s
constexpr double kSubstep = 0.005;         // s
constexpr std::uint32_t kPoles = 14;
constexpr double kLat = 0.82;              // rad
constexpr double kSiteHeight = 500.0;      // m

// Spin text (core contracts section 3): quad X seen from above, 1 front-right, 2 rear-left, 3 front-left,
// 4 rear-right; 1 and 2 counter-clockwise (reaction torque +z_FRD), 3 and 4 clockwise.
constexpr int kSpinTable[MARV_PLANT_N_MOTORS] = {1, 1, -1, -1};

inline marv_plant_config fixture_config() {
  marv_plant_config c{};
  c.struct_size = sizeof(c);
  c.esc_map = MARV_PLANT_ESC_LINEAR_IN_OMEGA;
  c.pole_count = kPoles;
  const double sx[MARV_PLANT_N_MOTORS] = {1.0, -1.0, 1.0, -1.0};
  const double sy[MARV_PLANT_N_MOTORS] = {1.0, -1.0, -1.0, 1.0};
  for (std::size_t i = 0; i < MARV_PLANT_N_MOTORS; ++i) {
    c.yaw_sign[i] = kSpinTable[i];
    c.rotor_position_frd_m[i][0] = sx[i] * kArmX;
    c.rotor_position_frd_m[i][1] = sy[i] * kArmY;
    c.rotor_position_frd_m[i][2] = 0.0;
  }
  c.mass_kg = kMass;
  c.thrust_coeff = kThrustCoeff;
  c.torque_ratio_m = kTorqueRatio;
  c.omega_min_rad_s = kOmegaMin;
  c.omega_max_rad_s = kOmegaMax;
  c.motor_tau_s = kTau;
  c.motor_substep_s = kSubstep;
  c.site_lat_rad = kLat;
  c.site_height_m = kSiteHeight;
  return c;
}

inline marv_plant_body identity_body() {
  marv_plant_body b{};
  b.struct_size = sizeof(b);
  b.q_wxyz[0] = 1.0;
  return b;
}

inline marv_plant_cmd make_cmd(std::uint16_t d0, std::uint16_t d1, std::uint16_t d2, std::uint16_t d3) {
  marv_plant_cmd c{};
  c.struct_size = sizeof(c);
  c.dshot[0] = d0;
  c.dshot[1] = d1;
  c.dshot[2] = d2;
  c.dshot[3] = d3;
  return c;
}

inline marv_plant_out make_out() {
  marv_plant_out o{};
  o.struct_size = sizeof(o);
  return o;
}

// The ESC map, restated independently of the implementation.
inline double esc_omega(const marv_plant_config& c, int dshot) {
  if (dshot == 0) {
    return 0.0;
  }
  return c.omega_min_rad_s + (c.omega_max_rad_s - c.omega_min_rad_s) * static_cast<double>(dshot - kDshotMin) /
                                 static_cast<double>(kDshotMax - kDshotMin);
}

// Rotation matrix of a unit quaternion (w, u): R = (w^2 - u.u) I + 2 u u^T + 2 w [u]x. Independent of marv::prim.
inline void rotate_to_ned(const double q[4], const double v[3], double out[3]) {
  const double w = q[0];
  const double u[3] = {q[1], q[2], q[3]};
  const double d = w * w - (u[0] * u[0] + u[1] * u[1] + u[2] * u[2]);
  const double uv = u[0] * v[0] + u[1] * v[1] + u[2] * v[2];
  const double c[3] = {u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]};
  for (std::size_t i = 0; i < 3; ++i) {
    out[i] = d * v[i] + 2.0 * u[i] * uv + 2.0 * w * c[i];
  }
}

// Steady-state settling: one step of kSettleTaus time constants, whole sub-steps (kSettleTaus * kTau / kSubstep = 300).
constexpr double kSettleTaus = 50.0;
constexpr double kSettleSubsteps = 300.0;
constexpr double kSettleDt = kSettleTaus * kTau;

// Relative tolerance of a wrench computed after settling n substeps, derived once here for every test that settles.
//  - Rotor speed. One sub-step is omega' = c + (omega - c) a with a = exp(-h/tau). Rounding of a: the division h/tau
//    (eps/2 of |x| <= 1) plus exp (1 ulp) gives a relative error <= 1.5 eps on a; the subtraction, product and sum
//    add eps/2 each. With |omega - c| <= c the per-sub-step error is <= (1.5 + 1.5) eps c = 3 eps c, and the error
//    already carried is only ever multiplied by a <= 1, so after n sub-steps |d omega| <= 3 n eps c, i.e. a relative
//    error of 3 n eps in omega. The residual exp(-50) ~ 2e-22 is below eps by five orders and is neglected.
//  - Thrust T = k omega^2 doubles that relative error and adds 2 eps: (6 n + 2) eps.
//  - Chain from T to the NED wrench: r x F (3 ops), four-motor sums (4), quaternion normalization (sqrt, divide,
//    product: ~3 on each component) into the rotation matrix entries (~20 combined), the product R v (~5), and for the
//    force the gravity term (sin, square, sqrt, quotient and the height series: ~25). Total <= 64 eps. This is the
//    stated rule; it is added, not multiplied, because those terms are relative to the same scale below.
// The tolerance is relative to a scale S that dominates every term summed into the checked quantity (|F| terms, or
// |r|+ratio times T for the torque), never to a possibly-zero component.
inline double settled_rel_tol(double n_substeps) { return (6.0 * n_substeps + 2.0 + 64.0) * kEps; }

}  // namespace marv::plant::test
