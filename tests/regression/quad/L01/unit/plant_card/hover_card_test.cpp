// Frozen path: tests/regression/quad/L01/unit/plant_card/hover_card_test.cpp
// T1 hover consistency check with the real vehicle card: the plant configuration comes from the header that
// tools/card/gen_plant_config.py generates from the product card (MARV_PLANT_CARD_HEADER, MARV_PLANT_CARD_FILL).
//
// The check is circular by construction: omega_h is derived from m, g and k, so it tests the arithmetic of thrust,
// summation, rotation and gravity against each other, not the physics of the vehicle.
//
// The ESC map is linear in omega, so the DShot value that gives exactly omega_h is in general not an integer, and
// the plant's motor state cannot be set through the C ABI (the internal Model in sim/plant/src is not exposed to
// tests and has no setter either). The check therefore drives all four motors to steady state at the two DShot
// values that bracket omega_h and shows that the net vertical force changes sign between them: thrust below m g at
// the lower value, above at the upper. The steady net force at a value D is m g - 4 k omega(D)^2, monotonic in D,
// so the exact hover point lies inside the bracket; and each net force is bounded by the thrust spacing of one
// DShot step, which is the analytic quantization error of the hover command.
//
// Tolerance: settled_rel_tol() (plant_fixture.hpp, derivation there) for the number of sub-steps run, plus
// kEscExtraOps below, times the scale m g, which dominates every term summed into the checked force.

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <cstdint>

#include MARV_PLANT_CARD_HEADER
#include "../plant/plant_fixture.hpp"
#include "marv/prim/gravity.hpp"
#include "marv_plant.h"

namespace marv::plant::test {
namespace {

// scenario value (L1 test fixture): the site of the check; not vehicle data and not from the card.
constexpr double kScenarioLat = 0.82;      // rad
constexpr double kScenarioHeight = 500.0;  // m
// scenario value (L1 test fixture): motor sub-step h = tau / kSubstepsPerTau, so h <= tau / 8; tau is the card's,
// read from the generated header.
constexpr double kSubstepsPerTau = 8.0;
// The ESC map evaluated in the restated oracle and in the plant differ in operation order: at most ~6 roundings on
// omega_cmd, so <= 6 eps on omega and <= 12 eps on T = k omega^2; 40 eps is that with the margin the wrench test
// uses for the same map (wrench_test.cpp, kHoverExtraOps).
constexpr double kEscExtraOps = 40.0;
// omega_h = sqrt(m g / (4 k)): the quotient (2 roundings with the product 4 k) and the sqrt (1) give <= 2.5 eps on
// omega_h, since the sqrt halves the relative error of its argument; T = 4 k omega_h^2 doubles that (5 eps) and adds
// 3 roundings: 8 eps.
constexpr double kExactHoverOps = 8.0;

marv_plant_config card_config() {
  marv_plant_config c{};
  MARV_PLANT_CARD_FILL(&c);
  return c;
}

marv_plant_config scenario_config(marv_plant_config c) {
  c.site_lat_rad = kScenarioLat;
  c.site_height_m = kScenarioHeight;
  c.motor_substep_s = c.motor_tau_s / kSubstepsPerTau;
  return c;
}

double settle_substeps() { return kSettleTaus * kSubstepsPerTau + 1.0; }  // whole sub-steps of the step, plus a partial

struct HoverVerdict {
  bool range_ok = false;        // omega_h lies inside the ESC map, so both bracketing DShot values are legal
  bool bracket_ok = false;      // omega(D_lo) < omega_h < omega(D_hi)
  bool lower_below = false;     // plant net force at D_lo is downward beyond the tolerance (thrust < m g)
  bool upper_above = false;     // plant net force at D_hi is upward beyond the tolerance (thrust > m g)
  bool matches_oracle = false;  // both net forces equal m g - 4 k omega(D)^2 within the tolerance
  bool within_step = false;     // both net forces are bounded by the thrust spacing of one DShot step
  bool lateral_zero = false;    // level attitude: no x or y force
  bool all() const {
    return range_ok && bracket_ok && lower_below && upper_above && matches_oracle && within_step && lateral_zero;
  }
};

// `card` is the reference: omega_h, the bracket and the oracle come from it. `plant_cfg` is what the plant is built
// with; it equals `card` in the real check and differs in the negative control.
HoverVerdict hover_check(const marv_plant_config& card, const marv_plant_config& plant_cfg) {
  HoverVerdict v;
  const double g = prim::normal_gravity<double>(card.site_lat_rad, card.site_height_m);  // body at pos_ned_m[2] = 0
  const double weight = card.mass_kg * g;
  const double omega_h = std::sqrt(card.mass_kg * g / (4.0 * card.thrust_coeff));
  const double span = static_cast<double>(kDshotMax - kDshotMin);
  const double d_h = static_cast<double>(kDshotMin) +
                     (omega_h - card.omega_min_rad_s) * span / (card.omega_max_rad_s - card.omega_min_rad_s);
  v.range_ok = d_h >= static_cast<double>(kDshotMin) && d_h < static_cast<double>(kDshotMax);
  if (!v.range_ok) {
    return v;
  }
  const int d_lo = static_cast<int>(std::floor(d_h));
  const int d_hi = d_lo + 1;
  const double w_lo = esc_omega(card, d_lo);
  const double w_hi = esc_omega(card, d_hi);
  v.bracket_ok = w_lo < omega_h && omega_h < w_hi;
  const double t_lo = 4.0 * card.thrust_coeff * w_lo * w_lo;
  const double t_hi = 4.0 * card.thrust_coeff * w_hi * w_hi;

  const double tol = (settled_rel_tol(settle_substeps()) + kEscExtraOps * kEps) * weight;
  const double dt = kSettleTaus * plant_cfg.motor_tau_s;

  double fz[2] = {0.0, 0.0};
  double lateral = 0.0;
  const int d_of[2] = {d_lo, d_hi};
  for (std::size_t i = 0; i < 2; ++i) {
    marv_plant* p = nullptr;
    EXPECT_EQ(marv_plant_create(&plant_cfg, &p), MARV_PLANT_OK);
    const marv_plant_body body = identity_body();
    const std::uint16_t d = static_cast<std::uint16_t>(d_of[i]);
    const marv_plant_cmd cmd = make_cmd(d, d, d, d);
    marv_plant_out out = make_out();
    EXPECT_EQ(marv_plant_step(p, &body, &cmd, dt, &out), MARV_PLANT_OK);
    marv_plant_destroy(p);
    fz[i] = out.force_ned_n[2];
    lateral = std::fmax(lateral, std::fmax(std::fabs(out.force_ned_n[0]), std::fabs(out.force_ned_n[1])));
  }
  v.lower_below = fz[0] > tol;
  v.upper_above = fz[1] < -tol;
  v.matches_oracle = std::fabs(fz[0] - (weight - t_lo)) <= tol && std::fabs(fz[1] - (weight - t_hi)) <= tol;
  const double step_thrust = t_hi - t_lo;
  v.within_step = std::fabs(fz[0]) <= step_thrust && std::fabs(fz[1]) <= step_thrust;
  v.lateral_zero = lateral <= tol;

  // The exact-omega_h level: thrust at omega_h equals m g within the rounding of the derivation.
  EXPECT_NEAR(4.0 * card.thrust_coeff * omega_h * omega_h, weight, kExactHoverOps * kEps * weight);
  return v;
}

TEST(PlantCardHover, ConsistencyCheckCircularByConstructionLuisItem6) {
  const marv_plant_config cfg = scenario_config(card_config());
  ASSERT_LE(cfg.motor_substep_s, cfg.motor_tau_s / 8.0);
  const HoverVerdict v = hover_check(cfg, cfg);
  EXPECT_TRUE(v.range_ok);
  EXPECT_TRUE(v.bracket_ok);
  EXPECT_TRUE(v.lower_below);
  EXPECT_TRUE(v.upper_above);
  EXPECT_TRUE(v.matches_oracle);
  EXPECT_TRUE(v.within_step);
  EXPECT_TRUE(v.lateral_zero);
}

// The card values reach the plant: the generated fill sets struct_size, the spin text gives the signs of core
// contracts section 3 (1 and 2 ccw = +1, 3 and 4 cw = -1), and the plant accepts the configuration.
TEST(PlantCardHover, CardConfigurationIsAcceptedByThePlant) {
  const marv_plant_config cfg = scenario_config(card_config());
  EXPECT_EQ(cfg.struct_size, sizeof(marv_plant_config));
  for (std::size_t i = 0; i < MARV_PLANT_N_MOTORS; ++i) {
    EXPECT_EQ(cfg.yaw_sign[i], kSpinTable[i]) << "motor " << i + 1;
  }
  marv_plant* p = nullptr;
  EXPECT_EQ(marv_plant_create(&cfg, &p), MARV_PLANT_OK);
  marv_plant_destroy(p);
}

// Negative control: the plant is built with the mass scaled by 1 + 2 w and by 1 - 2 w, where w is the relative
// thrust spacing of one DShot step at hover (d(omega^2) / omega^2 = 2 d(omega) / omega, one step d(omega) =
// (omega_max - omega_min) / (2047 - 48)). With 1 + 2 w the plant weighs more than the thrust at the upper bracket
// value, so the net force there is still downward; with 1 - 2 w it weighs less than the thrust at the lower value, so
// the net force there is upward. The same check must fail in both directions. 2 w is asserted far above the
// tolerance, so the failure is the scale and not a rounding accident.
TEST(PlantCardHover, NegativeControlScaledMassFails) {
  const marv_plant_config card = scenario_config(card_config());
  const double g = prim::normal_gravity<double>(card.site_lat_rad, card.site_height_m);
  const double omega_h = std::sqrt(card.mass_kg * g / (4.0 * card.thrust_coeff));
  const double omega_step = (card.omega_max_rad_s - card.omega_min_rad_s) / static_cast<double>(kDshotMax - kDshotMin);
  const double w = 2.0 * omega_step / omega_h;
  const double rel_tol = settled_rel_tol(settle_substeps()) + kEscExtraOps * kEps;
  ASSERT_GT(2.0 * w, 100.0 * rel_tol);

  marv_plant_config heavy = card;
  heavy.mass_kg = card.mass_kg * (1.0 + 2.0 * w);
  const HoverVerdict vh = hover_check(card, heavy);
  EXPECT_FALSE(vh.all());
  EXPECT_FALSE(vh.upper_above);

  marv_plant_config light = card;
  light.mass_kg = card.mass_kg * (1.0 - 2.0 * w);
  const HoverVerdict vl = hover_check(card, light);
  EXPECT_FALSE(vl.all());
  EXPECT_FALSE(vl.lower_below);
}

}  // namespace
}  // namespace marv::plant::test
