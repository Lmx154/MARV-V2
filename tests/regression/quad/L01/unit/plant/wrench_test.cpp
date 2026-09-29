// Frozen path: tests/regression/quad/L01/unit/plant/wrench_test.cpp
// T1 unit tests of the marv_plant v0 wrench: yaw torque sign and magnitude per motor, and the hover consistency check.
//
// Tolerances: settled_rel_tol() in plant_fixture.hpp (derivation there), times a scale that dominates every term
// summed into the checked quantity. Every rotor is driven to steady state by one step of 50 tau with a held DShot
// command, so no test hook is needed in the ABI.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <utility>

#include "marv/prim/gravity.hpp"
#include "marv_plant.h"
#include "plant_fixture.hpp"

namespace marv::plant::test {
namespace {

constexpr int kDshotProbe = 1500;  // scenario value (L1 test fixture)

// (2, 4, 5, 6) / 9: a unit quaternion (4 + 16 + 25 + 36 = 81), a non-trivial attitude.
void tilted_q(double q[4]) {
  q[0] = 2.0 / 9.0;
  q[1] = 4.0 / 9.0;
  q[2] = 5.0 / 9.0;
  q[3] = 6.0 / 9.0;
}

// Drives motor i alone to steady state under `cfg` and returns how many of its three NED torque components disagree
// with the oracle built from the spin text (kSpinTable) and the rotor geometry, not from cfg's yaw_sign:
// body torque = r x F + spin * ratio * T about +z_FRD with F = (0, 0, -T), so (-y T, x T, spin ratio T); then R q.
int violations_for_motor(const marv_plant_config& cfg, std::size_t i, const double q[4]) {
  marv_plant* p = nullptr;
  EXPECT_EQ(marv_plant_create(&cfg, &p), MARV_PLANT_OK);
  marv_plant_body body = identity_body();
  for (std::size_t j = 0; j < 4; ++j) {
    body.q_wxyz[j] = q[j];
  }
  int d[MARV_PLANT_N_MOTORS] = {0, 0, 0, 0};
  d[i] = kDshotProbe;
  const marv_plant_cmd cmd = make_cmd(static_cast<std::uint16_t>(d[0]), static_cast<std::uint16_t>(d[1]),
                                      static_cast<std::uint16_t>(d[2]), static_cast<std::uint16_t>(d[3]));
  marv_plant_out out = make_out();
  EXPECT_EQ(marv_plant_step(p, &body, &cmd, kSettleDt, &out), MARV_PLANT_OK);
  marv_plant_destroy(p);

  const double omega = esc_omega(cfg, kDshotProbe);
  const double thrust = cfg.thrust_coeff * omega * omega;
  const double x = cfg.rotor_position_frd_m[i][0];
  const double y = cfg.rotor_position_frd_m[i][1];
  const double tb[3] = {-y * thrust, x * thrust, static_cast<double>(kSpinTable[i]) * cfg.torque_ratio_m * thrust};
  double expected[3];
  rotate_to_ned(q, tb, expected);
  const double scale = thrust * (std::hypot(x, y) + cfg.torque_ratio_m);
  const double tol = settled_rel_tol(kSettleSubsteps) * scale;
  int bad = 0;
  for (std::size_t k = 0; k < 3; ++k) {
    if (!(std::fabs(out.torque_ned_nm[k] - expected[k]) <= tol)) {
      ++bad;
    }
  }
  return bad;
}

int violations_all(const marv_plant_config& cfg, const double q[4]) {
  int bad = 0;
  for (std::size_t i = 0; i < MARV_PLANT_N_MOTORS; ++i) {
    bad += violations_for_motor(cfg, i, q);
  }
  return bad;
}

TEST(PlantYawTorque, SignAndMagnitudePerMotorAtIdentityAttitude) {
  const marv_plant_config cfg = fixture_config();
  const double q[4] = {1.0, 0.0, 0.0, 0.0};
  for (std::size_t i = 0; i < MARV_PLANT_N_MOTORS; ++i) {
    EXPECT_EQ(violations_for_motor(cfg, i, q), 0) << "motor " << i + 1;
  }
  // The spin text itself: at identity attitude motors 1 and 2 (ccw) push +z_FRD, motors 3 and 4 (cw) push -z_FRD,
  // with magnitude torque_ratio * T.
  for (std::size_t i = 0; i < MARV_PLANT_N_MOTORS; ++i) {
    marv_plant* p = nullptr;
    ASSERT_EQ(marv_plant_create(&cfg, &p), MARV_PLANT_OK);
    const marv_plant_body body = identity_body();
    int d[MARV_PLANT_N_MOTORS] = {0, 0, 0, 0};
    d[i] = kDshotProbe;
    const marv_plant_cmd cmd = make_cmd(static_cast<std::uint16_t>(d[0]), static_cast<std::uint16_t>(d[1]),
                                        static_cast<std::uint16_t>(d[2]), static_cast<std::uint16_t>(d[3]));
    marv_plant_out out = make_out();
    ASSERT_EQ(marv_plant_step(p, &body, &cmd, kSettleDt, &out), MARV_PLANT_OK);
    marv_plant_destroy(p);
    const double omega = esc_omega(cfg, kDshotProbe);
    const double thrust = cfg.thrust_coeff * omega * omega;
    if (i < 2) {
      EXPECT_GT(out.torque_ned_nm[2], 0.0) << "motor " << i + 1 << " is ccw";
    } else {
      EXPECT_LT(out.torque_ned_nm[2], 0.0) << "motor " << i + 1 << " is cw";
    }
    EXPECT_NEAR(std::fabs(out.torque_ned_nm[2]), cfg.torque_ratio_m * thrust,
                settled_rel_tol(kSettleSubsteps) * thrust * (std::hypot(kArmX, kArmY) + cfg.torque_ratio_m));
  }
}

TEST(PlantYawTorque, NonTrivialAttitudeRotatesTheTorqueToNed) {
  const marv_plant_config cfg = fixture_config();
  double q[4];
  tilted_q(q);
  EXPECT_EQ(violations_all(cfg, q), 0);
}

// Negative control: swapping the spin directions of two motors (1 <-> 3) must be caught.
TEST(PlantYawTorque, NegativeControlSwappedSpinsFail) {
  marv_plant_config cfg = fixture_config();
  std::swap(cfg.yaw_sign[0], cfg.yaw_sign[2]);
  const double identity[4] = {1.0, 0.0, 0.0, 0.0};
  double tilted[4];
  tilted_q(tilted);
  // Motors 1 and 3 each disagree in the z torque at identity; in the tilted attitude z leaks into all three components.
  EXPECT_GE(violations_all(cfg, identity), 2);
  EXPECT_GE(violations_all(cfg, tilted), 2);
  EXPECT_GT(violations_for_motor(cfg, 0, identity), 0);
  EXPECT_GT(violations_for_motor(cfg, 2, identity), 0);
  EXPECT_EQ(violations_for_motor(cfg, 1, identity), 0);
  EXPECT_EQ(violations_for_motor(cfg, 3, identity), 0);
}

// consistency check (circular by construction: omega_h is derived from m, g, k). It checks the arithmetic of thrust,
// summation, rotation and gravity against each other, not the physics of any vehicle. To make omega_h reachable by
// the ESC map this fixture derives omega_max so that DShot kHoverDshot maps to omega_h exactly (up to rounding).
// Extra tolerance beyond settled_rel_tol: that derivation (sqrt, division, ~8 operations) and the ESC map evaluation
// (~6) give omega an extra relative error <= 20 eps, so T = k omega^2 gets <= 40 eps more; scale = m g.
constexpr int kHoverDshot = 1000;
constexpr double kHoverExtraOps = 40.0;

TEST(PlantHover, ConsistencyCheckCircularByConstruction) {
  const double heights_down[] = {0.0, -100.0, 40.0};
  const double cos_half = std::cos(0.35);
  const double sin_half = std::sin(0.35);
  const double yawed[4] = {cos_half, 0.0, 0.0, sin_half};
  const double level[4] = {1.0, 0.0, 0.0, 0.0};
  for (double z : heights_down) {
    marv_plant_config cfg = fixture_config();
    const double g = prim::normal_gravity<double>(cfg.site_lat_rad, cfg.site_height_m - z);
    const double omega_h = std::sqrt(cfg.mass_kg * g / (4.0 * cfg.thrust_coeff));
    cfg.omega_max_rad_s = cfg.omega_min_rad_s + (omega_h - cfg.omega_min_rad_s) *
                                                    static_cast<double>(kDshotMax - kDshotMin) /
                                                    static_cast<double>(kHoverDshot - kDshotMin);
    ASSERT_GT(cfg.omega_max_rad_s, cfg.omega_min_rad_s);
    const double weight = cfg.mass_kg * g;
    const double tol = (settled_rel_tol(kSettleSubsteps) + kHoverExtraOps * kEps) * weight;
    for (const double* q : {level, yawed}) {
      marv_plant* p = nullptr;
      ASSERT_EQ(marv_plant_create(&cfg, &p), MARV_PLANT_OK);
      marv_plant_body body = identity_body();
      body.pos_ned_m[2] = z;
      for (std::size_t j = 0; j < 4; ++j) {
        body.q_wxyz[j] = q[j];
      }
      const std::uint16_t h = static_cast<std::uint16_t>(kHoverDshot);
      const marv_plant_cmd cmd = make_cmd(h, h, h, h);
      marv_plant_out out = make_out();
      ASSERT_EQ(marv_plant_step(p, &body, &cmd, kSettleDt, &out), MARV_PLANT_OK);
      EXPECT_NEAR(out.force_ned_n[0], 0.0, tol) << "z = " << z;
      EXPECT_NEAR(out.force_ned_n[1], 0.0, tol) << "z = " << z;
      EXPECT_NEAR(out.force_ned_n[2], 0.0, tol) << "z = " << z;

      // Negative control: a command about 1 percent above hover (10 DShot steps of 1000 - 48) must leave a net
      // force far outside the tolerance, so the check above is sensitive to the thrust magnitude.
      const std::uint16_t up = static_cast<std::uint16_t>(kHoverDshot + 10);
      const marv_plant_cmd cmd_up = make_cmd(up, up, up, up);
      ASSERT_EQ(marv_plant_step(p, &body, &cmd_up, kSettleDt, &out), MARV_PLANT_OK);
      EXPECT_GT(std::fabs(out.force_ned_n[2]), 100.0 * tol) << "z = " << z;
      marv_plant_destroy(p);
    }
  }
}

}  // namespace
}  // namespace marv::plant::test
