// Frozen path: tests/regression/quad/L01/unit/plant/wrench_test.cpp
// T1 unit tests of the marv_plant v0 wrench: yaw torque sign and magnitude per motor, the hover consistency check, and
// the rotor drag force and its share of the IMU specific force (decision 0049).
//
// Tolerances: settled_rel_tol() in plant_fixture.hpp (derivation there), times a scale that dominates every term
// summed into the checked quantity. Every rotor is driven to steady state by one step of 50 tau with a held DShot
// command, so no test hook is needed in the ABI.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
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

// Rotor drag (decision 0049). F_drag = -R D R^T v, D = diag(d) in kg/s, v the NED velocity, R body -> NED. The plant is
// compared with the same plant at D = 0 (the wrench of which the tests above already pin), so the difference is the drag
// force alone; the expected drag is restated here from the formula with the fixture's own rotation (rotate_to_ned), by
// a conjugate quaternion for R^T. D and v are scenario values (L1 test fixture), distinct on every axis so that an axis
// mix-up, a sign error or a frame error (D applied in NED) shows.
constexpr double kDragD[3] = {0.30, 0.20, 0.10};
constexpr double kDragV[3] = {3.0, -2.0, 1.5};
// Accelerometer LSB of the specific-force test: 2^-10, exactly representable, far below the drag terms (about 1 m/s^2).
constexpr double kDragImuLsb = 1.0 / 1024.0;
constexpr double kDragImuFullScale = 100.0;  // 102400 counts, inside the 20-bit word

void expected_drag_body(const double q[4], const double v[3], const double d[3], double fb[3]) {
  const double qc[4] = {q[0], -q[1], -q[2], -q[3]};
  double vb[3];
  rotate_to_ned(qc, v, vb);  // R^T v
  for (std::size_t k = 0; k < 3; ++k) {
    fb[k] = -d[k] * vb[k];
  }
}

marv_plant_config drag_config(const double d[3]) {
  marv_plant_config cfg = fixture_config();
  for (std::size_t k = 0; k < 3; ++k) {
    cfg.drag_coeff_kg_s[k] = d[k];
  }
  return cfg;
}

// One settled step at DShot kDshotProbe on all four motors, body at attitude q and velocity v; the plant is returned
// alive (for the IMU sample) with the step's output.
marv_plant* settled_plant(const marv_plant_config& cfg, const double q[4], const double v[3], marv_plant_out& out,
                          marv_plant_body& body) {
  marv_plant* p = nullptr;
  EXPECT_EQ(marv_plant_create(&cfg, &p), MARV_PLANT_OK);
  body = identity_body();
  for (std::size_t j = 0; j < 4; ++j) {
    body.q_wxyz[j] = q[j];
  }
  for (std::size_t k = 0; k < 3; ++k) {
    body.vel_ned_m_s[k] = v[k];
  }
  const std::uint16_t d = static_cast<std::uint16_t>(kDshotProbe);
  const marv_plant_cmd cmd = make_cmd(d, d, d, d);
  out = make_out();
  EXPECT_EQ(marv_plant_step(p, &body, &cmd, kSettleDt, &out), MARV_PLANT_OK);
  return p;
}

TEST(PlantDrag, KnownAnswerAtIdentityAttitude) {
  const double d_zero[3] = {0.0, 0.0, 0.0};
  const double q[4] = {1.0, 0.0, 0.0, 0.0};
  marv_plant_out with = make_out();
  marv_plant_out none = make_out();
  marv_plant_body body{};
  marv_plant_destroy(settled_plant(drag_config(kDragD), q, kDragV, with, body));
  marv_plant_destroy(settled_plant(drag_config(d_zero), q, kDragV, none, body));
  const double weight = fixture_config().mass_kg * 9.81;  // dominates every term summed into the force (g within 0.1 %)
  const double tol = settled_rel_tol(kSettleSubsteps) * 2.0 * weight;
  // F_drag = -D v by hand: (-0.30 * 3.0, -0.20 * -2.0, -0.10 * 1.5).
  const double expected[3] = {-0.9, 0.4, -0.15};
  for (std::size_t k = 0; k < 3; ++k) {
    EXPECT_NEAR(with.force_ned_n[k] - none.force_ned_n[k], expected[k], tol) << "axis " << k;
    EXPECT_EQ(with.torque_ned_nm[k], none.torque_ned_nm[k]) << "drag acts at the centre of mass: no torque";
  }
  // Negative control: the check is sensitive to the sign and to an x/y mix-up of the coefficients.
  const double swapped[3] = {-0.6, 0.3, -0.15};  // d_x and d_y exchanged
  EXPECT_GT(std::fabs(with.force_ned_n[0] - none.force_ned_n[0] - swapped[0]), 100.0 * tol);
  EXPECT_GT(std::fabs(with.force_ned_n[0] - none.force_ned_n[0] + expected[0]), 100.0 * tol);
}

TEST(PlantDrag, RotatedBodyAppliesTheCoefficientsInTheBodyFrame) {
  const double d_zero[3] = {0.0, 0.0, 0.0};
  double q[4];
  tilted_q(q);
  marv_plant_out with = make_out();
  marv_plant_out none = make_out();
  marv_plant_body body{};
  marv_plant_destroy(settled_plant(drag_config(kDragD), q, kDragV, with, body));
  marv_plant_destroy(settled_plant(drag_config(d_zero), q, kDragV, none, body));
  double fb[3];
  expected_drag_body(q, kDragV, kDragD, fb);
  double expected[3];
  rotate_to_ned(q, fb, expected);  // R (-D R^T v)
  const double weight = fixture_config().mass_kg * 9.81;
  const double tol = settled_rel_tol(kSettleSubsteps) * 2.0 * weight;
  for (std::size_t k = 0; k < 3; ++k) {
    EXPECT_NEAR(with.force_ned_n[k] - none.force_ned_n[k], expected[k], tol) << "axis " << k;
    EXPECT_EQ(with.torque_ned_nm[k], none.torque_ned_nm[k]);
  }
  // Negative control: D applied in the NED frame (-D v) is a different force at this attitude.
  double worst = 0.0;
  for (std::size_t k = 0; k < 3; ++k) {
    worst = std::max(worst, std::fabs(expected[k] + kDragD[k] * kDragV[k]));
  }
  EXPECT_GT(worst, 100.0 * tol);
}

// Opt-in: with every coefficient 0 the outputs are the same bytes whatever the velocity.
TEST(PlantDrag, ZeroCoefficientsAreByteIdenticalWhateverTheVelocity) {
  const double d_zero[3] = {0.0, 0.0, 0.0};
  const double v_zero[3] = {0.0, 0.0, 0.0};
  double q[4];
  tilted_q(q);
  marv_plant_out moving = make_out();
  marv_plant_out still = make_out();
  marv_plant_body body{};
  marv_plant_destroy(settled_plant(drag_config(d_zero), q, kDragV, moving, body));
  marv_plant_destroy(settled_plant(drag_config(d_zero), q, v_zero, still, body));
  EXPECT_EQ(std::memcmp(&moving, &still, sizeof(moving)), 0);
}

marv_plant_imu_config drag_imu_config() {
  marv_plant_imu_config c{};
  c.struct_size = sizeof(c);
  c.gyro.lsb = kDragImuLsb;
  c.gyro.full_scale = kDragImuFullScale;
  c.accel.lsb = kDragImuLsb;
  c.accel.full_scale = kDragImuFullScale;
  return c;
}

marv_plant_imu_out imu_sample_after_settle(const marv_plant_config& cfg, const double q[4], const double v[3]) {
  marv_plant_out out = make_out();
  marv_plant_body body{};
  marv_plant* p = settled_plant(cfg, q, v, out, body);
  const marv_plant_imu_config ic = drag_imu_config();
  EXPECT_EQ(marv_plant_imu_attach(p, &ic), MARV_PLANT_OK);
  marv_plant_imu_out imu{};
  EXPECT_EQ(marv_plant_imu_sample(p, &body, kSubstep, &imu), MARV_PLANT_OK);
  marv_plant_destroy(p);
  return imu;
}

// Specific force = (thrust along -z_FRD + F_drag in the body frame) / m, F_drag = -D R^T v.
TEST(PlantDrag, SpecificForceIsThrustPlusDragOverMass) {
  const double d_zero[3] = {0.0, 0.0, 0.0};
  const marv_plant_config cfg = drag_config(kDragD);
  const double omega = esc_omega(cfg, kDshotProbe);
  const double thrust_over_m = 4.0 * cfg.thrust_coeff * omega * omega / cfg.mass_kg;
  const double identity[4] = {1.0, 0.0, 0.0, 0.0};
  double tilted[4];
  tilted_q(tilted);
  // Float output rounded to the LSB: half an LSB, plus the float rounding of an output near 10 (about 5e-7), plus the
  // settled thrust tolerance on thrust / m.
  const double tol = 0.5 * kDragImuLsb + 1.0e-6 + settled_rel_tol(kSettleSubsteps) * thrust_over_m;
  const double* const attitudes[2] = {identity, tilted};
  for (const double* q : attitudes) {
    double fb[3];
    expected_drag_body(q, kDragV, kDragD, fb);
    const double expected[3] = {fb[0] / cfg.mass_kg, fb[1] / cfg.mass_kg, fb[2] / cfg.mass_kg - thrust_over_m};
    const marv_plant_imu_out with = imu_sample_after_settle(cfg, q, kDragV);
    const float got[3] = {with.accel_m_s2.x, with.accel_m_s2.y, with.accel_m_s2.z};
    for (std::size_t k = 0; k < 3; ++k) {
      EXPECT_NEAR(static_cast<double>(got[k]), expected[k], tol) << "axis " << k;
    }
    // Negative control: the drag share is above the tolerance, so the same plant without drag is outside it.
    const marv_plant_imu_out none = imu_sample_after_settle(drag_config(d_zero), q, kDragV);
    const float plain[3] = {none.accel_m_s2.x, none.accel_m_s2.y, none.accel_m_s2.z};
    double worst = 0.0;
    for (std::size_t k = 0; k < 3; ++k) {
      worst = std::max(worst, std::fabs(static_cast<double>(plain[k]) - expected[k]));
    }
    EXPECT_GT(worst, 10.0 * tol);
    // And without drag the specific force is the thrust alone, along -z_FRD, whatever the velocity.
    EXPECT_NEAR(static_cast<double>(plain[0]), 0.0, tol);
    EXPECT_NEAR(static_cast<double>(plain[1]), 0.0, tol);
    EXPECT_NEAR(static_cast<double>(plain[2]), -thrust_over_m, tol);
  }
}

}  // namespace
}  // namespace marv::plant::test
