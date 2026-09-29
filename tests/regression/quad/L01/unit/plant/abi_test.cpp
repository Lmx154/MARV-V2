// Frozen path: tests/regression/quad/L01/unit/plant/abi_test.cpp
// T1 tests of the marv_plant C ABI contract: every refusal, validate-before-act, and the eRPM formula.
//
// Refusal tests are exact (a status code, or bit-identical state); they need no numeric tolerance. The eRPM check
// uses erpm = omega * (60 / 2 pi) * (pole_count / 2): the plant computes it from omega with one product by the
// constant 60/2pi and one by the pole-pair count, so the tolerance is 4 eps relative (three roundings in the
// oracle's own expression plus the product/quotient pair in the plant).

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>

#include "marv_plant.h"
#include "plant_fixture.hpp"

namespace marv::plant::test {
namespace {

constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

marv_plant* create_ok(const marv_plant_config& cfg) {
  marv_plant* p = nullptr;
  EXPECT_EQ(marv_plant_create(&cfg, &p), MARV_PLANT_OK);
  EXPECT_NE(p, nullptr);
  return p;
}

marv_plant_status create_status(const marv_plant_config& cfg) {
  marv_plant* p = reinterpret_cast<marv_plant*>(1);
  const marv_plant_status s = marv_plant_create(&cfg, &p);
  if (s != MARV_PLANT_OK) {
    EXPECT_EQ(p, nullptr) << "*out must be NULL after a refused create";
  } else {
    marv_plant_destroy(p);
  }
  return s;
}

TEST(PlantAbiCreate, AcceptsTheFixture) {
  const marv_plant_config cfg = fixture_config();
  marv_plant_destroy(create_ok(cfg));
  marv_plant_destroy(nullptr);
}

TEST(PlantAbiCreate, RefusesNullAndBadStructSize) {
  marv_plant_config cfg = fixture_config();
  marv_plant* p = nullptr;
  EXPECT_EQ(marv_plant_create(nullptr, &p), MARV_PLANT_E_NULL);
  EXPECT_EQ(p, nullptr);
  EXPECT_EQ(marv_plant_create(&cfg, nullptr), MARV_PLANT_E_NULL);
  cfg.struct_size = sizeof(cfg) - 1;
  EXPECT_EQ(create_status(cfg), MARV_PLANT_E_ABI);
  cfg.struct_size = sizeof(cfg) + 1;
  EXPECT_EQ(create_status(cfg), MARV_PLANT_E_ABI);
  cfg.struct_size = 0;
  EXPECT_EQ(create_status(cfg), MARV_PLANT_E_ABI);
}

TEST(PlantAbiCreate, RefusesEveryInvalidConfigField) {
  const double bad_values[] = {kNan, kInf, -kInf, 0.0, -1.0};
  struct Field {
    const char* name;
    double marv_plant_config::*member;
    bool zero_ok;  // a value of 0 or a negative one is physical for this field
  };
  const Field scalars[] = {
      {"mass_kg", &marv_plant_config::mass_kg, false},
      {"thrust_coeff", &marv_plant_config::thrust_coeff, false},
      {"torque_ratio_m", &marv_plant_config::torque_ratio_m, false},
      {"omega_min_rad_s", &marv_plant_config::omega_min_rad_s, false},
      {"omega_max_rad_s", &marv_plant_config::omega_max_rad_s, false},
      {"motor_tau_s", &marv_plant_config::motor_tau_s, false},
      {"motor_substep_s", &marv_plant_config::motor_substep_s, false},
      {"site_lat_rad", &marv_plant_config::site_lat_rad, true},
      {"site_height_m", &marv_plant_config::site_height_m, true},
  };
  for (const Field& f : scalars) {
    for (double v : bad_values) {
      const bool physical = f.zero_ok && std::isfinite(v);
      marv_plant_config cfg = fixture_config();
      cfg.*(f.member) = v;
      EXPECT_EQ(create_status(cfg), physical ? MARV_PLANT_OK : MARV_PLANT_E_CONFIG) << f.name << " = " << v;
    }
  }
  marv_plant_config cfg = fixture_config();
  cfg.site_lat_rad = 1.6;  // beyond pi/2
  EXPECT_EQ(create_status(cfg), MARV_PLANT_E_CONFIG);
  cfg = fixture_config();
  cfg.site_lat_rad = -1.6;
  EXPECT_EQ(create_status(cfg), MARV_PLANT_E_CONFIG);

  cfg = fixture_config();
  cfg.omega_min_rad_s = cfg.omega_max_rad_s;
  EXPECT_EQ(create_status(cfg), MARV_PLANT_E_CONFIG) << "omega_min == omega_max";
  cfg.omega_min_rad_s = cfg.omega_max_rad_s + 1.0;
  EXPECT_EQ(create_status(cfg), MARV_PLANT_E_CONFIG) << "omega_min > omega_max";

  for (std::size_t i = 0; i < MARV_PLANT_N_MOTORS; ++i) {
    for (const int32_t s : {0, 2, -2}) {
      cfg = fixture_config();
      cfg.yaw_sign[i] = s;
      EXPECT_EQ(create_status(cfg), MARV_PLANT_E_CONFIG) << "yaw_sign[" << i << "] = " << s;
    }
    for (std::size_t j = 0; j < 3; ++j) {
      for (double v : {kNan, kInf}) {
        cfg = fixture_config();
        cfg.rotor_position_frd_m[i][j] = v;
        EXPECT_EQ(create_status(cfg), MARV_PLANT_E_CONFIG) << "rotor_position[" << i << "][" << j << "]";
      }
    }
  }

  cfg = fixture_config();
  cfg.esc_map = 0;
  EXPECT_EQ(create_status(cfg), MARV_PLANT_E_CONFIG);
  cfg.esc_map = MARV_PLANT_ESC_LINEAR_IN_OMEGA + 1;
  EXPECT_EQ(create_status(cfg), MARV_PLANT_E_CONFIG);

  cfg = fixture_config();
  cfg.pole_count = 7;
  EXPECT_EQ(create_status(cfg), MARV_PLANT_E_CONFIG) << "odd pole_count";
  cfg.pole_count = 0;
  EXPECT_EQ(create_status(cfg), MARV_PLANT_OK) << "pole_count 0 is 'unknown', allowed";
}

// A plant that has already moved, so "unchanged" means something.
struct Driven {
  marv_plant* p;
  marv_plant_body body = identity_body();
  marv_plant_cmd cmd = make_cmd(1200, 800, 1500, 600);
  Driven() : p(create_ok(fixture_config())) {
    marv_plant_out out = make_out();
    EXPECT_EQ(marv_plant_step(p, &body, &cmd, kSubstep * 3.0, &out), MARV_PLANT_OK);
  }
  ~Driven() { marv_plant_destroy(p); }
  Driven(const Driven&) = delete;
  Driven& operator=(const Driven&) = delete;
};

bool same_bytes(const marv_plant_out& a, const marv_plant_out& b) { return std::memcmp(&a, &b, sizeof(a)) == 0; }

// Steps a refused call, then the same valid call on it and on an untouched twin; every output must be bit-identical
// and the refused call must not have touched `out`.
void expect_refusal_leaves_state(const marv_plant_body& bbody, const marv_plant_cmd& bcmd, double bdt,
                                 marv_plant_status want, const char* what) {
  Driven a;
  Driven b;
  marv_plant_out canary = make_out();
  std::memset(&canary.force_ned_n, 0x5a, sizeof(canary.force_ned_n));
  std::memset(&canary.torque_ned_nm, 0x5a, sizeof(canary.torque_ned_nm));
  std::memset(&canary.rotor_speed_rad_s, 0x5a, sizeof(canary.rotor_speed_rad_s));
  std::memset(&canary.erpm, 0x5a, sizeof(canary.erpm));
  canary.erpm_valid = 0xdeadbeefU;
  marv_plant_out refused = canary;
  EXPECT_EQ(marv_plant_step(a.p, &bbody, &bcmd, bdt, &refused), want) << what;
  EXPECT_TRUE(same_bytes(refused, canary)) << what << ": a refused step wrote to out";

  const marv_plant_cmd next = make_cmd(300, 1900, 48, 0);
  marv_plant_out oa = make_out();
  marv_plant_out ob = make_out();
  EXPECT_EQ(marv_plant_step(a.p, &a.body, &next, kSubstep * 2.5, &oa), MARV_PLANT_OK) << what;
  EXPECT_EQ(marv_plant_step(b.p, &b.body, &next, kSubstep * 2.5, &ob), MARV_PLANT_OK) << what;
  EXPECT_TRUE(same_bytes(oa, ob)) << what << ": a refused step changed the motor state";
}

TEST(PlantAbiStep, RefusesNullPointers) {
  Driven d;
  marv_plant_out out = make_out();
  EXPECT_EQ(marv_plant_step(nullptr, &d.body, &d.cmd, kSubstep, &out), MARV_PLANT_E_NULL);
  EXPECT_EQ(marv_plant_step(d.p, nullptr, &d.cmd, kSubstep, &out), MARV_PLANT_E_NULL);
  EXPECT_EQ(marv_plant_step(d.p, &d.body, nullptr, kSubstep, &out), MARV_PLANT_E_NULL);
  EXPECT_EQ(marv_plant_step(d.p, &d.body, &d.cmd, kSubstep, nullptr), MARV_PLANT_E_NULL);
}

TEST(PlantAbiStep, RefusesBadStructSizesAndLeavesStateUnchanged) {
  Driven d;
  marv_plant_body body = d.body;
  body.struct_size += 1;
  expect_refusal_leaves_state(body, d.cmd, kSubstep, MARV_PLANT_E_ABI, "body struct_size");
  marv_plant_cmd cmd = d.cmd;
  cmd.struct_size -= 1;
  expect_refusal_leaves_state(d.body, cmd, kSubstep, MARV_PLANT_E_ABI, "cmd struct_size");
  marv_plant_out out = make_out();
  out.struct_size = 0;
  EXPECT_EQ(marv_plant_step(d.p, &d.body, &d.cmd, kSubstep, &out), MARV_PLANT_E_ABI);
  EXPECT_EQ(out.struct_size, 0U);
}

TEST(PlantAbiStep, RefusesNonFiniteBodyFields) {
  Driven d;
  for (double v : {kNan, kInf, -kInf}) {
    for (std::size_t j = 0; j < 3; ++j) {
      marv_plant_body b = d.body;
      b.pos_ned_m[j] = v;
      expect_refusal_leaves_state(b, d.cmd, kSubstep, MARV_PLANT_E_BODY, "pos");
      b = d.body;
      b.vel_ned_m_s[j] = v;
      expect_refusal_leaves_state(b, d.cmd, kSubstep, MARV_PLANT_E_BODY, "vel");
      b = d.body;
      b.omega_frd_rad_s[j] = v;
      expect_refusal_leaves_state(b, d.cmd, kSubstep, MARV_PLANT_E_BODY, "omega");
    }
    for (std::size_t j = 0; j < 4; ++j) {
      marv_plant_body b = d.body;
      b.q_wxyz[j] = v;
      expect_refusal_leaves_state(b, d.cmd, kSubstep, MARV_PLANT_E_BODY, "q");
    }
  }
}

// Quaternion tolerance: | |q|^2 - 1 | <= 4 eps_float (marv_plant.h). Just inside is accepted, well outside refused.
TEST(PlantAbiStep, QuaternionNormalizationTolerance) {
  const double eps_f = static_cast<double>(std::numeric_limits<float>::epsilon());
  Driven d;
  marv_plant_out out = make_out();
  marv_plant_body b = d.body;
  b.q_wxyz[0] = std::sqrt(1.0 + 3.0 * eps_f);  // |q|^2 - 1 = 3 eps_f, inside
  EXPECT_EQ(marv_plant_step(d.p, &b, &d.cmd, kSubstep, &out), MARV_PLANT_OK);
  b.q_wxyz[0] = std::sqrt(1.0 - 3.0 * eps_f);
  EXPECT_EQ(marv_plant_step(d.p, &b, &d.cmd, kSubstep, &out), MARV_PLANT_OK);
  for (double w : {std::sqrt(1.0 + 5.0 * eps_f), std::sqrt(1.0 - 5.0 * eps_f), 1.1, 0.9, 0.0, 2.0}) {
    b.q_wxyz[0] = w;
    expect_refusal_leaves_state(b, d.cmd, kSubstep, MARV_PLANT_E_BODY, "q norm");
  }
}

TEST(PlantAbiStep, RefusesInvalidDshotAndLeavesStateUnchanged) {
  Driven d;
  const std::uint16_t bad[] = {1, 2, 47, 2048, 4095, 65535};
  for (std::uint16_t v : bad) {
    for (std::size_t i = 0; i < MARV_PLANT_N_MOTORS; ++i) {
      marv_plant_cmd c = d.cmd;
      c.dshot[i] = v;
      expect_refusal_leaves_state(d.body, c, kSubstep, MARV_PLANT_E_CMD, "dshot");
    }
  }
  for (std::uint16_t v : {std::uint16_t{0}, std::uint16_t{48}, std::uint16_t{2047}}) {
    marv_plant_cmd c = d.cmd;
    c.dshot[0] = v;
    marv_plant_out out = make_out();
    EXPECT_EQ(marv_plant_step(d.p, &d.body, &c, kSubstep, &out), MARV_PLANT_OK) << v;
  }
}

TEST(PlantAbiStep, RefusesBadDtAndLeavesStateUnchanged) {
  Driven d;
  for (double dt : {0.0, -kSubstep, -0.0, kNan, kInf, -kInf}) {
    expect_refusal_leaves_state(d.body, d.cmd, dt, MARV_PLANT_E_DT, "dt");
  }
}

TEST(PlantAbiStep, ErpmFormulaAndValidity) {
  marv_plant_config cfg = fixture_config();
  marv_plant* p = create_ok(cfg);
  const marv_plant_body body = identity_body();
  const marv_plant_cmd cmd = make_cmd(1000, 1500, 48, 0);
  marv_plant_out out = make_out();
  ASSERT_EQ(marv_plant_step(p, &body, &cmd, kSettleDt, &out), MARV_PLANT_OK);
  marv_plant_destroy(p);
  EXPECT_EQ(out.erpm_valid, 1U);
  const double two_pi = 2.0 * std::acos(-1.0);
  for (std::size_t i = 0; i < MARV_PLANT_N_MOTORS; ++i) {
    const double expected = out.rotor_speed_rad_s[i] * (60.0 / two_pi) * (static_cast<double>(kPoles) / 2.0);
    EXPECT_NEAR(out.erpm[i], expected, 4.0 * kEps * expected) << "motor " << i + 1;
  }
  EXPECT_EQ(out.erpm[3], 0.0);
  EXPECT_GT(out.erpm[0], 0.0);

  cfg.pole_count = 0;
  p = create_ok(cfg);
  out = make_out();
  ASSERT_EQ(marv_plant_step(p, &body, &cmd, kSettleDt, &out), MARV_PLANT_OK);
  marv_plant_destroy(p);
  EXPECT_EQ(out.erpm_valid, 0U);
  for (std::size_t i = 0; i < MARV_PLANT_N_MOTORS; ++i) {
    EXPECT_EQ(out.erpm[i], 0.0);
  }
  EXPECT_GT(out.rotor_speed_rad_s[0], 0.0);
}

TEST(PlantAbiStatus, EveryStatusHasAName) {
  for (marv_plant_status s = MARV_PLANT_OK; s <= MARV_PLANT_E_ALLOC; ++s) {
    EXPECT_STRNE(marv_plant_status_str(s), "unknown status");
  }
  EXPECT_STREQ(marv_plant_status_str(-1), "unknown status");
}

}  // namespace
}  // namespace marv::plant::test
