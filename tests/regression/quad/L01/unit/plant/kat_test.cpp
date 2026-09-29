// Frozen path: tests/regression/quad/L01/unit/plant/kat_test.cpp
// KNOWN-ANSWER TEST (Luis item 6): the marv_plant v0 wrench against an independent reference. The reference is
// reference/plant_ref.py (plain Python math, thrust, r x F, yaw torque, rotation to NED, WGS 84 gravity from the
// NIMA TR8350.2 tables, sharing no code with the C++). It reads reference/plant_ref_inputs.txt (scenario values,
// L1 test fixture) and writes reference/plant_ref_expected.txt, which is committed and read here.
//
// Tolerance rule. Each case runs from motor state 0 by one step of settle_taus * tau, so the C++ rotor speed is the
// ESC-map speed to within 3 n eps relative (n sub-steps; derivation at settled_rel_tol in plant_fixture.hpp) and the
// residual exp(-50) ~ 2e-22 is below round-off. The Python reference takes omega = omega_cmd exactly, so:
//  - rotor speed:  |d| <= (3 n + 4) eps |omega_expected|  (the 4 charges the ESC-map product and sum in both codes)
//  - eRPM:         |d| <= (3 n + 8) eps |erpm_expected|   (plus the factor 60/2 pi and the pole-pair product)
//  - force, torque: |d| <= settled_rel_tol(n) * S, with S = sum T_i + m g for the force and sum (|r_i| + ratio) T_i for
//    the torque, T_i = k omega_expected_i^2. The Python and C++ libm (sin, sqrt, exp) may differ by 1 ulp each and
//    the chain order differs; both are inside the 64 eps chain term.
//
// Negative control: with the spin directions of motors 1 and 3 swapped in the plant configuration the comparison
// must fail on exactly those cases where the two rotor speeds differ (a swap of equal-thrust rotors changes nothing).

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "marv_plant.h"
#include "plant_fixture.hpp"

#ifndef MARV_PLANT_REF_DIR
#error "MARV_PLANT_REF_DIR must name the reference directory"
#endif

namespace marv::plant::test {
namespace {

struct Case {
  std::string name;
  double pos[3];
  double q[4];
  std::uint16_t dshot[MARV_PLANT_N_MOTORS];
};

struct Expected {
  double omega[4];
  double erpm[4];
  double force[3];
  double torque[3];
  double g;
};

struct Inputs {
  std::map<std::string, std::vector<double>> cfg;
  std::vector<Case> cases;
};

Inputs read_inputs() {
  Inputs in;
  std::ifstream f(std::string(MARV_PLANT_REF_DIR) + "/plant_ref_inputs.txt");
  EXPECT_TRUE(f.is_open());
  std::string line;
  while (std::getline(f, line)) {
    if (line.empty() || line[0] == '#') {
      continue;
    }
    std::istringstream ss(line);
    std::string key;
    ss >> key;
    if (key == "case") {
      Case c{};
      ss >> c.name;
      for (double& v : c.pos) {
        ss >> v;
      }
      for (double& v : c.q) {
        ss >> v;
      }
      for (std::uint16_t& d : c.dshot) {
        unsigned v = 0;
        ss >> v;
        d = static_cast<std::uint16_t>(v);
      }
      EXPECT_FALSE(ss.fail()) << line;
      in.cases.push_back(c);
    } else {
      std::vector<double> vals;
      double v = 0.0;
      while (ss >> v) {
        vals.push_back(v);
      }
      in.cfg[key] = vals;
    }
  }
  return in;
}

std::map<std::string, Expected> read_expected() {
  std::map<std::string, Expected> out;
  std::ifstream f(std::string(MARV_PLANT_REF_DIR) + "/plant_ref_expected.txt");
  EXPECT_TRUE(f.is_open());
  std::string line;
  while (std::getline(f, line)) {
    if (line.empty() || line[0] == '#') {
      continue;
    }
    std::istringstream ss(line);
    std::string name;
    ss >> name;
    Expected e{};
    for (double& v : e.omega) ss >> v;
    for (double& v : e.erpm) ss >> v;
    for (double& v : e.force) ss >> v;
    for (double& v : e.torque) ss >> v;
    ss >> e.g;
    EXPECT_FALSE(ss.fail()) << line;
    out[name] = e;
  }
  return out;
}

marv_plant_config config_from(const Inputs& in) {
  marv_plant_config c{};
  c.struct_size = sizeof(c);
  c.esc_map = MARV_PLANT_ESC_LINEAR_IN_OMEGA;
  const auto get = [&in](const char* k) -> const std::vector<double>& { return in.cfg.at(k); };
  c.mass_kg = get("mass_kg")[0];
  for (std::size_t i = 0; i < MARV_PLANT_N_MOTORS; ++i) {
    for (std::size_t j = 0; j < 3; ++j) {
      c.rotor_position_frd_m[i][j] = get("rotor_position_frd_m")[3 * i + j];
    }
    c.yaw_sign[i] = static_cast<int32_t>(get("yaw_sign")[i]);
  }
  c.thrust_coeff = get("thrust_coeff")[0];
  c.torque_ratio_m = get("torque_ratio_m")[0];
  c.omega_min_rad_s = get("omega_min_rad_s")[0];
  c.omega_max_rad_s = get("omega_max_rad_s")[0];
  c.motor_tau_s = get("motor_tau_s")[0];
  c.motor_substep_s = get("motor_substep_s")[0];
  c.pole_count = static_cast<std::uint32_t>(get("pole_count")[0]);
  c.site_lat_rad = get("site_lat_rad")[0];
  c.site_height_m = get("site_height_m")[0];
  return c;
}

// Runs every case against `cfg`; returns per-case whether all outputs are inside the tolerance rule above.
std::map<std::string, bool> run_all(const Inputs& in, const std::map<std::string, Expected>& want,
                                    const marv_plant_config& cfg, bool report) {
  std::map<std::string, bool> ok;
  const double dt = in.cfg.at("settle_taus")[0] * cfg.motor_tau_s;
  const double n = std::ceil(dt / cfg.motor_substep_s);
  const double rel = settled_rel_tol(n);
  for (const Case& c : in.cases) {
    const Expected& e = want.at(c.name);
    marv_plant* p = nullptr;
    EXPECT_EQ(marv_plant_create(&cfg, &p), MARV_PLANT_OK);
    marv_plant_body body = identity_body();
    for (std::size_t j = 0; j < 3; ++j) {
      body.pos_ned_m[j] = c.pos[j];
    }
    for (std::size_t j = 0; j < 4; ++j) {
      body.q_wxyz[j] = c.q[j];
    }
    const marv_plant_cmd cmd = make_cmd(c.dshot[0], c.dshot[1], c.dshot[2], c.dshot[3]);
    marv_plant_out out = make_out();
    EXPECT_EQ(marv_plant_step(p, &body, &cmd, dt, &out), MARV_PLANT_OK);
    marv_plant_destroy(p);

    double thrust_sum = 0.0;
    double torque_scale = 0.0;
    for (std::size_t i = 0; i < MARV_PLANT_N_MOTORS; ++i) {
      const double t = cfg.thrust_coeff * e.omega[i] * e.omega[i];
      thrust_sum += t;
      torque_scale += t * (std::hypot(cfg.rotor_position_frd_m[i][0], cfg.rotor_position_frd_m[i][1]) +
                           cfg.torque_ratio_m);
    }
    const double force_tol = rel * (thrust_sum + cfg.mass_kg * e.g);
    const double torque_tol = rel * torque_scale;
    bool good = out.erpm_valid == 1;
    const auto within = [&](double got, double want_v, double tol, const char* what, std::size_t idx) {
      const bool in_tol = std::fabs(got - want_v) <= tol;
      if (!in_tol && report) {
        ADD_FAILURE() << c.name << " " << what << "[" << idx << "]: got " << got << " expected " << want_v
                      << " tol " << tol;
      }
      good = good && in_tol;
    };
    for (std::size_t i = 0; i < MARV_PLANT_N_MOTORS; ++i) {
      within(out.rotor_speed_rad_s[i], e.omega[i], (3.0 * n + 4.0) * kEps * std::fabs(e.omega[i]), "omega", i);
      within(out.erpm[i], e.erpm[i], (3.0 * n + 8.0) * kEps * std::fabs(e.erpm[i]), "erpm", i);
    }
    for (std::size_t i = 0; i < 3; ++i) {
      within(out.force_ned_n[i], e.force[i], force_tol, "force", i);
      within(out.torque_ned_nm[i], e.torque[i], torque_tol, "torque", i);
    }
    ok[c.name] = good;
  }
  return ok;
}

TEST(PlantKnownAnswer, MatchesIndependentPythonReference) {
  const Inputs in = read_inputs();
  const std::map<std::string, Expected> want = read_expected();
  ASSERT_GE(in.cases.size(), 5U);
  ASSERT_EQ(want.size(), in.cases.size());
  const marv_plant_config cfg = config_from(in);
  for (const auto& [name, good] : run_all(in, want, cfg, true)) {
    EXPECT_TRUE(good) << name;
  }
}

TEST(PlantKnownAnswer, NegativeControlSwappedSpinsFail) {
  const Inputs in = read_inputs();
  const std::map<std::string, Expected> want = read_expected();
  marv_plant_config cfg = config_from(in);
  std::swap(cfg.yaw_sign[0], cfg.yaw_sign[2]);
  const std::map<std::string, bool> ok = run_all(in, want, cfg, false);
  std::size_t distinguishing = 0;
  for (const Case& c : in.cases) {
    const bool speeds_differ = want.at(c.name).omega[0] != want.at(c.name).omega[2];
    distinguishing += speeds_differ ? 1U : 0U;
    EXPECT_EQ(ok.at(c.name), !speeds_differ) << c.name;
  }
  EXPECT_GE(distinguishing, 3U);
}

}  // namespace
}  // namespace marv::plant::test
