// T1 unit test of marv_plant_config::initial_omega_rad_s (decision 0007, the additive L2 interface change).
//
// Oracle for "honoured". One sub-step from omega0 under a held command c is omega = c + (omega0 - c) exp(-h / tau)
// (the closed form of motor_test.cpp, evaluated directly). The plant does one exp, one subtraction, one product and one
// sum: <= 3 eps scale; the oracle adds 2.5 eps scale (motor_test.cpp's bound); tol = 6 eps scale, scale =
// max(|c|, |omega0|).
//
// "Zero equals today's behaviour" is exact: a default-constructed config (the field never touched, value-initialised to
// 0) and one with the field set to 0.0 explicitly give bit-identical outputs over a run. The frozen L1 plant
// reference (plant_ref) pins the default against the pre-change plant.
//
// Negative controls: (a) the closed form from omega0 = 0 must fall outside tol for a nonzero omega0; (b) a nonzero
// omega0 must change the run's bits against the default; (c) omega_max + 1 ulp must be refused where omega_max is
// accepted, so the bound is exactly the one stated.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <vector>

#include "marv_plant.h"
#include "plant_fixture.hpp"

namespace marv::plant::test {
namespace {

constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr std::uint16_t kCmd[MARV_PLANT_N_MOTORS] = {1000, 2047, 48, 1500};
constexpr double kW0[MARV_PLANT_N_MOTORS] = {1234.5, 3800.0, 0.0, 250.0};  // scenario values; in [0, kOmegaMax]

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

marv_plant_config with_w0(const double w[MARV_PLANT_N_MOTORS]) {
  marv_plant_config cfg = fixture_config();
  for (std::size_t i = 0; i < MARV_PLANT_N_MOTORS; ++i) {
    cfg.initial_omega_rad_s[i] = w[i];
  }
  return cfg;
}

std::vector<marv_plant_out> run(const marv_plant_config& cfg, std::size_t steps) {
  marv_plant* p = nullptr;
  EXPECT_EQ(marv_plant_create(&cfg, &p), MARV_PLANT_OK);
  const marv_plant_body body = identity_body();
  const marv_plant_cmd cmd = make_cmd(kCmd[0], kCmd[1], kCmd[2], kCmd[3]);
  std::vector<marv_plant_out> outs;
  for (std::size_t n = 0; n < steps; ++n) {
    marv_plant_out out = make_out();
    EXPECT_EQ(marv_plant_step(p, &body, &cmd, cfg.motor_substep_s, &out), MARV_PLANT_OK);
    outs.push_back(out);
  }
  marv_plant_destroy(p);
  return outs;
}

double closed_form(double c, double w0) { return c + (w0 - c) * std::exp(-kSubstep / kTau); }

double worst_ratio(const marv_plant_config& cfg, const double oracle_w0[MARV_PLANT_N_MOTORS]) {
  const marv_plant_out out = run(cfg, 1)[0];
  double worst = 0.0;
  for (std::size_t i = 0; i < MARV_PLANT_N_MOTORS; ++i) {
    const double c = esc_omega(cfg, kCmd[i]);
    const double tol = 6.0 * kEps * std::max(std::fabs(c), std::fabs(cfg.initial_omega_rad_s[i]));
    worst = std::max(worst, std::fabs(out.rotor_speed_rad_s[i] - closed_form(c, oracle_w0[i])) / tol);
  }
  return worst;
}

TEST(PlantInitialRate, FirstSubstepStartsFromTheConfiguredSpeed) {
  const marv_plant_config cfg = with_w0(kW0);
  EXPECT_LE(worst_ratio(cfg, kW0), 1.0);
}

TEST(PlantInitialRate, ControlTheClosedFormFromRestFallsOutsideTheTolerance) {
  const marv_plant_config cfg = with_w0(kW0);
  const double rest[MARV_PLANT_N_MOTORS] = {};
  EXPECT_GT(worst_ratio(cfg, rest), 1.0);
}

TEST(PlantInitialRate, DefaultConfigStartsAtRest) {
  const marv_plant_config cfg = fixture_config();
  for (double w : cfg.initial_omega_rad_s) {
    EXPECT_EQ(w, 0.0);
  }
  const double rest[MARV_PLANT_N_MOTORS] = {};
  EXPECT_LE(worst_ratio(cfg, rest), 1.0);
}

bool same_bits(const std::vector<marv_plant_out>& a, const std::vector<marv_plant_out>& b) {
  return a.size() == b.size() && std::memcmp(a.data(), b.data(), a.size() * sizeof(marv_plant_out)) == 0;
}

constexpr std::size_t kRunSteps = 40;

TEST(PlantInitialRate, ExplicitZeroIsBitIdenticalToTheDefaultConfig) {
  const double rest[MARV_PLANT_N_MOTORS] = {};
  EXPECT_TRUE(same_bits(run(fixture_config(), kRunSteps), run(with_w0(rest), kRunSteps)));
}

TEST(PlantInitialRate, ControlANonzeroSpeedChangesTheRun) {
  EXPECT_FALSE(same_bits(run(fixture_config(), kRunSteps), run(with_w0(kW0), kRunSteps)));
}

TEST(PlantInitialRate, RefusesOutOfRangeAndNonFiniteEntries) {
  const double bad[] = {-1.0, -kEps, kOmegaMax + 1.0, kNan, kInf, -kInf};
  for (std::size_t i = 0; i < MARV_PLANT_N_MOTORS; ++i) {
    for (double v : bad) {
      marv_plant_config cfg = fixture_config();
      cfg.initial_omega_rad_s[i] = v;
      EXPECT_EQ(create_status(cfg), MARV_PLANT_E_CONFIG) << "motor " << i + 1 << " value " << v;
    }
  }
}

TEST(PlantInitialRate, AcceptsTheClosedRangeAndControlTheBoundIsExact) {
  for (std::size_t i = 0; i < MARV_PLANT_N_MOTORS; ++i) {
    for (double v : {0.0, -0.0, kOmegaMax}) {
      marv_plant_config cfg = fixture_config();
      cfg.initial_omega_rad_s[i] = v;
      EXPECT_EQ(create_status(cfg), MARV_PLANT_OK) << "motor " << i + 1 << " value " << v;
    }
    marv_plant_config cfg = fixture_config();
    cfg.initial_omega_rad_s[i] = std::nextafter(kOmegaMax, kInf);
    EXPECT_EQ(create_status(cfg), MARV_PLANT_E_CONFIG) << "motor " << i + 1;
  }
}

TEST(PlantInitialRate, AConfigOfTheOlderSizeIsAnAbiMismatch) {
  marv_plant_config cfg = fixture_config();
  cfg.struct_size = static_cast<std::uint32_t>(offsetof(marv_plant_config, initial_omega_rad_s));
  EXPECT_EQ(create_status(cfg), MARV_PLANT_E_ABI);
}

}  // namespace
}  // namespace marv::plant::test
