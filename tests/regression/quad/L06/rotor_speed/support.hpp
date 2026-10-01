// Shared helpers of the L6 stage (b) rotor-speed tests (decision 0013, owner decision 7). Every number is a labelled test
// value with its reason, a cited constant, or derived on the line. The oracle below is independent of the model: it
// works from the period as a double and finds the exponent by division by powers of two, where the model works on
// integers with shifts.
#pragma once

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

#include "marv_plant.h"
#include "plant_fixture.hpp"

namespace marv::plant::rotor_test {

using namespace marv::plant::test;

// The double nearest 2 pi.
constexpr double kTwoPi = 0x1.921fb54442d18p+2;

// The telemetry grid of Betaflight src/main/drivers/dshot.c at commit 5a09417ee75e91e81003cf7891bc1f4de86f3a83, lines
// 206-221 (quoted in sim/plant/src/rotor_speed_model.hpp): the word is eee mmmmmmmmm, the period in us is m << e.
constexpr std::uint32_t kExponentBits = 3;
constexpr std::uint32_t kMantissaBits = 9;
constexpr double kPeriodUnitS = 1.0e-6;
// The profile's pole count (sensors/profiles/marv_v2_board_default.yaml, rotor_speed.pole_count: 14).
constexpr std::uint32_t kPoleCount = 14;
// 2^kMantissaBits and the largest exponent 2^kExponentBits - 1, restated so the oracle does not read the model's.
constexpr double kMantissaLimit = 512.0;
constexpr int kExponentMax = 7;
// The all-ones word's period, (2^9 - 1) << 7 = 65408 us: the smallest period that reads as stopped.
constexpr double kStoppedPeriod = 511.0 * 128.0;

inline marv_plant_rotor_speed_config grid_config(std::uint32_t latency = 0, std::uint32_t poles = kPoleCount) {
  marv_plant_rotor_speed_config c{};
  c.struct_size = sizeof(c);
  c.pole_count = poles;
  c.latency_ticks = latency;
  c.exponent_bits = kExponentBits;
  c.mantissa_bits = kMantissaBits;
  c.period_unit_s = kPeriodUnitS;
  return c;
}

// Electrical period in us of a rotor at omega rad/s, pole_count poles: 2 pi / (omega * pole_count / 2) s.
inline double period_us(double omega, std::uint32_t poles) {
  return kTwoPi / (omega * static_cast<double>(poles / 2)) / kPeriodUnitS;
}
// The speed that has electrical period `us`.
inline double omega_of_period_us(double us, std::uint32_t poles) {
  return kTwoPi / (us * kPeriodUnitS * static_cast<double>(poles / 2));
}

struct Reading {
  bool valid = false;
  float omega = 0.0F;
};

enum class Rule { Truncate, Nearest };

// The expected reading of one motor: truncate the period to the grid (or, for a control, take the nearest grid point)
// and decode. The stopped and invalid rules are those of rotor_speed_model.hpp.
inline Reading oracle(double omega, std::uint32_t poles, Rule rule = Rule::Truncate) {
  if (!std::isfinite(omega) || omega < 0.0) {
    return {false, 0.0F};
  }
  const double p = omega > 0.0 ? period_us(omega, poles) : kStoppedPeriod;
  if (!(p < kStoppedPeriod)) {
    return {true, 0.0F};
  }
  if (std::floor(p) < 1.0) {
    return {false, 0.0F};
  }
  for (int e = 0; e <= kExponentMax; ++e) {
    const double step = std::ldexp(1.0, e);
    const double m = std::floor(rule == Rule::Truncate ? p / step : p / step + 0.5);
    if (m < kMantissaLimit) {
      return {true, static_cast<float>(omega_of_period_us(m * step, poles))};
    }
  }
  return {true, 0.0F};
}

// A plant of the L1 fixture at rest except for the per-motor initial speeds (rad/s), the config's pole count.
class PlantHandle {
 public:
  explicit PlantHandle(const std::array<double, MARV_PLANT_N_MOTORS>& omega0 = {}, std::uint64_t seed = 0) {
    marv_plant_config c = fixture_config();
    c.rng_seed = seed;
    for (std::size_t i = 0; i < MARV_PLANT_N_MOTORS; ++i) {
      c.initial_omega_rad_s[i] = omega0[i];
    }
    EXPECT_EQ(marv_plant_create(&c, &p_), MARV_PLANT_OK);
  }
  ~PlantHandle() { marv_plant_destroy(p_); }
  PlantHandle(const PlantHandle&) = delete;
  PlantHandle& operator=(const PlantHandle&) = delete;
  marv_plant* get() const { return p_; }

 private:
  marv_plant* p_ = nullptr;
};

inline marv_plant_rotor_speed_out sample_ok(marv_plant* p) {
  marv_plant_rotor_speed_out o{};
  EXPECT_EQ(marv_plant_rotor_speed_sample(p, &o), MARV_PLANT_OK);
  return o;
}

constexpr std::uint32_t all_valid() { return (std::uint32_t{1} << MARV_PLANT_N_MOTORS) - 1; }

inline bool same_bits(const marv_plant_rotor_speed_out& a, const marv_plant_rotor_speed_out& b) {
  return std::memcmp(&a, &b, sizeof(a)) == 0;
}
inline bool same_bits(const marv_plant_out& a, const marv_plant_out& b) { return std::memcmp(&a, &b, sizeof(a)) == 0; }

inline bool matches(const marv_plant_rotor_speed_out& o, const std::array<Reading, MARV_PLANT_N_MOTORS>& want) {
  for (std::size_t i = 0; i < MARV_PLANT_N_MOTORS; ++i) {
    const bool valid = ((o.flags >> i) & 1U) != 0;
    if (valid != want[i].valid || std::memcmp(&o.omega_rad_s[i], &want[i].omega, sizeof(float)) != 0) {
      return false;
    }
  }
  return (o.flags & ~all_valid()) == 0;
}

}  // namespace marv::plant::rotor_test
