// Shared helpers of the L6 IMU tests. Every number is a labelled test value with its reason, or derived on the line.
#pragma once

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "marv_plant.h"
#include "plant_fixture.hpp"

namespace marv::plant::imu_test {

using namespace marv::plant::test;

// Binary-exact quantisation steps, so ties and counts are exact in double: 2^-3 rad/s and 2^-4 m/s^2.
constexpr double kGyroLsb = 0.125;
constexpr double kAccelLsb = 0.0625;
// Full scale 100 (rad/s or m/s^2): 800 gyro counts and 1600 accel counts, well inside the 20-bit word, so the full scale binds.
constexpr double kFullScale = 100.0;
// A full scale far above the word at both LSBs (2^19 * 0.125 = 65536), so the word binds.
constexpr double kHugeFullScale = 1.0e6;
// Sample interval of the tests (any positive value; the zero-noise results do not depend on it).
constexpr double kDt = 0.01;
// Largest delay line accepted (the header's labelled bound).
constexpr std::uint32_t kMaxLatency = MARV_PLANT_IMU_MAX_LATENCY_SAMPLES;

inline marv_plant_imu_config zero_noise_config(std::uint32_t latency = 0) {
  marv_plant_imu_config c{};
  c.struct_size = sizeof(c);
  c.latency_samples = latency;
  c.gyro.lsb = kGyroLsb;
  c.gyro.full_scale = kFullScale;
  c.accel.lsb = kAccelLsb;
  c.accel.full_scale = kFullScale;
  return c;
}

// A plant of the L1 fixture with every rotor at `omega0` rad/s, `seed` as its rng_seed.
class PlantHandle {
 public:
  explicit PlantHandle(std::uint64_t seed = 0, double omega0 = 0.0) {
    marv_plant_config c = fixture_config();
    c.rng_seed = seed;
    for (double& w : c.initial_omega_rad_s) {
      w = omega0;
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

inline marv_plant_body body_with_omega(double x, double y, double z) {
  marv_plant_body b = identity_body();
  b.omega_frd_rad_s[0] = x;
  b.omega_frd_rad_s[1] = y;
  b.omega_frd_rad_s[2] = z;
  return b;
}

inline marv_plant_imu_out sample_ok(marv_plant* p, const marv_plant_body& b, double dt = kDt) {
  marv_plant_imu_out o{};
  EXPECT_EQ(marv_plant_imu_sample(p, &b, dt, &o), MARV_PLANT_OK);
  return o;
}

constexpr std::uint32_t bit(int n) { return std::uint32_t{1} << n; }
constexpr std::uint32_t kValidBits = bit(MARV_PLANT_IMU_GYRO_VALID) | bit(MARV_PLANT_IMU_ACCEL_VALID);

inline bool same_bits(const marv_plant_imu_out& a, const marv_plant_imu_out& b) {
  return std::memcmp(&a, &b, sizeof(a)) == 0;
}
inline bool same_bits(const marv_plant_out& a, const marv_plant_out& b) { return std::memcmp(&a, &b, sizeof(a)) == 0; }

// One sample of a fresh plant (rotors at rest, seed 0) with the given IMU config and body rate.
inline marv_plant_imu_out one_sample(const marv_plant_imu_config& c, const marv_plant_body& b) {
  PlantHandle plant;
  EXPECT_EQ(marv_plant_imu_attach(plant.get(), &c), MARV_PLANT_OK);
  return sample_ok(plant.get(), b);
}

}  // namespace marv::plant::imu_test
