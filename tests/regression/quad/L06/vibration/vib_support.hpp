// Shared helpers of the L6 vibration tests. Every number is a labelled test value with its reason, or derived on the line.
#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "../imu/support.hpp"
#include "noise/musl_math.h"
#include "noise/noise.hpp"
#include "plant_model.hpp"

namespace marv::plant::vib_test {

using namespace marv::plant::imu_test;
namespace noise = marv::plant::noise;

// The stream id of the vibration phases is ABI (stream_ids.hpp): a renumbering is a decision, so a literal is pinned here.
constexpr std::uint16_t kVibrationStream = 1;
static_assert(noise::kVibration == kVibrationStream);
static_assert(noise::kStreamPrimaryImu == 0);

// The double nearest 2 pi.
constexpr double kTwoPi = 0x1.921fb54442d18p+2;

// Gyro quantisation step of these tests: 2^-10 rad/s (binary exact, 0.001 rad/s: fine against amplitudes of ~1 rad/s,
// and 100 / 2^-10 = 102400 counts < 2^19, so the full scale binds, not the word).
constexpr double kVibLsb = 0x1p-10;

// A zero-noise IMU config: N = B = 0, turn-on 0, latency 0, the vibration LSB, full scale 100 rad/s (kFullScale).
inline marv_plant_imu_config clean_imu() {
  marv_plant_imu_config c = zero_noise_config(0);
  c.gyro.lsb = kVibLsb;
  return c;
}

// A noisy config: N and B are positive labelled test values (rad/s/sqrt(Hz), rad/s), enough that the noise changes
// counts at this LSB; latency 1 like the profile.
constexpr double kTestNoiseDensity = 1.0e-2;
constexpr double kTestBiasInstability = 1.0e-3;
inline marv_plant_imu_config noisy_imu() {
  marv_plant_imu_config c = clean_imu();
  c.latency_samples = 1;
  c.gyro.noise_density = kTestNoiseDensity;
  c.gyro.bias_instability = kTestBiasInstability;
  c.accel.noise_density = kTestNoiseDensity;
  c.accel.bias_instability = kTestBiasInstability;
  return c;
}

// omega_hover of the fixture quad: 4 k w^2 = m g, with g = 9.80665 m/s^2 (standard gravity; any positive value serves,
// it only scales the ratio).
inline double hover_omega() { return std::sqrt(kMass * 9.80665 / (4.0 * kThrustCoeff)); }

inline marv_plant_vibration_config vib_config(std::array<double, 3> amp, double exponent = 2.0) {
  marv_plant_vibration_config c{};
  c.struct_size = sizeof(c);
  for (std::size_t h = 0; h < 3; ++h) {
    c.amplitude_rad_s[h] = amp[h];
  }
  c.omega_hover_rad_s = hover_omega();
  c.speed_exponent = exponent;
  return c;
}

// The phase phi_{motor, harmonic, axis} recomputed from the stream (documented order, vibration_model.hpp), all indices
// zero-based. Recomputed rather than read back: it needs no hook in the ABI, and pins the documented draw order and the
// stream id independently of VibrationModel.
inline double phase(std::uint64_t seed, std::size_t motor, std::size_t harmonic, std::size_t axis) {
  const std::uint64_t n = (3 * motor + harmonic) * 3 + axis;
  return kTwoPi * noise::uniform_closed_open(noise::draw(seed, kVibrationStream, n));
}

// The vibration value of the documented formula, restated (same sum order: axis, motor, harmonic; same expression
// order, so the double is bitwise the model's), from rotor speeds and angles (from a twin Model).
inline std::array<double, 3> expected_value(std::uint64_t seed, const std::array<double, 4>& omega,
                                            const std::array<double, 4>& theta, std::array<double, 3> amp, double hover,
                                            int exponent) {
  std::array<double, 3> v{};
  for (std::size_t a = 0; a < 3; ++a) {
    double sum = 0.0;
    for (std::size_t i = 0; i < 4; ++i) {
      double r = 1.0;
      for (int k = 0; k < exponent; ++k) {
        r *= omega[i] / hover;
      }
      for (std::size_t h = 0; h < 3; ++h) {
        if (amp[h] == 0.0) {
          continue;
        }
        sum += amp[h] * r * marv_musl_sin(static_cast<double>(h + 1) * theta[i] + phase(seed, i, h, a));
      }
    }
    v[a] = sum;
  }
  return v;
}

// The same value from independent libm calls (std::sin, std::pow, long double sum): used only to show the formula is the
// documented one, within a derived tolerance.
inline std::array<long double, 3> reference_value(std::uint64_t seed, const std::array<double, 4>& omega,
                                                  const std::array<double, 4>& theta, std::array<double, 3> amp,
                                                  double hover, double exponent) {
  std::array<long double, 3> v{};
  for (std::size_t a = 0; a < 3; ++a) {
    for (std::size_t i = 0; i < 4; ++i) {
      for (std::size_t h = 0; h < 3; ++h) {
        v[a] += static_cast<long double>(amp[h]) *
                std::pow(static_cast<long double>(omega[i]) / hover, static_cast<long double>(exponent)) *
                std::sin(static_cast<long double>(h + 1) * theta[i] + phase(seed, i, h, a));
      }
    }
  }
  return v;
}

// The quantiser of the IMU, restated for these tests: round half away from zero of y / LSB, clamped to +-floor(FS / LSB)
// (the full scale binds at this LSB, support.hpp), times the LSB, as a float; sat true iff clamped.
inline float quantise(double y, bool& sat) {
  const double span = std::floor(kFullScale / kVibLsb);
  double c = std::round(y / kVibLsb);
  sat = c > span || c < -span;
  c = c > span ? span : (c < -span ? -span : c);
  return static_cast<float>(c * kVibLsb);
}

}  // namespace marv::plant::vib_test
