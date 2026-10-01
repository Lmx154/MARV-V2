#pragma once

// The gyro vibration model of the plant (quad spec L6 stage (b); decisions 0013 owner decisions 1 and 2). Motion at the
// IMU, gyro only, added to the truth rate before the sensor model (delay, noise, quantiser, saturation). Opt-in: the C
// entry is marv_plant_vibration_attach; detached, or with every amplitude exactly 0, the plant adds nothing at all.
//
// Value on gyro axis a (a = x, y, z, FRD; all three axes share the amplitudes, INFERRED: nothing sources a per-axis
// split), from the rotor states after the last step:
//     v_a = sum over motors i = 1..4, harmonics h = 1..3 of  A_h (omega_i / omega_hover)^p sin(h theta_i + phi_{i,h,a})
// A_h is the reference amplitude at omega_hover, rad/s. p is the speed exponent, an integer, evaluated by repeated
// multiplication (no pow call). theta_i is the rotor's mechanical angle (plant_model.hpp), in [0, 2 pi).
// The sum runs motors outermost, then harmonics, in index order, so its rounding is fixed. sin is the vendored musl
// sin, never the platform's; the arithmetic is compiled without contraction (marv_apply_flags), so the value is the
// same on every host.
//
// Phases. 36 numbers phi_{i,h,a}, drawn once at attach from stream kVibration of the plant's seed: draw number
//     n = (3 (i - 1) + (h - 1)) * 3 + a            (i = 1..4, h = 1..3, a = 0 x, 1 y, 2 z)
// gives phi = 2 pi * uniform_closed_open(draw(seed, kVibration, n)), in [0, 2 pi).

#include <array>
#include <cstddef>
#include <cstdint>

#include "marv/prim/constants.hpp"
#include "marv/types/actuator.hpp"
#include "noise/musl_math.h"
#include "noise/noise.hpp"

namespace marv::plant {

inline constexpr std::size_t kVibrationHarmonics = 3;

// The largest speed exponent accepted: a labelled capacity bound (the owner's value is 2, a middle of 1 to 3), not a
// physical figure.
inline constexpr std::uint32_t kVibrationMaxExponent = 8;

struct VibrationParams {
  std::array<double, kVibrationHarmonics> amplitude{};  // A_h, rad/s at omega_hover
  double omega_hover = 1.0;                             // rad/s, > 0
  std::uint32_t exponent = 0;                           // p
};

class VibrationModel {
 public:
  bool attached() const { return attached_; }
  // True iff some A_h is not exactly 0; only then does the model change the gyro truth.
  bool active() const { return active_; }

  void attach(const VibrationParams& p, std::uint64_t seed) {
    p_ = p;
    constexpr double kTwoPi = 2.0 * prim::kPi;
    std::uint64_t n = 0;
    for (std::size_t i = 0; i < kQuadXMotors; ++i) {
      for (std::size_t h = 0; h < kVibrationHarmonics; ++h) {
        for (std::size_t a = 0; a < 3; ++a) {
          phase_[i][h][a] = kTwoPi * noise::uniform_closed_open(noise::draw(seed, noise::kVibration, n));
          ++n;
        }
      }
    }
    active_ = false;
    for (double amp : p.amplitude) {
      active_ = active_ || amp != 0.0;
    }
    attached_ = true;
  }

  double phase(std::size_t motor, std::size_t harmonic_index, std::size_t axis) const {
    return phase_[motor][harmonic_index][axis];
  }

  std::array<double, 3> value(const std::array<double, kQuadXMotors>& omega,
                              const std::array<double, kQuadXMotors>& theta) const {
    std::array<double, kQuadXMotors> scale{};
    for (std::size_t i = 0; i < kQuadXMotors; ++i) {
      const double ratio = omega[i] / p_.omega_hover;
      double r = 1.0;
      for (std::uint32_t k = 0; k < p_.exponent; ++k) {
        r *= ratio;
      }
      scale[i] = r;
    }
    std::array<double, 3> v{};
    for (std::size_t a = 0; a < 3; ++a) {
      double sum = 0.0;
      for (std::size_t i = 0; i < kQuadXMotors; ++i) {
        for (std::size_t h = 0; h < kVibrationHarmonics; ++h) {
          if (p_.amplitude[h] == 0.0) {
            continue;  // a zero amplitude contributes nothing, even where the speed ratio overflows (0 * inf)
          }
          const double arg = static_cast<double>(h + 1) * theta[i] + phase_[i][h][a];
          sum += p_.amplitude[h] * scale[i] * marv_musl_sin(arg);
        }
      }
      v[a] = sum;
    }
    return v;
  }

 private:
  bool attached_ = false;
  bool active_ = false;
  VibrationParams p_;
  std::array<std::array<std::array<double, 3>, kVibrationHarmonics>, kQuadXMotors> phase_{};
};

}  // namespace marv::plant
