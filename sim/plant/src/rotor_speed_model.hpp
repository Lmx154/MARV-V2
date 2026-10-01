#pragma once

// The rotor-speed sensor model of the plant (quad spec L6 stage (b); decision 0013, owner decision 7). A generic model
// of the rotor-speed class of core section 5 ("eRPM -> rotor speed"): the ESC measures the electrical period of each
// motor, sends it in the bidirectional-DShot telemetry word, and the host turns it back into a speed. Opt-in: the C
// entries are marv_plant_rotor_speed_attach and marv_plant_rotor_speed_sample; a plant that never attaches it is
// untouched, and the model never writes to the rest of the plant (it only reads the rotor speeds).
//
// One sample, per motor i, from omega_i (rad/s, the rotor state after the last marv_plant_step, the initial speed
// before any step: the freshness of the IMU's specific force):
//   1. omega_i not finite or < 0: the motor is INVALID (valid bit clear, value exactly 0). Otherwise
//   2. electrical period T_e = 2 pi / (omega_i * pole_count / 2), in period units P = T_e / period_unit_s. omega_i = 0 has
//      no finite period (P = +infinity).
//   3. encode on the telemetry grid (below): P is truncated to an integer p; m = p >> e with the smallest exponent e for
//      which m fits in mantissa_bits; the word is (e, m) and the period it carries is m << e;
//      - P >= the largest encodable value (the all-ones word's value, ((2^mantissa_bits) - 1) << (2^exponent_bits - 1)),
//        which includes omega_i = 0: STOPPED. The word is the reserved all-ones word, which the protocol reads as zero
//        eRPM. The motor is VALID and the value exactly 0 (a stopped or too-slow rotor is a measurement of zero speed,
//        not a missing one);
//      - p = 0 (P < 1, a period under one unit): the encoded period is 0, which the protocol marks INVALID. The motor
//        is invalid, value 0. (Not reachable below omega = 2 pi / (pole_count / 2 * period_unit_s) = 0.9 Mrad/s at 14
//        poles and 1 us.)
//   4. decode: omega_hat_i = 2 pi / ((m << e) * period_unit_s * pole_count / 2), rounded once to float (rad/s).
// then the delay: sample k reports the sample k - latency_ticks (sample 0's before the start, as the IMU's delay line).
//
// The grid (the citation of core section 2 rule 1; "published", pinned to a commit).
//   Betaflight, github.com/betaflight/betaflight, commit 5a09417ee75e91e81003cf7891bc1f4de86f3a83 (the latest commit to
//   touch src/main/drivers/dshot.c on master, fetched 2026-10-01), src/main/drivers/dshot.c lines 206-221:
//       static uint32_t dshot_decode_eRPM_telemetry_value(uint16_t value)
//       {
//           // eRPM range
//           if (value == 0x0fff) {
//               return 0;
//           }
//           // Convert value to 16 bit from the GCR telemetry format (eeem mmmm mmmm)
//           value = (value & 0x01ff) << ((value & 0xfe00) >> 9);
//           if (!value) {
//               return DSHOT_TELEMETRY_INVALID;
//           }
//           // Convert period to erpm * 100
//           return (1000000 * 60 / 100 + value / 2) / value;
//       }
//   and line 395: "rpm = (erpm * ERPM_PER_LSB) / (motorConfig()->motorPoleCount / 2)". So the 12-bit word is eee mmmmmmmmm
//   (3 exponent bits, 9 mantissa bits), the period in microseconds is m << e, and 60 * 10^6 / period is the electrical
//   revolutions per minute (that is why the period unit is 1 us). The word 0x0fff is zero eRPM; a decoded period of 0 is
//   invalid. Betaflight is GPL-3.0: it is cited for the format only, no code is taken (core C-4). Betaflight rounds the
//   decoded eRPM to 100-eRPM steps; this model does not (it decodes the period to a continuous speed): that rounding is
//   not modelled.
//   The grid parameters (exponent_bits, mantissa_bits, period_unit_s) are passed in the config, cited by the caller.
//
// The rounding rule of the ESC's encoder (truncate the low bits of the period) is INFERRED: nothing read here shows what
// the ESC firmware (AM32) does, and no test shows it. Truncation makes the decoded period at most one step below the
// true one: the relative error is below 2^(1 - mantissa_bits) (an exact value of the grid decodes exactly).
// The ESC's own timing error (its clock, the sampling of the commutation) is not modelled here.

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "marv/prim/constants.hpp"
#include "marv/types/actuator.hpp"

namespace marv::plant {

inline constexpr std::size_t kRotorSpeedMotors = kQuadXMotors;

// The delay-line capacity: a labelled bound (the same as the IMU's), not a sensor figure.
inline constexpr std::size_t kRotorSpeedMaxLatency = 64;

// The largest grid parameters accepted: labelled capacity bounds that keep the largest period, ((2^m) - 1) << (2^e - 1),
// below 2^31 so that it is exact in a double and in 64-bit integers. The protocol's grid is 3 and 9.
inline constexpr std::uint32_t kRotorSpeedMaxExponentBits = 4;
inline constexpr std::uint32_t kRotorSpeedMaxMantissaBits = 16;

struct RotorSpeedParams {
  std::uint32_t pole_count = 0;      // even, > 0
  std::size_t latency = 0;           // ticks, <= kRotorSpeedMaxLatency
  std::uint32_t exponent_bits = 0;   // >= 1
  std::uint32_t mantissa_bits = 0;   // >= 1
  double period_unit_s = 0.0;        // finite, > 0
};

struct RotorSpeedOut {
  std::array<float, kRotorSpeedMotors> omega{};
  std::uint32_t flags = 0;  // bit i = motor i + 1 valid
};

namespace rotor_speed_detail {

enum class Kind : std::uint8_t { Invalid, Stopped, Measured };

struct Word {
  Kind kind = Kind::Invalid;
  std::uint64_t period = 0;  // m << e, in period units; 0 unless Measured
};

inline double two_pi() { return 2.0 * prim::kPi; }

// The largest encodable value: the all-ones word, ((2^mantissa_bits) - 1) << (2^exponent_bits - 1).
inline std::uint64_t all_ones_period(const RotorSpeedParams& p) {
  const std::uint64_t mantissa_max = (std::uint64_t{1} << p.mantissa_bits) - 1;
  const std::uint64_t exponent_max = (std::uint64_t{1} << p.exponent_bits) - 1;
  return mantissa_max << exponent_max;
}

// Steps 1 to 3 above. The returned period is m << e, the period the word carries.
inline Word encode(double omega, const RotorSpeedParams& p) {
  Word w;
  if (!std::isfinite(omega) || omega < 0.0) {
    return w;
  }
  const double pole_pairs = static_cast<double>(p.pole_count / 2);
  const std::uint64_t top = all_ones_period(p);
  const double periods = omega > 0.0 ? two_pi() / (omega * pole_pairs * p.period_unit_s)
                                     : static_cast<double>(top);
  if (!(periods < static_cast<double>(top))) {
    w.kind = Kind::Stopped;
    return w;
  }
  const std::uint64_t whole = static_cast<std::uint64_t>(std::floor(periods));
  if (whole == 0) {
    return w;
  }
  std::uint32_t shift = 0;
  while ((whole >> shift) >= (std::uint64_t{1} << p.mantissa_bits)) {
    ++shift;
  }
  w.kind = Kind::Measured;
  w.period = (whole >> shift) << shift;
  return w;
}

// Step 4. A Measured word only.
inline double decode(std::uint64_t period, const RotorSpeedParams& p) {
  const double pole_pairs = static_cast<double>(p.pole_count / 2);
  return two_pi() / (static_cast<double>(period) * p.period_unit_s * pole_pairs);
}

inline constexpr std::size_t kLines = kRotorSpeedMaxLatency + 1;  // the oldest sample needed is `latency` back

}  // namespace rotor_speed_detail

class RotorSpeedModel {
 public:
  bool attached() const { return attached_; }

  void attach(const RotorSpeedParams& p) {
    p_ = p;
    index_ = 0;
    attached_ = true;
  }

  // omega: rotor speed of motors 1..4 (index = logical number - 1), rad/s.
  RotorSpeedOut sample(const std::array<double, kRotorSpeedMotors>& omega) {
    RotorSpeedOut now;
    for (std::size_t i = 0; i < kRotorSpeedMotors; ++i) {
      const rotor_speed_detail::Word w = rotor_speed_detail::encode(omega[i], p_);
      if (w.kind == rotor_speed_detail::Kind::Invalid) {
        continue;
      }
      now.flags |= std::uint32_t{1} << i;
      if (w.kind == rotor_speed_detail::Kind::Measured) {
        now.omega[i] = static_cast<float>(rotor_speed_detail::decode(w.period, p_));
      }
    }
    line_[index_ % rotor_speed_detail::kLines] = now;
    const std::uint64_t src = index_ >= p_.latency ? index_ - p_.latency : 0;
    ++index_;
    return line_[src % rotor_speed_detail::kLines];
  }

 private:
  bool attached_ = false;
  RotorSpeedParams p_;
  std::uint64_t index_ = 0;
  std::array<RotorSpeedOut, rotor_speed_detail::kLines> line_{};
};

}  // namespace marv::plant
