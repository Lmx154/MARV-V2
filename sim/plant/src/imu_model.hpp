#pragma once

// The generic IMU model of the plant (quad spec L6 stage (a); decision 0012). Pure model, no C ABI: the C entries are
// in marv_plant.cpp. All arithmetic is double; the output is rounded to float once, at the end.
//
// One sample, in this order (the order is normative):
//   1. delay: sample k measures the truth of sample k - L; before sample 0 the truth of sample 0 is used;
//   2. the sample's 12 normals, always drawn, whatever the parameters (noise.hpp: gyro RW xyz, gyro white xyz, accel RW
//      xyz, accel white xyz);
//   3. bias: b_k = b_{k-1} + K sqrt(dt) w_k, b_{-1} = the turn-on bias;
//   4. y = truth_{k-L} + b_k + sigma_d v_k;
//   5. count = round-half-away-from-zero(y / LSB), clamped to the tighter of the 20-bit word and the full scale;
//      the saturation flag of a component is set iff its count was clamped;
//   6. out = (float)(count * LSB); gyro and accel valid; temperature not provided (temp_k 0, TempValid clear).
//
// Bias random walk. Per axis, with N the one-sided noise density (unit / sqrt(Hz)) and B the bias instability (the
// minimum of the Allan deviation, unit). The white noise has Allan variance N_A^2 / tau with N_A = N / sqrt(2) (the
// one-sided density N is the two-sided N / sqrt(2) folded: sigma_d^2 dt = N^2 / 2, below). The rate random walk
// b += K sqrt(dt) w has Allan variance K^2 tau / 3. The sum N_A^2 / tau + K^2 tau / 3 has its minimum where
// tau* = sqrt(3) N_A / K, of value 2 N_A K / sqrt(3). Setting that to B^2 gives
//     K = sqrt(3) B^2 / (2 N_A) = (sqrt(6) / 2) B^2 / N,         tau* = (N / B)^2   (check: N_A^2 / tau* = N_A K / sqrt(3)).
// B = 0 gives K = 0 (no bias drift). N = 0 gives K = 0 and sigma_d = 0 (the rule for the undefined B^2 / N: no noise
// model at all; the turn-on bias remains).
//
// White noise. sigma_d = N / sqrt(2 dt): the one-sided density N over the Nyquist bandwidth 1 / (2 dt) gives the
// variance N^2 / (2 dt). The total RMS in a bandwidth f is then N sqrt(f) (DS-000577 rev 1.0 Table 1 note 5, "Calculated
// from Rate Noise Spectral Density": 0.0038 deg/s/sqrt(Hz) * sqrt(100 Hz) = 0.038 deg/s-rms).

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "marv_plant.h"
#include "noise/noise.hpp"

namespace marv::plant {

inline constexpr std::size_t kImuMaxLatency = MARV_PLANT_IMU_MAX_LATENCY_SAMPLES;

// The output data word is 20 bits, two's complement (architect's design of stage (a)): counts in [-2^19, 2^19 - 1].
inline constexpr int kImuWordBits = 20;
inline constexpr double kImuWordMax = static_cast<double>((std::int64_t{1} << (kImuWordBits - 1)) - 1);
inline constexpr double kImuWordMin = -static_cast<double>(std::int64_t{1} << (kImuWordBits - 1));

struct ImuAxisParams {
  double density = 0.0;   // N, unit / sqrt(Hz)
  double instability = 0.0;  // B, unit
  double lsb = 1.0;       // unit / count
  double full_scale = 1.0;   // unit, +-
  std::array<double, 3> turn_on{};
};

struct ImuParams {
  ImuAxisParams gyro;
  ImuAxisParams accel;
  std::size_t latency = 0;
};

struct ImuOut {
  std::array<float, 3> gyro{};
  std::array<float, 3> accel{};
  std::uint32_t flags = 0;
};

namespace imu_detail {

inline constexpr std::size_t kLines = kImuMaxLatency + 1;  // delay line: the oldest needed truth is L <= max back

// (sqrt(6) / 2): derived above.
inline double rw_factor() { return std::sqrt(6.0) * 0.5; }

inline double rw_gain(const ImuAxisParams& p) {
  return (p.density > 0.0 && p.instability > 0.0) ? rw_factor() * p.instability * p.instability / p.density : 0.0;
}

// Counts of the tighter of the word and the full scale. Returns the count (an integer value held in a double) and
// sets sat iff it was clamped. A non-finite y/LSB clamps (NaN to the low end), so the output is always finite.
inline float quantise(double y, const ImuAxisParams& p, bool& sat) {
  using std::floor;
  using std::round;
  const double span = floor(p.full_scale / p.lsb);
  const double hi = span < kImuWordMax ? span : kImuWordMax;
  const double lo = -span > kImuWordMin ? -span : kImuWordMin;
  double c = round(y / p.lsb);
  sat = false;
  if (!(c >= lo)) {
    c = lo;
    sat = true;
  } else if (!(c <= hi)) {
    c = hi;
    sat = true;
  }
  const double count = static_cast<double>(static_cast<std::int64_t>(c));  // integer count; -0 becomes +0
  return static_cast<float>(count * p.lsb);
}

}  // namespace imu_detail

class ImuModel {
 public:
  bool attached() const { return attached_; }
  std::uint64_t samples_taken() const { return index_; }

  void attach(const ImuParams& p, std::uint64_t seed) {
    p_ = p;
    seed_ = seed;
    gain_gyro_ = imu_detail::rw_gain(p.gyro);
    gain_accel_ = imu_detail::rw_gain(p.accel);
    for (std::size_t i = 0; i < 3; ++i) {
      bias_gyro_[i] = p.gyro.turn_on[i];
      bias_accel_[i] = p.accel.turn_on[i];
    }
    index_ = 0;
    attached_ = true;
  }

  // truth: gyro xyz then specific force xyz (FRD), the plant's truth of this sample. dt > 0, finite.
  ImuOut sample(const std::array<double, 6>& truth, double dt) {
    using std::sqrt;
    line_[index_ % imu_detail::kLines] = truth;
    const std::uint64_t src = index_ >= p_.latency ? index_ - p_.latency : 0;
    const std::array<double, 6>& t = line_[src % imu_detail::kLines];

    const noise::SampleNormals z = noise::sample_normals(seed_, noise::kStreamPrimaryImu, index_);
    const double root_dt = sqrt(dt);
    const double white_gyro = p_.gyro.density > 0.0 ? p_.gyro.density / sqrt(2.0 * dt) : 0.0;
    const double white_accel = p_.accel.density > 0.0 ? p_.accel.density / sqrt(2.0 * dt) : 0.0;

    ImuOut out;
    bool sat = false;
    for (std::size_t i = 0; i < 3; ++i) {
      bias_gyro_[i] += gain_gyro_ * root_dt * z[i];
      bias_accel_[i] += gain_accel_ * root_dt * z[6 + i];
      const double yg = t[i] + bias_gyro_[i] + white_gyro * z[3 + i];
      const double ya = t[3 + i] + bias_accel_[i] + white_accel * z[9 + i];
      out.gyro[i] = imu_detail::quantise(yg, p_.gyro, sat);
      out.flags |= sat ? (std::uint32_t{1} << (MARV_PLANT_IMU_GYRO_SAT_X + i)) : 0U;
      out.accel[i] = imu_detail::quantise(ya, p_.accel, sat);
      out.flags |= sat ? (std::uint32_t{1} << (MARV_PLANT_IMU_ACCEL_SAT_X + i)) : 0U;
    }
    out.flags |= (std::uint32_t{1} << MARV_PLANT_IMU_GYRO_VALID) | (std::uint32_t{1} << MARV_PLANT_IMU_ACCEL_VALID);
    ++index_;
    return out;
  }

 private:
  bool attached_ = false;
  ImuParams p_;
  std::uint64_t seed_ = 0;
  std::uint64_t index_ = 0;
  double gain_gyro_ = 0.0;
  double gain_accel_ = 0.0;
  std::array<double, 3> bias_gyro_{};
  std::array<double, 3> bias_accel_{};
  std::array<std::array<double, 6>, imu_detail::kLines> line_{};
};

}  // namespace marv::plant
