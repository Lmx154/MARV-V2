#include "marv/null_plant/imu_synth.hpp"

namespace marv::null_plant {
namespace {

[[nodiscard]] float centred(SplitMix64& rng) noexcept { return (2.0F * rng.next_unit()) - 1.0F; }

// Clips v to +-limit; returns true iff it was clipped.
[[nodiscard]] bool clip(float& v, float limit) noexcept {
  if (v > limit) {
    v = limit;
    return true;
  }
  if (v < -limit) {
    v = -limit;
    return true;
  }
  return false;
}

constexpr std::uint32_t bit(std::uint32_t index) noexcept { return std::uint32_t{1} << index; }

}  // namespace

marv_imu_meas ImuSynth::next() noexcept {
  // Draw order (fixed): gyro x y z, accel x y z, temperature, then the three dropout draws (gyro, accel, temp).
  float g[3];
  for (int i = 0; i < 3; ++i) {
    gyro_[i] = (kGyroDecay * gyro_[i]) + (kGyroStep_rad_s * centred(rng_));
    g[i] = gyro_[i];
  }
  float a[3];
  for (int i = 0; i < 3; ++i) {
    a[i] = kAccelNoise_m_s2 * centred(rng_);
  }
  a[2] -= kGravity_m_s2;
  float temp = kTempBase_K + (kTempSpan_K * centred(rng_));
  const float gyro_drop = rng_.next_unit();
  const float accel_drop = rng_.next_unit();
  const float temp_drop = rng_.next_unit();

  marv_imu_meas m{};
  if (gyro_drop >= kDropoutProb) {
    m.flags |= bit(MARV_IMU_GYRO_VALID);
    for (int i = 0; i < 3; ++i) {
      if (clip(g[i], kGyroFullScale_rad_s)) {
        m.flags |= bit(static_cast<std::uint32_t>(MARV_IMU_GYRO_SAT_X + i));
      }
    }
    m.gyro_rad_s = marv_vec3f{g[0], g[1], g[2]};
  }
  if (accel_drop >= kDropoutProb) {
    m.flags |= bit(MARV_IMU_ACCEL_VALID);
    for (int i = 0; i < 3; ++i) {
      if (clip(a[i], kAccelFullScale_m_s2)) {
        m.flags |= bit(static_cast<std::uint32_t>(MARV_IMU_ACCEL_SAT_X + i));
      }
    }
    m.accel_m_s2 = marv_vec3f{a[0], a[1], a[2]};
  }
  if (temp_drop >= kDropoutProb) {
    m.flags |= bit(MARV_IMU_TEMP_VALID);
    m.temp_k = temp;
  }
  return m;
}

}  // namespace marv::null_plant
