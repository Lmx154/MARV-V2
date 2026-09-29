// Deterministic synthetic IMU stream for the null plant: a pure function of the seed and the tick index order.
//
// The stream is generated one sample per tick in tick order, 10 PRNG draws per sample in a fixed order, so the
// batching of ticks into marv_sil_tick calls cannot change it. Every number below is a labelled SCENARIO VALUE
// (core 2), not a vehicle or sensor number; the null plant models no physics.
//
// IMU validity rules (core 3, enforced at the SIL boundary): finite fields; no reserved flag bits; a clear valid bit
// means the field is exactly 0 and its saturation bits are clear; TempValid means temp_k > 0. A saturation bit is
// set for an FRD component whose value was clipped at the scenario full-scale limit.
#pragma once

#include <cstdint>

#include <marv_sil.h>

#include "marv/null_plant/prng.hpp"

namespace marv::null_plant {

// Scenario values.
inline constexpr float kGyroDecay = 0.95F;           // per-tick AR(1) decay of the gyro state
inline constexpr float kGyroStep_rad_s = 0.4F;       // half-width of the per-tick gyro innovation
inline constexpr float kGyroFullScale_rad_s = 2.0F;  // clip and saturation limit per axis
inline constexpr float kGravity_m_s2 = 9.80665F;     // standard gravity (CGPM 1901); FRD level: specific force -z
inline constexpr float kAccelNoise_m_s2 = 3.0F;      // half-width of the accel noise per axis
inline constexpr float kAccelFullScale_m_s2 = 12.0F; // clip and saturation limit per axis
inline constexpr float kTempBase_K = 300.0F;
inline constexpr float kTempSpan_K = 2.0F;           // half-width of the temperature excursion
inline constexpr float kDropoutProb = 0.03125F;      // 2^-5: chance per sensor per sample that its valid bit is clear

class ImuSynth {
 public:
  explicit ImuSynth(std::uint64_t seed) noexcept : rng_{seed} {}

  // The next sample (tick order). Always passes the SIL validity rules.
  [[nodiscard]] marv_imu_meas next() noexcept;

 private:
  SplitMix64 rng_;
  float gyro_[3] = {0.0F, 0.0F, 0.0F};
};

}  // namespace marv::null_plant
