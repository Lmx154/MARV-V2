#pragma once

// The boot gyro calibration (quad spec section 4.0 M3, decision 0042 approval): the per-axis mean of the gyro over a still
// window is the bias, and a window in which the vehicle moved is thrown away and started again. Pure: stepped with the
// plain sample and its stamp, it reads no clock, no HAL and no parameter. The caller passes the duration and the movement
// limit.
//
// The rule follows Betaflight 4.5.1 (tag 4.5.1 of github.com/betaflight/betaflight; INFERRED from a source read, no code
// copied): src/main/sensors/gyro.c, performGyroCalibration() lines 206-261. Every raw gyro sample from the start of the
// window adds to a running sum and a running variance (lines 220-221); on the final cycle of the window (line 223) the
// standard deviation of each axis (line 224) is compared with gyroMovementCalibrationThreshold and a deviation above it, on
// any axis, starts the window over (lines 234-235, 246-248); otherwise the sum over the cycle count is the zero of the axis
// (line 238) and the calibration is complete (isGyroSensorCalibrationComplete(), lines 136-139); from then on the zero is
// subtracted from every sample (gyroUpdateSensor(), lines 389-399). The window is a count of cycles in Betaflight
// (gyroCalculateCalibratingCycles(), lines 164-167, duration x 10000 / sample period); here it is a time, the span between
// the stamps of the first and the last accepted sample, so a dropped sample shortens the count and not the time.
//
// Here: update() takes one sample and its stamp. A sample whose valid flag is clear, or any of whose axes is not finite,
// is not accumulated and does not move the window. The window opens at the first accepted sample and closes at the first
// accepted sample whose stamp is at least `duration_us` after the opening one (that sample is included). At the close the
// per-axis sample standard deviation (n - 1 in the denominator, as Betaflight's devVariance) of every axis must not exceed
// `movement_max`, in rad/s, and at least two samples must have been accepted; then the bias is the per-axis mean and
// complete() is true for good (until reset()). Otherwise the window is discarded and the next accepted sample opens a new
// one. A stamp before the opening one restarts the window on that sample. A limit that is not a number never completes.
// Welford's recurrence gives the mean and the sum of squared deviations in one pass.

#include <cmath>
#include <cstddef>
#include <cstdint>

#include "marv/prim/vec.hpp"

namespace marv::estimator {

template <class T>
struct GyroCalConfig {
  std::uint64_t duration_us = 0;  // span of one window, microseconds
  T movement_max{};               // largest per-axis standard deviation of a still window, rad/s
};

template <class T>
class GyroCal {
 public:
  void init(const GyroCalConfig<T>& config) noexcept {
    cfg_ = config;
    reset();
  }

  void reset() noexcept {
    bias_ = prim::Vec3<T>();
    complete_ = false;
    restart();
  }

  // gyro rad/s, body FRD; valid is the sample's GyroValid flag; t_us its stamp.
  void update(const prim::Vec3<T>& gyro, bool valid, std::uint64_t t_us) noexcept {
    if (complete_ || !valid || !(std::isfinite(gyro[0]) && std::isfinite(gyro[1]) && std::isfinite(gyro[2]))) {
      return;
    }
    if (n_ == 0 || t_us < start_us_) {
      restart();
      start_us_ = t_us;
    }
    ++n_;
    const T n = static_cast<T>(n_);
    for (std::size_t i = 0; i < prim::kSpatialDim; ++i) {
      const T delta = gyro[i] - mean_[i];
      mean_[i] += delta / n;
      m2_[i] += delta * (gyro[i] - mean_[i]);
    }
    if (t_us - start_us_ < cfg_.duration_us) {
      return;
    }
    bool still = n_ >= 2;
    for (std::size_t i = 0; i < prim::kSpatialDim && still; ++i) {
      still = std::sqrt(m2_[i] / (n - T(1))) <= cfg_.movement_max;
    }
    if (still) {
      bias_ = mean_;
      complete_ = true;
    } else {
      restart();
    }
  }

  [[nodiscard]] bool complete() const noexcept { return complete_; }
  // The bias, rad/s (the gyro reads omega + bias): zero until complete().
  [[nodiscard]] const prim::Vec3<T>& bias() const noexcept { return bias_; }

 private:
  void restart() noexcept {
    mean_ = prim::Vec3<T>();
    m2_ = prim::Vec3<T>();
    n_ = 0;
    start_us_ = 0;
  }

  GyroCalConfig<T> cfg_{};
  prim::Vec3<T> bias_{};
  prim::Vec3<T> mean_{};
  prim::Vec3<T> m2_{};
  std::uint32_t n_ = 0;
  std::uint64_t start_us_ = 0;
  bool complete_ = false;
};

}  // namespace marv::estimator
