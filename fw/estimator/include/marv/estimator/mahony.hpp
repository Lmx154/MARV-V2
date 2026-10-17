#pragma once

// The Mahony attitude filter (quad spec section 4.0 M3, L7): the explicit complementary filter with gyro-bias estimate of
//   R. Mahony, T. Hamel, J.-M. Pflimlin, "Nonlinear Complementary Filters on the Special Orthogonal Group", IEEE
//   Transactions on Automatic Control 53(5), 2008, pp. 1203-1218, in its quaternion form (the explicit complementary
//   filter with bias: the estimate follows the corrected gyro rate and the bias estimate integrates the same error with the
//   opposite sign).
// Pure: stepped once per IMU tick with plain inputs, it reads no clock, no HAL and no parameter. The caller passes the gains.
//
// Frames and units (core section 3). Body is FRD, the world NED. q is the Hamilton unit quaternion [w, x, y, z] that rotates
// body into NED, normalised every step, sign canonical (w >= 0). Gyro in rad/s (body), accelerometer in m/s^2 as a specific
// force (body), dt from the caller's microsecond difference. At rest the specific force is the reaction to gravity: level it
// is (0, 0, -g) in FRD.
//
// The step. Let v = f / |f| be the measured specific-force direction (body) and vhat = -R^T e_z the direction the estimate
// predicts (R the rotation matrix of q, e_z the NED down axis, so vhat = -(R20, R21, R22)). The error is the cross product
//   e = v x vhat                       (zero when they agree, perpendicular to both otherwise)
// and, with w the gyro, bhat the bias estimate (the gyro reads omega + b):
//   omega_c = w - bhat + Kp e
//   bhat   <- bhat - Ki e dt           (bhat_dot = -Ki e)
//   q      <- q (x) exp(1/2 omega_c dt)   (exact for a constant omega_c over the step; the Euler update 1 + 1/2 omega_c dt
//                                         followed by the normalisation is the same to first order)
// Sign check. With phi the small rotation of the estimate against the truth, e = -phi (the part perpendicular to v) and
// phi_dot = -(bhat - b) - Kp phi, (bhat - b)_dot = Ki phi: the tilt error and the bias error obey s^2 + Kp s + Ki = 0, so
// the natural frequency is sqrt(Ki), the damping ratio Kp / (2 sqrt(Ki)), and Kp = 2 sqrt(Ki) is critical damping with a
// double pole at -Kp / 2. Only the part of the bias perpendicular to v is observable (there is no magnetometer: the
// rotation about the gravity direction, the yaw, and the bias along it are not corrected).
//
// Accelerometer use. The correction runs only when |f|^2 is finite and greater than 0; otherwise that step follows the gyro
// alone (a zero vector has no direction, a non-finite one carries none). No other gating is applied: not a norm window
// around g, not a rate limit on the integral, no gain schedule. The first valid sample initialises the attitude from its
// tilt, with yaw 0 (the tilt is the roll phi and pitch theta of R = Rz(0) Ry(theta) Rx(phi), for which
// f = g (sin theta, -cos theta sin phi, -cos theta cos phi), so phi = atan2(-fy, -fz) and theta = atan2(fx, sqrt(fy^2 + fz^2))).
// Until then step() changes nothing and initialised() is false.
//
// Gains. The configuration carries Kp (1/s) and Ki (1/s^2); this library compiles none in. Candidate source for the
// parameters of the flight composition (INFERRED until a committed test shows them suit this filter): Betaflight 4.5.1
// (tag 4.5.1 of github.com/betaflight/betaflight), src/main/flight/imu.c,
//   line 125   .imu_dcm_kp = 2500,   default Kp, stored x 10000 (line 176: imuDcmKp = imu_dcm_kp / 10000.0f), so Kp = 0.25
//   line 126   .imu_dcm_ki = 0,      default Ki, stored x 10000 (line 177: imuDcmKi = imu_dcm_ki / 10000.0f), so Ki = 0
//   (the comments on lines 125-126 read "1.0 * 10000" and "0.003 * 10000" and disagree with the values; the line 175 comment
//   reads "current default for imu_dcm_kp is 2500; our 'normal' or baseline value for imuDcmKp is 0.25"; the range of both
//   settings is 0 .. 32000, src/main/cli/settings.c lines 1006-1007).
// Betaflight's Kp and Ki have the units of this filter (the error is the unit-vector cross product, the integral
// accumulates Ki e dt into a rate added to the gyro, lines 261 and 272). It also schedules Kp: 100x for a window after a
// quiet period of 250 ms with the gyro below 15 deg/s while disarmed (lines 87-88, 364-377, 391), 10x otherwise while
// disarmed (line 394), 1x when armed (line 398); it integrates only below a spin rate of 20 deg/s (lines 85, 259) and uses the
// accelerometer only when its squared norm in g exceeds 0.01 (line 242) and, per its comment, all smoothed axes are within
// 20 % of 1 g (lines 243, 690). None of that is implemented here.

#include <cmath>
#include <cstdint>

#include "marv/prim/constants.hpp"
#include "marv/prim/quat.hpp"
#include "marv/prim/vec.hpp"

namespace marv::estimator {

template <class T>
struct MahonyConfig {
  T kp{};  // proportional gain, 1/s
  T ki{};  // integral gain, 1/s^2
};

template <class T>
class Mahony {
 public:
  void init(const MahonyConfig<T>& config) noexcept {
    cfg_ = config;
    reset();
  }

  void reset() noexcept {
    q_ = prim::Quat<T>::identity();
    bias_ = prim::Vec3<T>();
    initialised_ = false;
  }

  // gyro rad/s and specific force m/s^2, both body FRD; dt_us the time since the previous step, in microseconds.
  void step(const prim::Vec3<T>& gyro, const prim::Vec3<T>& accel, std::uint32_t dt_us) noexcept {
    const T n2 = accel.dot(accel);
    const bool accel_ok = std::isfinite(n2) && n2 > T(0);
    if (!initialised_) {
      if (accel_ok) {
        q_ = from_tilt(accel);
        initialised_ = true;
      }
      return;
    }
    const T dt = static_cast<T>(dt_us) / static_cast<T>(prim::kMicrosecondsPerSecond);
    prim::Vec3<T> e;
    if (accel_ok) {
      const prim::Vec3<T> v = accel * (T(1) / std::sqrt(n2));
      const prim::Vec3<T> vhat(-T(2) * (q_.x * q_.z - q_.w * q_.y), -T(2) * (q_.y * q_.z + q_.w * q_.x),
                               -(T(1) - T(2) * (q_.x * q_.x + q_.y * q_.y)));
      e = v.cross(vhat);
    }
    const prim::Vec3<T> omega = gyro - bias_ + e * cfg_.kp;
    bias_ = bias_ - e * (cfg_.ki * dt);
    q_ = (q_ * exp_half(omega * dt)).normalized().canonical();
  }

  [[nodiscard]] const prim::Quat<T>& quat() const noexcept { return q_; }
  [[nodiscard]] const prim::Vec3<T>& bias() const noexcept { return bias_; }  // rad/s, the gyro reads omega + bias
  [[nodiscard]] bool initialised() const noexcept { return initialised_; }

 private:
  // exp(1/2 phi) for the rotation vector phi: (cos(|phi|/2), phi sin(|phi|/2) / |phi|).
  static prim::Quat<T> exp_half(const prim::Vec3<T>& phi) noexcept {
    const T angle = phi.norm();
    if (!(angle > T(0))) {
      return prim::Quat<T>::identity();
    }
    const T half = angle * T(0.5);
    const T k = std::sin(half) / angle;
    return prim::Quat<T>(std::cos(half), phi[0] * k, phi[1] * k, phi[2] * k);
  }

  static prim::Quat<T> from_tilt(const prim::Vec3<T>& f) noexcept {
    const T roll = std::atan2(-f[1], -f[2]);
    const T pitch = std::atan2(f[0], std::sqrt(f[1] * f[1] + f[2] * f[2]));
    const prim::Quat<T> qp = prim::Quat<T>::from_axis_angle(prim::Vec3<T>(T(0), T(1), T(0)), pitch);
    const prim::Quat<T> qr = prim::Quat<T>::from_axis_angle(prim::Vec3<T>(T(1), T(0), T(0)), roll);
    return (qp * qr).normalized().canonical();
  }

  MahonyConfig<T> cfg_{};
  prim::Quat<T> q_ = prim::Quat<T>::identity();
  prim::Vec3<T> bias_{};
  bool initialised_ = false;
};

}  // namespace marv::estimator
