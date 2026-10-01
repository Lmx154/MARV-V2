#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "marv/prim/constants.hpp"
#include "marv/prim/gravity.hpp"
#include "marv/prim/mat.hpp"
#include "marv/prim/quat.hpp"
#include "marv/prim/vec.hpp"
#include "marv/types/actuator.hpp"

namespace marv::plant {

inline constexpr std::size_t kMotors = kQuadXMotors;

// Model parameters, already validated by the caller. Scalar-templated; the C ABI instantiates double.
template <class T>
struct Params {
  T mass = T(0);
  std::array<prim::Vec3<T>, kMotors> rotor_position;  // FRD, m
  std::array<T, kMotors> yaw_sign{};                  // +1 ccw viewed from above (reaction torque +z_FRD), -1 cw
  T thrust_coeff = T(0);
  T torque_ratio = T(0);
  T omega_min = T(0);
  T omega_max = T(0);
  T tau = T(0);
  T substep = T(0);
  T site_lat = T(0);
  T site_height = T(0);
  std::array<T, kMotors> omega0{};                    // rotor speed at the first step, rad/s
};

template <class T>
struct Wrench {
  prim::Vec3<T> force_ned;
  prim::Vec3<T> torque_ned;
};

template <class T>
class Model {
 public:
  explicit Model(const Params<T>& p) : p_(p), omega_(p.omega0) {}

  // 2 pi: exactly twice the double nearest pi (a doubling is exact), so it is the double nearest 2 pi.
  static constexpr T kTwoPi = T(2) * static_cast<T>(prim::kPi);

  // ESC map, linear in omega: DShot 0 -> 0; kDshotThrottleMin..kDshotThrottleMax -> omega_min..omega_max.
  T omega_cmd(std::uint16_t dshot) const {
    if (dshot == 0) {
      return T(0);
    }
    const T span = static_cast<T>(prim::kDshotThrottleMax - prim::kDshotThrottleMin);
    const T frac = static_cast<T>(dshot - prim::kDshotThrottleMin) / span;
    return p_.omega_min + (p_.omega_max - p_.omega_min) * frac;
  }

  // Advances every motor by dt under the held commands. Exact zero-order-hold solution of
  // d omega/dt = (omega_cmd - omega)/tau per fixed sub-step h: omega <- c + (omega - c) exp(-h/tau); floor(dt/h)
  // whole sub-steps, then one partial sub-step of the remainder (if positive), so the times sum to dt.
  //
  // Rotor angle (L6 stage (b), decision 0013). Over a sub-step of length s the speed is c + (omega0 - c) exp(-t/tau), so
  // its exact integral, the angle turned, is
  //     integral_0^s omega dt = c s + (omega0 - c) tau (1 - exp(-s/tau)),
  // with 1 - exp(-s/tau) taken as -expm1(-s/tau) so that it keeps its relative precision for s << tau. At omega0 = c
  // (steady state) the second term is exactly 0 and the angle turned is c s, one rounding. The angle is kept in
  // [0, 2 pi) by theta - 2 pi floor(theta / 2 pi) after every sub-step (the increment is far below 2 pi for any
  // physical speed; a larger one is still reduced exactly the same way). The reduction subtracts the double 2 pi, which
  // is 2.45e-16 rad below the real 2 pi, so each wrap carries that systematic phase error; the addition rounds to at
  // most half an ulp of theta (4.5e-16 rad) per sub-step. The omega update is unchanged, so every output of the plant
  // is bit-identical to before the angle existed.
  void advance(const std::array<std::uint16_t, kMotors>& dshot, T dt) {
    using std::exp;
    using std::expm1;
    using std::floor;
    std::array<T, kMotors> cmd{};
    for (std::size_t i = 0; i < kMotors; ++i) {
      cmd[i] = omega_cmd(dshot[i]);
    }
    const T whole = floor(dt / p_.substep);
    const T rest = dt - whole * p_.substep;
    const T decay = exp(-p_.substep / p_.tau);
    const T turn = -expm1(-p_.substep / p_.tau);
    for (T n = T(0); n < whole; n += T(1)) {
      substep(cmd, p_.substep, decay, turn);
    }
    if (rest > T(0)) {
      substep(cmd, rest, exp(-rest / p_.tau), -expm1(-rest / p_.tau));
    }
  }

  // Wrench about the centre of mass from the current motor state, in NED. The quaternion is normalized here so a
  // quaternion within the ABI's tolerance still gives an exactly orthonormal rotation.
  Wrench<T> wrench(const prim::Quat<T>& q_body_to_ned, T pos_down) const {
    const prim::Vec3<T> z_frd(T(0), T(0), T(1));
    prim::Vec3<T> force_b;
    prim::Vec3<T> torque_b;
    for (std::size_t i = 0; i < kMotors; ++i) {
      const T thrust = p_.thrust_coeff * omega_[i] * omega_[i];
      const prim::Vec3<T> f = z_frd * (-thrust);
      force_b = force_b + f;
      torque_b = torque_b + p_.rotor_position[i].cross(f) + z_frd * (p_.yaw_sign[i] * p_.torque_ratio * thrust);
    }
    const prim::Mat3<T> rot = q_body_to_ned.normalized().to_rotation_matrix();
    const T g = prim::normal_gravity<T>(p_.site_lat, p_.site_height - pos_down);
    Wrench<T> w;
    w.force_ned = rot * force_b + prim::Vec3<T>(T(0), T(0), p_.mass * g);
    w.torque_ned = rot * torque_b;
    return w;
  }

  const std::array<T, kMotors>& omega() const { return omega_; }
  // Mechanical angle of each rotor, rad, in [0, 2 pi); 0 at the start.
  const std::array<T, kMotors>& theta() const { return theta_; }
  const Params<T>& params() const { return p_; }

 private:
  static T wrap(T angle) {
    using std::floor;
    T r = angle - kTwoPi * floor(angle / kTwoPi);
    if (r < T(0)) {
      r += kTwoPi;
    }
    if (r >= kTwoPi) {
      r -= kTwoPi;
    }
    return r;
  }

  // One sub-step of length `len`: decay = exp(-len/tau), turn = 1 - exp(-len/tau).
  void substep(const std::array<T, kMotors>& cmd, T len, T decay, T turn) {
    for (std::size_t i = 0; i < kMotors; ++i) {
      theta_[i] = wrap(theta_[i] + (cmd[i] * len + (omega_[i] - cmd[i]) * p_.tau * turn));
      omega_[i] = cmd[i] + (omega_[i] - cmd[i]) * decay;
    }
  }

  Params<T> p_;
  std::array<T, kMotors> omega_{};
  std::array<T, kMotors> theta_{};
};

}  // namespace marv::plant
