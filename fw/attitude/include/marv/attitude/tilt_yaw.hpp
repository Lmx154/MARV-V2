#pragma once

// The split of a rotation error into a reduced attitude (tilt) and a yaw part (decision 0006 C, step 3):
// q_e = q_xy (x) q_z with q_xy = (rho, a, b, 0) and q_z a rotation about body z.

#include <cmath>

#include "marv/prim/quat.hpp"

namespace marv::attitude {

template <class T>
[[nodiscard]] bool is_finite(const prim::Quat<T>& q) noexcept {
  using std::isfinite;
  return isfinite(q.w) && isfinite(q.x) && isfinite(q.y) && isfinite(q.z);
}

template <class T>
struct TiltYawSplit {
  prim::Quat<T> q_xy{};   // (rho, a, b, 0); q_e itself when singular
  T rho{};                // sqrt(w^2 + z^2)
  T a{};
  T b{};
  T psi{};                // yaw angle of q_z, rad, in [-pi, pi]; 0 when singular
  bool singular = false;  // rho == 0 exactly
};

// pre: q_e is canonical (w >= 0) and finite. With rho = sqrt(w^2 + z^2) (hypot, which neither overflows nor underflows,
// so rho == 0 iff w == z == 0):
//   rho > 0:  a = (w x - y z) / rho, b = (w y + x z) / rho, q_z = canonical((w, 0, 0, z) / rho),
//             psi = 2 atan2(q_z.z, q_z.w)
//   rho == 0: q_xy = q_e and psi = 0.
template <class T>
[[nodiscard]] TiltYawSplit<T> split_tilt_yaw(const prim::Quat<T>& q_e) noexcept {
  using std::atan2;
  using std::hypot;
  TiltYawSplit<T> s;
  s.rho = hypot(q_e.w, q_e.z);
  if (!(s.rho > T(0))) {
    s.q_xy = q_e;
    s.singular = true;
    return s;
  }
  s.a = (q_e.w * q_e.x - q_e.y * q_e.z) / s.rho;
  s.b = (q_e.w * q_e.y + q_e.x * q_e.z) / s.rho;
  const prim::Quat<T> q_z = prim::Quat<T>(q_e.w / s.rho, T(0), T(0), q_e.z / s.rho).canonical();
  s.psi = T(2) * atan2(q_z.z, q_z.w);
  s.q_xy = prim::Quat<T>(s.rho, s.a, s.b, T(0));
  return s;
}

}  // namespace marv::attitude
