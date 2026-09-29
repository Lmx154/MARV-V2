#pragma once

#include <cmath>

#include "marv/prim/mat.hpp"
#include "marv/prim/vec.hpp"

namespace marv::prim {

// Hamilton unit quaternion q = w + x i + y j + z k, scalar first. The memory layout is exactly
// [w, x, y, z] in that order, four T with no padding (core section 3, Attitude).
//
// Attitude convention: q_nb rotates body (FRD) to world (NED): v_n = rotate(q_nb, v_b).
// Composition is the Hamilton product with the rotation applied on the right first:
//   q_na = q_nb * q_ba      rotate(q_na, v_a) == rotate(q_nb, rotate(q_ba, v_a))
// (q_ba rotates frame a into frame b; q_nb then rotates frame b into n.) The reverse order
// q_ba * q_nb is a different rotation in general.
//
// Rotation sense is right-handed about each axis, so from_axis_angle((0,0,1), +pi/2) is a
// positive yaw (nose right) and maps body x to NED east.
template <class T>
struct Quat {
  T w = T(1);
  T x = T(0);
  T y = T(0);
  T z = T(0);

  constexpr Quat() = default;
  constexpr Quat(T w_, T x_, T y_, T z_) : w(w_), x(x_), y(y_), z(z_) {}

  static constexpr Quat identity() { return Quat(T(1), T(0), T(0), T(0)); }

  // axis must be a unit vector; angle in rad, right-handed about axis.
  static Quat from_axis_angle(const Vec3<T>& axis, T angle) {
    using std::cos;
    using std::sin;
    const T half = angle * T(0.5);
    const T s = sin(half);
    return Quat(cos(half), axis[0] * s, axis[1] * s, axis[2] * s);
  }

  // Hamilton product (composition), see the convention above.
  constexpr Quat operator*(const Quat& r) const {
    return Quat(w * r.w - x * r.x - y * r.y - z * r.z,
                w * r.x + x * r.w + y * r.z - z * r.y,
                w * r.y - x * r.z + y * r.w + z * r.x,
                w * r.z + x * r.y - y * r.x + z * r.w);
  }

  constexpr Quat conjugate() const { return Quat(w, -x, -y, -z); }

  constexpr T norm2() const { return w * w + x * x + y * y + z * z; }

  T norm() const {
    using std::sqrt;
    return sqrt(norm2());
  }

  // Precondition: norm() > 0. Not guarded; a zero quaternion yields non-finite components.
  Quat normalized() const {
    const T inv = T(1) / norm();
    return Quat(w * inv, x * inv, y * inv, z * inv);
  }

  // Unique sign: w >= 0; when w == 0 the first nonzero of x, y, z is positive. q and -q are
  // the same rotation; this returns the one satisfying the rule.
  constexpr Quat canonical() const {
    bool flip;
    if (w != T(0)) {
      flip = w < T(0);
    } else if (x != T(0)) {
      flip = x < T(0);
    } else if (y != T(0)) {
      flip = y < T(0);
    } else {
      flip = z < T(0);
    }
    return flip ? Quat(-w, -x, -y, -z) : *this;
  }

  // Rotates v by this (unit) quaternion: v + w t + u x t with u = (x, y, z), t = 2 (u x v).
  constexpr Vec3<T> rotate(const Vec3<T>& v) const {
    const Vec3<T> u(x, y, z);
    const Vec3<T> t = u.cross(v) * T(2);
    return v + t * w + u.cross(t);
  }

  // Rotation matrix R with R * v == rotate(v), for a unit quaternion.
  constexpr Mat3<T> to_rotation_matrix() const {
    const T xx = x * x;
    const T yy = y * y;
    const T zz = z * z;
    const T xy = x * y;
    const T xz = x * z;
    const T yz = y * z;
    const T wx = w * x;
    const T wy = w * y;
    const T wz = w * z;
    Mat3<T> m;
    m(0, 0) = T(1) - T(2) * (yy + zz);
    m(0, 1) = T(2) * (xy - wz);
    m(0, 2) = T(2) * (xz + wy);
    m(1, 0) = T(2) * (xy + wz);
    m(1, 1) = T(1) - T(2) * (xx + zz);
    m(1, 2) = T(2) * (yz - wx);
    m(2, 0) = T(2) * (xz - wy);
    m(2, 1) = T(2) * (yz + wx);
    m(2, 2) = T(1) - T(2) * (xx + yy);
    return m;
  }
};

}  // namespace marv::prim
