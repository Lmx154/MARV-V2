#pragma once

// Frame maps of the Gazebo adapter (decision 0003 item 6). Host-free, double. Gazebo works in a world ENU frame with
// bodies FLU; marv_plant works in world NED with bodies FRD.
//
//   world ENU <-> NED  (x, y, z) -> (y, x, -z)   a signed permutation, its own inverse
//   body  FLU <-> FRD  (x, y, z) -> (x, -y, -z)  a signed permutation, its own inverse
//
// Only unary negation is used, so both maps are bit-exact both ways, +0 and -0 included.
//
// Attitude. With q_ne = (0, 1/sqrt 2, 1/sqrt 2, 0) (a 180 degree rotation about (1,1,0)/sqrt 2, ENU -> NED) and
// q_fb = (0, 1, 0, 0) (180 degrees about x, FRD -> FLU), q_nb = q_ne (x) q_eu (x) q_fb, which written out is
//   q_nb = s * (-(w + z), -(x + y), y - x, z - w),  s = 1/sqrt 2,  q_eu = (w, x, y, z).
// The map is its own inverse (applying it twice gives 2 s^2 q = q), so NED/FRD -> ENU/FLU is the same formula. s is
// not representable, so no binary64 formula is exact. s^ = fl(sqrt 2) / 2 (the division by 2 is exact). The sign of
// the result is not canonicalised: q and -q map to n and -n.
//
// Rounding bound of the attitude map. Component i is s^ * fl(a +- b) (a, b components of q_eu). Relative errors:
// s^ vs s <= u, the sum <= u, the product <= u, so |e_i| <= 3 u s |a +- b| <= 3 u ||q|| (|a +- b| <= sqrt 2 ||q||,
// s sqrt 2 = 1). To first order in u. The independent route (two Hamilton products) has the same bound, the second
// product being exact (factors 0 and 1), so the two differ by <= 6 u ||q||. The round trip is
// M(M q + e1) + e2 = q + M e1 + e2 with |(M e1)_i| <= s (|e1_a| + |e1_b|) <= 3 sqrt 2 u ||q||, hence
// <= (3 + 3 sqrt 2) u ||q|| per component.
//
// Body rates. Gazebo gives the world-frame angular velocity, so omega_frd = R(q_nb)^T * perm(omega_enu_world). The
// nine entries of R are evaluated as in marv::prim::Quat::to_rotation_matrix, then three-term dot products in a fixed
// order. Rounding bound for a unit q_nb: a diagonal entry 1 - 2(a^2 + b^2) has error <= 2 u S (two squares and a sum,
// S = a^2 + b^2 <= 1) doubled, plus u |entry| <= 4 u + u = 5 u; an off-diagonal entry 2(ab - cd) has error <= 2 u
// (|ab| + |cd| + |ab - cd| <= 1); so every entry has error <= 5 u. In R^T v each product adds u |R v| and each of the
// two sums adds u times a partial sum, and every |partial sum| <= sum |R_ji v_j| <= ||v|| (a column of R has unit
// norm); the entry errors add 5 u sum |v_j| <= 5 sqrt 3 u ||v||. Total per component
//   |d omega_i| <= (5 sqrt 3 + 3) u ||omega||_2,
// to first order in u.

#include <array>
#include <cstddef>

#include "marv/gz/constants.hpp"
#include "marv_plant.h"

namespace marv::gz {

using Vec3 = std::array<double, 3>;
using Quat = std::array<double, 4>;  // [w, x, y, z], Hamilton

inline Vec3 enu_to_ned_world(const Vec3& v) { return {v[1], v[0], -v[2]}; }
inline Vec3 ned_to_enu_world(const Vec3& v) { return {v[1], v[0], -v[2]}; }
inline Vec3 flu_to_frd_body(const Vec3& v) { return {v[0], -v[1], -v[2]}; }
inline Vec3 frd_to_flu_body(const Vec3& v) { return {v[0], -v[1], -v[2]}; }

// s^ = fl(sqrt 2) / 2, exact halving of the cited constant.
inline constexpr double kAttitudeScale = kSqrt2 / 2.0;

// q_eu (FLU body -> ENU world) to q_nb (FRD body -> NED world).
inline Quat attitude_enu_flu_to_ned_frd(const Quat& q) {
  const double w = q[0];
  const double x = q[1];
  const double y = q[2];
  const double z = q[3];
  return {kAttitudeScale * -(w + z), kAttitudeScale * -(x + y), kAttitudeScale * (y - x), kAttitudeScale * (z - w)};
}

// The same formula, the map being its own inverse.
inline Quat attitude_ned_frd_to_enu_flu(const Quat& q) { return attitude_enu_flu_to_ned_frd(q); }

// omega_frd = R(q_nb)^T perm(omega_enu_world).
inline Vec3 omega_frd_from_world_enu(const Quat& q_nb, const Vec3& omega_enu_world) {
  const double w = q_nb[0];
  const double x = q_nb[1];
  const double y = q_nb[2];
  const double z = q_nb[3];
  const double xx = x * x;
  const double yy = y * y;
  const double zz = z * z;
  const double xy = x * y;
  const double xz = x * z;
  const double yz = y * z;
  const double wx = w * x;
  const double wy = w * y;
  const double wz = w * z;
  const double r00 = 1.0 - 2.0 * (yy + zz);
  const double r01 = 2.0 * (xy - wz);
  const double r02 = 2.0 * (xz + wy);
  const double r10 = 2.0 * (xy + wz);
  const double r11 = 1.0 - 2.0 * (xx + zz);
  const double r12 = 2.0 * (yz - wx);
  const double r20 = 2.0 * (xz - wy);
  const double r21 = 2.0 * (yz + wx);
  const double r22 = 1.0 - 2.0 * (xx + yy);
  const Vec3 v = enu_to_ned_world(omega_enu_world);
  return {r00 * v[0] + r10 * v[1] + r20 * v[2], r01 * v[0] + r11 * v[1] + r21 * v[2],
          r02 * v[0] + r12 * v[1] + r22 * v[2]};
}

// The state Gazebo reports for the body, in its own frames: world ENU, body FLU.
struct GzState {
  Vec3 pos_enu_m;
  Quat q_eu_wxyz;           // FLU body -> ENU world
  Vec3 lin_vel_world_m_s;   // ENU
  Vec3 ang_vel_world_rad_s; // ENU (Gazebo gives the world-frame angular velocity)
};

inline marv_plant_body to_plant_body(const GzState& s) {
  marv_plant_body b{};
  b.struct_size = sizeof(b);
  const Vec3 p = enu_to_ned_world(s.pos_enu_m);
  const Vec3 v = enu_to_ned_world(s.lin_vel_world_m_s);
  const Quat q = attitude_enu_flu_to_ned_frd(s.q_eu_wxyz);
  const Vec3 w = omega_frd_from_world_enu(q, s.ang_vel_world_rad_s);
  for (std::size_t i = 0; i < 3; ++i) {
    b.pos_ned_m[i] = p[i];
    b.vel_ned_m_s[i] = v[i];
    b.omega_frd_rad_s[i] = w[i];
  }
  for (std::size_t i = 0; i < 4; ++i) {
    b.q_wxyz[i] = q[i];
  }
  return b;
}

// Force and torque on the body, both world-frame vectors: NED (marv_plant) to ENU (Gazebo).
struct Wrench {
  Vec3 force;
  Vec3 torque;
};

inline Wrench wrench_ned_to_enu(const Vec3& force_ned_n, const Vec3& torque_ned_nm) {
  return {ned_to_enu_world(force_ned_n), ned_to_enu_world(torque_ned_nm)};
}

}  // namespace marv::gz
