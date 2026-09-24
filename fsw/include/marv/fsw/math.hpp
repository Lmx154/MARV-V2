// Vector and quaternion operations on the contract types. float only, header-only, no allocation.
// Quaternions are Hamilton, scalar first; rotate(q, v) maps a body vector into the world frame.
#pragma once

#include <cmath>

#include <marv/fsw/contracts.hpp>

namespace marv {

inline Vec3 operator+(Vec3 a, Vec3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline Vec3 operator-(Vec3 a, Vec3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline Vec3 operator-(Vec3 a) { return {-a.x, -a.y, -a.z}; }
inline Vec3 operator*(float s, Vec3 a) { return {s * a.x, s * a.y, s * a.z}; }
inline Vec3 operator*(Vec3 a, float s) { return s * a; }
inline Vec3& operator+=(Vec3& a, Vec3 b) { return a = a + b; }
inline Vec3& operator-=(Vec3& a, Vec3 b) { return a = a - b; }

inline float dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vec3 cross(Vec3 a, Vec3 b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
inline float norm(Vec3 a) { return std::sqrt(dot(a, a)); }
inline Vec3 normalized(Vec3 a) {
    const float n = norm(a);
    return n > 0.f ? (1.f / n) * a : a;
}

inline constexpr Quat kQuatIdentity{1.f, 0.f, 0.f, 0.f};

inline Quat operator*(Quat a, Quat b) {
    return {a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
            a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
}

inline Quat conj(Quat q) { return {q.w, -q.x, -q.y, -q.z}; }

inline Quat normalized(Quat q) {
    const float n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    return n > 0.f ? Quat{q.w / n, q.x / n, q.y / n, q.z / n} : kQuatIdentity;
}

// Rotates v by q: body -> world for an attitude quaternion.
inline Vec3 rotate(Quat q, Vec3 v) {
    const Vec3 u{q.x, q.y, q.z};
    const Vec3 t = 2.f * cross(u, v);
    return v + q.w * t + cross(u, t);
}

// World -> body.
inline Vec3 rotate_inv(Quat q, Vec3 v) { return rotate(conj(q), v); }

// Quaternion of the rotation vector r (axis * angle, rad).
inline Quat quat_from_rotvec(Vec3 r) {
    const float a = norm(r);
    if (a < 1e-6f) return normalized(Quat{1.f, 0.5f * r.x, 0.5f * r.y, 0.5f * r.z});
    const float s = std::sin(0.5f * a) / a;
    return {std::cos(0.5f * a), s * r.x, s * r.y, s * r.z};
}

// Rotation vector of q, on the short way round.
inline Vec3 rotvec_from_quat(Quat q) {
    if (q.w < 0.f) q = {-q.w, -q.x, -q.y, -q.z};
    const Vec3 u{q.x, q.y, q.z};
    const float s = norm(u);
    if (s < 1e-6f) return 2.f * u;
    return (2.f * std::atan2(s, q.w) / s) * u;
}

// ZYX Euler angles (rad) of a body -> NED attitude.
inline Quat quat_from_euler(float roll, float pitch, float yaw) {
    const float cr = std::cos(0.5f * roll), sr = std::sin(0.5f * roll);
    const float cp = std::cos(0.5f * pitch), sp = std::sin(0.5f * pitch);
    const float cy = std::cos(0.5f * yaw), sy = std::sin(0.5f * yaw);
    return {cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy};
}

inline float yaw_of(Quat q) { return std::atan2(2.f * (q.w * q.z + q.x * q.y), 1.f - 2.f * (q.y * q.y + q.z * q.z)); }

// Angle (rad) between two attitudes.
inline float quat_angle(Quat a, Quat b) { return norm(rotvec_from_quat(conj(a) * b)); }

}  // namespace marv
