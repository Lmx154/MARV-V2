#pragma once

// A double re-implementation of PX4's AttitudeControl::setProportionalGain and AttitudeControl::update (BSD-3), steps
// transcribed from src/modules/mc_att_control/AttitudeControl/AttitudeControl.cpp at commit
// 3b828e157a6d9b7d811c80c73d95fd39fca5e181, lines 39-48 and 50-103. Test-only; it is the independent oracle the closed
// form of decision 0006 C is compared with. PX4 thresholds (the near-opposite switch and the Quaternion(src, dst) eps)
// are kept here, where they belong to the oracle, and a sample that reaches them is reported so the test can skip it.
//
// Differences from the PX4 text, all stated:
//   - binary64 throughout, so the rounding of the oracle is negligible against the binary32 closed form;
//   - the rate limit (the last loop of update) is not applied; the test compares unclamped rates;
//   - PX4's canonical() FLT_EPSILON threshold is replaced by the exact sign rule (w < 0 flips); the two differ only
//     for |w| < FLT_EPSILON, which the agreement samples do not reach except through the near-opposite skip.

#include <array>
#include <cmath>

namespace marv::l5_attitude_oracle {

struct Qd {
  double w = 1;
  double x = 0;
  double y = 0;
  double z = 0;
};

inline Qd mul(const Qd& a, const Qd& b) {
  return Qd{a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z, a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x, a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
}

inline Qd inv(const Qd& a) { return Qd{a.w, -a.x, -a.y, -a.z}; }

inline Qd normalized(const Qd& a) {
  const double n = std::sqrt(a.w * a.w + a.x * a.x + a.y * a.y + a.z * a.z);
  return Qd{a.w / n, a.x / n, a.y / n, a.z / n};
}

inline Qd canonical(const Qd& a) { return a.w < 0 ? Qd{-a.w, -a.x, -a.y, -a.z} : a; }

// Third column of the rotation matrix (matrix::Quaternion::dcm_z).
inline std::array<double, 3> dcm_z(const Qd& q) {
  return {2 * (q.x * q.z + q.w * q.y), 2 * (q.y * q.z - q.w * q.x), q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z};
}

struct Px4Result {
  std::array<double, 3> rate{};
  bool near_opposite = false;  // PX4 took a branch the closed form does not reproduce
};

// PX4's two thresholds, verbatim from the commit above: AttitudeControl.cpp:64 (1 - 1e-5) and matrix
// Quaternion(src, dst, eps = 1e-5).
constexpr double kPx4OppositeThreshold = 1.0 - 1e-5;
constexpr double kPx4QuatEps = 1e-5;

// q, qd: the measured and desired attitudes (normalised here). kp: the proportional gain of all three axes. yaw_w: the
// yaw weight. compensate: setProportionalGain divides the yaw gain by the weight. yawspeed: the world-z feed-forward.
inline Px4Result px4_update(const Qd& q_in, const Qd& qd_in, double kp, double yaw_w, bool compensate,
                            double yawspeed) {
  Px4Result out;
  const Qd q = normalized(q_in);
  Qd qd = normalized(qd_in);
  std::array<double, 3> gain{kp, kp, kp};
  if (compensate) {
    gain[2] /= yaw_w;
  }

  const std::array<double, 3> e_z = dcm_z(q);
  const std::array<double, 3> e_z_d = dcm_z(qd);
  // Quatf qd_red(e_z, e_z_d): the half-way quaternion of matrix::Quaternion(src, dst), normal branch.
  const std::array<double, 3> cr{e_z[1] * e_z_d[2] - e_z[2] * e_z_d[1], e_z[2] * e_z_d[0] - e_z[0] * e_z_d[2],
                                 e_z[0] * e_z_d[1] - e_z[1] * e_z_d[0]};
  const double dt = e_z[0] * e_z_d[0] + e_z[1] * e_z_d[1] + e_z[2] * e_z_d[2];
  const double cr_norm = std::sqrt(cr[0] * cr[0] + cr[1] * cr[1] + cr[2] * cr[2]);
  if (cr_norm < kPx4QuatEps && dt < 0) {
    out.near_opposite = true;
    return out;
  }
  const double n_src = e_z[0] * e_z[0] + e_z[1] * e_z[1] + e_z[2] * e_z[2];
  const double n_dst = e_z_d[0] * e_z_d[0] + e_z_d[1] * e_z_d[1] + e_z_d[2] * e_z_d[2];
  Qd qd_red = normalized(Qd{dt + std::sqrt(n_src * n_dst), cr[0], cr[1], cr[2]});

  if (std::fabs(qd_red.x) > kPx4OppositeThreshold || std::fabs(qd_red.y) > kPx4OppositeThreshold) {
    out.near_opposite = true;
    return out;
  }
  qd_red = mul(qd_red, q);  // qd_red *= q

  Qd qd_dyaw = canonical(mul(inv(qd_red), qd));
  qd_dyaw.w = std::fmin(std::fmax(qd_dyaw.w, -1.0), 1.0);
  qd_dyaw.z = std::fmin(std::fmax(qd_dyaw.z, -1.0), 1.0);

  qd = mul(qd_red, Qd{std::cos(yaw_w * std::acos(qd_dyaw.w)), 0, 0, std::sin(yaw_w * std::asin(qd_dyaw.z))});

  const Qd qe = canonical(mul(inv(q), qd));
  for (int i = 0; i < 3; ++i) {
    const double eq = 2 * (i == 0 ? qe.x : (i == 1 ? qe.y : qe.z));
    out.rate[static_cast<std::size_t>(i)] = eq * gain[static_cast<std::size_t>(i)];
  }
  const std::array<double, 3> zb = dcm_z(inv(q));
  for (std::size_t i = 0; i < 3; ++i) {
    out.rate[i] += zb[i] * yawspeed;
  }
  return out;
}

}  // namespace marv::l5_attitude_oracle
