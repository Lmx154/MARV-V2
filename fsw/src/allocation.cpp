#include <marv/fsw/allocation.hpp>

#include <cmath>

#include <marv/fsw/math.hpp>
#include <marv/fsw/vehicle.hpp>

namespace marv {

using namespace vehicle;

Allocation::Allocation() {
    // Mixer: rotor thrusts -> (collective, torque). A rotor at (x, y) thrusting T along -z makes the
    // moment (x, y, 0) x (0, 0, -T) = (-y T, x T, 0), and the yaw reaction km s T.
    float a[4][2 * kMotorCount];
    for (int i = 0; i < kMotorCount; ++i) {
        a[0][i] = 1.f;
        a[1][i] = -kRotorY[i];
        a[2][i] = kRotorX[i];
        a[3][i] = kMomentConstant * kRotorYaw[i];
        for (int j = 0; j < 4; ++j) a[j][kMotorCount + i] = j == i ? 1.f : 0.f;
    }
    // Gauss-Jordan with partial pivoting on [mixer | identity]; the right half becomes the inverse.
    for (int c = 0; c < 4; ++c) {
        int p = c;
        for (int r = c + 1; r < 4; ++r)
            if (std::fabs(a[r][c]) > std::fabs(a[p][c])) p = r;
        for (int k = 0; k < 2 * kMotorCount; ++k) {
            const float t = a[c][k];
            a[c][k] = a[p][k];
            a[p][k] = t;
        }
        const float d = 1.f / a[c][c];
        for (int k = 0; k < 2 * kMotorCount; ++k) a[c][k] *= d;
        for (int r = 0; r < 4; ++r) {
            if (r == c) continue;
            const float f = a[r][c];
            for (int k = 0; k < 2 * kMotorCount; ++k) a[r][k] -= f * a[c][k];
        }
    }
    for (int i = 0; i < kMotorCount; ++i)
        for (int j = 0; j < 4; ++j) inv_[i][j] = a[i][kMotorCount + j];
}

ActuatorCommand Allocation::run(const ControlRequest& req, const State& nav, Mode mode) {
    ActuatorCommand cmd{};
    cmd.t_us = nav.t_us;
    if (mode != Mode::kFly) return cmd;

    // Collective: the requested force along the thrust axis (-z body) the vehicle has now.
    const float collective = std::fmax(0.f, dot(req.force_ned, rotate(nav.q, Vec3{0.f, 0.f, -1.f})));

    // Per-rotor thrust split into three parts. The collective column is uniform (T/4 on every rotor),
    // since the rotors' moment arms and spins sum to zero.
    float rp[kMotorCount], yaw[kMotorCount];
    for (int i = 0; i < kMotorCount; ++i) {
        rp[i] = inv_[i][1] * req.torque_frd.x + inv_[i][2] * req.torque_frd.y;
        yaw[i] = inv_[i][3] * req.torque_frd.z;
    }

    // Saturation, priority roll/pitch > yaw > collective. Each rotor has the range [0, Tmax], so a
    // differential fits if its spread (max - min) is at most Tmax.
    //  1. Roll/pitch: if their spread alone exceeds Tmax, scale them down (the torque direction holds).
    //  2. Yaw: scale it by the largest s in [0, 1] that keeps the spread of rp + s yaw within Tmax.
    //  3. Collective: shift it into the band the differentials leave free.
    float lo = rp[0], hi = rp[0];
    for (float v : rp) {
        lo = std::fmin(lo, v);
        hi = std::fmax(hi, v);
    }
    if (hi - lo > kMaxRotorThrust) {
        const float s = kMaxRotorThrust / (hi - lo);
        for (float& v : rp) v *= s;
    }
    float s = 1.f;
    for (int i = 0; i < kMotorCount; ++i)
        for (int j = 0; j < kMotorCount; ++j) {
            const float dy = yaw[i] - yaw[j];
            if (dy > 0.f) s = std::fmin(s, (kMaxRotorThrust - (rp[i] - rp[j])) / dy);
        }
    s = std::fmax(s, 0.f);
    float d[kMotorCount];
    lo = hi = rp[0] + s * yaw[0];
    for (int i = 0; i < kMotorCount; ++i) {
        d[i] = rp[i] + s * yaw[i];
        lo = std::fmin(lo, d[i]);
        hi = std::fmax(hi, d[i]);
    }
    const float c = std::fmin(std::fmax(inv_[0][0] * collective, -lo), kMaxRotorThrust - hi);

    // Thrust -> rotor speed -> fraction of full speed.
    for (int i = 0; i < kMotorCount; ++i) {
        const float t = std::fmin(std::fmax(c + d[i], 0.f), kMaxRotorThrust);
        cmd.motor[i] = std::sqrt(t / kMotorConstant) / kMaxRotVelocity;
    }
    cmd.armed = true;
    return cmd;
}

}  // namespace marv
