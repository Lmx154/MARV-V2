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
    // since the rotors' moment arms and spins sum to zero; so is the sum of each torque column, so the
    // roll/pitch and yaw parts are zero-mean.
    float rp[kMotorCount], yaw[kMotorCount];
    for (int i = 0; i < kMotorCount; ++i) {
        rp[i] = inv_[i][1] * req.torque_frd.x + inv_[i][2] * req.torque_frd.y;
        yaw[i] = inv_[i][3] * req.torque_frd.z;
    }

    // Saturation. Each rotor has the range [0, Tmax]. Priority: the collective up to the hover thrust,
    // then roll/pitch, then the rest of the collective, then yaw.
    //  1. Floor c_min: the per-rotor collective kept whatever the torques, the request up to m g / 4.
    //  2. Roll/pitch: scaled by one factor (the torque direction holds) so the highest rotor stays within
    //     min(Tmax / 2, Tmax - c_min) of the collective, and the spread within Tmax.
    //  3. Collective: the request, clamped into the band roll/pitch leave free; never below c_min.
    //  4. Yaw: scaled by the largest s in [0, 1] that keeps every rotor in [0, Tmax]; it never moves
    //     the collective.
    const float request = inv_[0][0] * collective;
    const float c_min = std::fmin(request, kMass * kGravity / static_cast<float>(kMotorCount));
    float lo = rp[0], hi = rp[0];
    for (float v : rp) {
        lo = std::fmin(lo, v);
        hi = std::fmax(hi, v);
    }
    const float hi_max = std::fmin(0.5f * kMaxRotorThrust, kMaxRotorThrust - c_min);
    float k = 1.f;
    if (hi > hi_max) k = hi_max / hi;
    if (k * (hi - lo) > kMaxRotorThrust) k = kMaxRotorThrust / (hi - lo);
    if (k < 1.f) {
        for (float& v : rp) v *= k;
        lo *= k;
        hi *= k;
    }
    const float c = std::fmin(std::fmax(request, -lo), kMaxRotorThrust - hi);
    float s = 1.f;
    for (int i = 0; i < kMotorCount; ++i) {
        const float base = c + rp[i];
        if (yaw[i] > 0.f) s = std::fmin(s, (kMaxRotorThrust - base) / yaw[i]);
        else if (yaw[i] < 0.f) s = std::fmin(s, -base / yaw[i]);
    }
    s = std::fmax(s, 0.f);

    // Thrust -> rotor speed -> fraction of full speed.
    for (int i = 0; i < kMotorCount; ++i) {
        const float t = std::fmin(std::fmax(c + rp[i] + s * yaw[i], 0.f), kMaxRotorThrust);
        cmd.motor[i] = std::sqrt(t / kMotorConstant) / kMaxRotVelocity;
    }
    cmd.armed = true;
    return cmd;
}

}  // namespace marv
