#include <marv/fsw/allocation.hpp>

#include <cmath>

#include <marv/fsw/math.hpp>

namespace marv {
namespace {

// Rotor thrust per unit of roll, pitch and yaw fraction: ArduPilot's quad X factors normalized to a largest of 0.5
// (AP_MotorsMatrix.cpp:1353-1368), in the X3's rotor order, which is PX4's quad X: 0 front right, 1 rear left,
// 2 front left, 3 rear right. Roll (FRD +x, right side down) raises the left rotors, pitch (nose up) the front ones.
// Yaw (FRD +z, nose right) raises the ccw rotors 0 and 1: SDF:519/536/553 label rotors 0, 1 ccw and 2, 3 cw, and it
// was measured in the sim (2026-09-23, world paused, motors commanded open-loop, odometry read after 256 steps of 1 ms):
//   rotor speeds (750, 750, 650, 650) rad/s -> FLU angular z = -0.0815 rad/s (FRD +0.0815: nose right)
//   rotor speeds (650, 650, 750, 750) rad/s -> FLU angular z = +0.0815 rad/s (FRD -0.0815: nose left)
// (the model predicted 0.016 * 2 * k * (750^2 - 650^2) / Izz = 0.39 rad/s^2, about 0.09 rad/s by then), so a ccw rotor
// pushes the body +z FRD and a cw rotor -z FRD.
constexpr float kRoll[kMotorCount] = {-0.5f, 0.5f, 0.5f, -0.5f};
constexpr float kPitch[kMotorCount] = {0.5f, -0.5f, 0.5f, -0.5f};
constexpr float kYaw[kMotorCount] = {0.5f, 0.5f, -0.5f, -0.5f};

}  // namespace

ActuatorCommand Allocation::run(const ControlRequest& req, const State& nav, Mode mode) {
    ActuatorCommand cmd{};
    cmd.t_us = nav.t_us;
    if (mode == Mode::kArmed) cmd.armed = true;
    if (mode != Mode::kFly) return cmd;

    // Collective: the requested thrust along the thrust axis (-z body) the vehicle has now.
    const float request = std::fmax(0.f, dot(req.thrust_ned, rotate(nav.q, Vec3{0.f, 0.f, -1.f})));

    // Per-rotor thrust fraction split into three parts; the roll/pitch and yaw parts are zero-mean.
    float rp[kMotorCount], yaw[kMotorCount];
    for (int i = 0; i < kMotorCount; ++i) {
        rp[i] = kRoll[i] * req.torque_frd.x + kPitch[i] * req.torque_frd.y;
        yaw[i] = kYaw[i] * req.torque_frd.z;
    }

    // Saturation. Each rotor has the range [0, 1] of its full thrust. Priority: the collective up to the hover
    // thrust, then roll/pitch, then the rest of the collective, then yaw.
    //  1. Floor c_min: the collective kept whatever the torques, the request up to the hover thrust.
    //  2. Roll/pitch: scaled by one factor (the torque direction holds) so the highest rotor stays within
    //     min(1/2, 1 - c_min) of the collective, and the spread within 1.
    //  3. Collective: the request, clamped into the band roll/pitch leave free; never below c_min.
    //  4. Yaw: scaled by the largest s in [0, 1] that keeps every rotor in [0, 1]; it never moves the collective.
    const float c_min = std::fmin(request, req.thrust_hover);
    float lo = rp[0], hi = rp[0];
    for (float v : rp) {
        lo = std::fmin(lo, v);
        hi = std::fmax(hi, v);
    }
    const float hi_max = std::fmin(0.5f, 1.f - c_min);
    float k = 1.f;
    if (hi > hi_max) k = hi_max / hi;
    if (k * (hi - lo) > 1.f) k = 1.f / (hi - lo);
    if (k < 1.f) {
        for (float& v : rp) v *= k;
        lo *= k;
        hi *= k;
    }
    const float c = std::fmin(std::fmax(request, -lo), 1.f - hi);
    float s = 1.f;
    for (int i = 0; i < kMotorCount; ++i) {
        const float base = c + rp[i];
        if (yaw[i] > 0.f) s = std::fmin(s, (1.f - base) / yaw[i]);
        else if (yaw[i] < 0.f) s = std::fmin(s, -base / yaw[i]);
    }
    s = std::fmax(s, 0.f);

    for (int i = 0; i < kMotorCount; ++i) cmd.motor[i] = std::fmin(std::fmax(c + rp[i] + s * yaw[i], 0.f), 1.f);
    cmd.armed = true;
    return cmd;
}

}  // namespace marv
