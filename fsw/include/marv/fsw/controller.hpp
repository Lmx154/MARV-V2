// Cascaded multirotor controller: reference and navigation state -> thrust and torque request, normalized.
//
//   position P -> velocity PI -> acceleration (+ reference feed-forward) -> thrust fraction (hover / g)(a - g),
//   tilt and thrust limited -> desired attitude (thrust direction + reference yaw) -> attitude target shaped toward
//   it (ArduPilot's input shaping) -> quaternion attitude P on the target + its rate fed forward ->
//   body-rate PID -> torque as a fraction of each axis' full authority.
//
// The airframe enters only through the hover thrust: seeded each run from the setup's vehicle, learned in level
// hover (ArduPilot's low-pass, AP_MotorsMulticopter.cpp:561, gated as ArduCopter/Attitude.cpp:32-62), never stored.
//
// Flight profiles (ADR-0012): the tilt limit (tilt_max_deg) and the input shaping's time constant (input_tc) are the
// profile's; nothing else differs, so a switch keeps every gain, integrator, the learned hover thrust and the attitude
// target's state, which moves on at the new time constant.
#pragma once

#include <marv/fsw/contracts.hpp>
#include <marv/fsw/params.hpp>

namespace marv {

class Controller {
public:
    // Gains and limits from the setup; the hover thrust seed from the vehicle, gravity from the sensors.
    explicit Controller(const param::ControllerParams& c = {}, const param::UavParams& v = {},
                        const param::SensorParams& s = {});

    // Anything but kFly: zero request but the learned hover thrust, integrators reset. profile < param::kProfileCount.
    ControlRequest run(const Reference& ref, const State& nav, Mode mode, float dt,
                       std::uint8_t profile = param::k_profile_hold);
    // The velocity setpoint of the last run (position P plus the reference velocity, within vel_max); zero out of kFly.
    Vec3 velocity_setpoint() const { return v_sp_; }

private:
    param::ControllerParams c_;
    float gravity_;       // m/s^2
    float tan_tilt_max_[param::kProfileCount];  // tan(tilt_max_deg), by profile
    float hover_;         // learned hover thrust, fraction of full collective
    float hover_min_;     // the range of vehicle/uav hover_thrust
    float hover_max_;
    float iv_max_;        // m, horizontal velocity integrator limit: vel_int_accel / vel_i
    float iv_max_z_;      // m, vertical velocity integrator limit: gravity / vel_i
    Vec3 iw_max_;         // rad, rate integrator limit: rate_int_max / rate_i

    Vec3 v_sp_{0.f, 0.f, 0.f};    // the last velocity setpoint, m/s
    Vec3 iv_{0.f, 0.f, 0.f};      // integrated velocity error, m
    Vec3 iw_{0.f, 0.f, 0.f};      // integrated body-rate error, rad
    Vec3 w_prev_{0.f, 0.f, 0.f};  // body rate of the previous tick, for the derivative
    bool have_prev_ = false;
    float yaw_hold_ = 0.f;        // heading held under a yaw rate reference, rad
    bool have_hold_ = false;
    Quat q_t_{1.f, 0.f, 0.f, 0.f};  // attitude target, body -> NED
    Vec3 w_t_{0.f, 0.f, 0.f};       // its angular rate, target frame, rad/s
    Vec3 a_t_{0.f, 0.f, 0.f};       // its angular acceleration, target frame, rad/s^2
    bool have_target_ = false;
};

}  // namespace marv
