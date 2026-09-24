// Cascaded multirotor controller: reference and navigation state -> NED force and body torque request.
//
//   position P -> velocity PI -> acceleration (+ reference feed-forward) -> force m (a - g), tilt and
//   thrust limited -> desired attitude (force direction + reference yaw) -> quaternion attitude P ->
//   body-rate PID -> torque I alpha + w x I w.
#pragma once

#include <marv/fsw/contracts.hpp>
#include <marv/fsw/params.hpp>

namespace marv {

class Controller {
public:
    // Gains and limits from the setup; the force limits are derived from them, the airframe and the rotors here.
    explicit Controller(const param::ControllerParams& c = {}, const param::VehicleParams& v = {},
                        const param::ActuatorParams& a = {});

    // Anything but kFly: zero request, integrators reset.
    ControlRequest run(const Reference& ref, const State& nav, Mode mode, float dt);

private:
    param::ControllerParams c_;
    float mass_;          // kg
    float gravity_;       // m/s^2
    Vec3 inertia_;        // principal, kg m^2
    float tan_tilt_max_;  // tan(tilt_max_deg)
    float thrust_min_;    // N, thrust_min_g m g
    float thrust_max_;    // N, thrust_max_frac of the four rotors' full thrust
    float iv_max_;        // m, velocity integrator limit: vel_int_accel / vel_i
    Vec3 iw_max_;         // rad, rate integrator limit: rate_int_accel / rate_i

    Vec3 iv_{0.f, 0.f, 0.f};      // integrated velocity error, m
    Vec3 iw_{0.f, 0.f, 0.f};      // integrated body-rate error, rad
    Vec3 w_prev_{0.f, 0.f, 0.f};  // body rate of the previous tick, for the derivative
    bool have_prev_ = false;
    float yaw_hold_ = 0.f;        // heading held under a yaw rate reference, rad
    bool have_hold_ = false;
};

}  // namespace marv
