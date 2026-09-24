// Cascaded multirotor controller: reference and navigation state -> NED force and body torque request.
//
//   position P -> velocity PI -> acceleration (+ reference feed-forward) -> force m (a - g), tilt and
//   thrust limited -> desired attitude (force direction + reference yaw) -> quaternion attitude P ->
//   body-rate PID -> torque I alpha + w x I w.
#pragma once

#include <marv/fsw/contracts.hpp>

namespace marv {

class Controller {
public:
    // Anything but kFly: zero request, integrators reset.
    ControlRequest run(const Reference& ref, const State& nav, Mode mode, float dt);

private:
    Vec3 iv_{0.f, 0.f, 0.f};      // integrated velocity error, m
    Vec3 iw_{0.f, 0.f, 0.f};      // integrated body-rate error, rad
    Vec3 w_prev_{0.f, 0.f, 0.f};  // body rate of the previous tick, for the derivative
    bool have_prev_ = false;
};

}  // namespace marv
