// Allocation: the controller's thrust and torque fractions -> one thrust fraction per rotor of a quad X, with fixed
// factors (no geometry). The actuators module turns each into a motor command.
#pragma once

#include <marv/fsw/contracts.hpp>

namespace marv {

class Allocation {
public:
    // Saturation priority: the collective up to the request's hover thrust, then roll/pitch (scaled together,
    // the direction kept), then the rest of the collective, then yaw (only ever cut, never paid for with
    // collective). Idle: every motor zero, disarmed. Armed: every motor zero, armed (the actuators spin them). The
    // brake is always zero.
    ActuatorCommand run(const ControlRequest& req, const State& nav, Mode mode);
};

}  // namespace marv
