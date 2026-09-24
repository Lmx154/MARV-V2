// Allocation, rocket-brake (toolbox allocation.ts:59-69): the controller's brake deployment, clamped to 0..1, in kFly;
// closed otherwise. Every motor command is zero, always, and the command is never armed (no motor to spin).
#pragma once

#include <marv/fsw/contracts.hpp>

namespace marv {

class RocketBrake {
public:
    ActuatorCommand run(const ControlRequest& req, Mode mode) const;
};

}  // namespace marv
