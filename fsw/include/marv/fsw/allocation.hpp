// Allocation: the controller's force and torque request -> one command per rotor, for the X quad of
// vehicle.hpp.
#pragma once

#include <marv/fsw/contracts.hpp>

namespace marv {

class Allocation {
public:
    Allocation();

    // Saturation priority: the collective up to the hover thrust m g, then roll/pitch (scaled together,
    // the direction kept), then the rest of the collective, then yaw (only ever cut, never paid for with
    // collective). Idle: every motor zero, disarmed.
    ActuatorCommand run(const ControlRequest& req, const State& nav, Mode mode);

private:
    // Rotor thrust per unit of (collective N, roll N m, pitch N m, yaw N m): the inverse of the mixer
    // [sum T; sum -y T; sum x T; sum km s T] built from the rotor geometry.
    float inv_[kMotorCount][4];
};

}  // namespace marv
