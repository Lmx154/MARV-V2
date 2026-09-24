// Allocation: the controller's force and torque request -> one command per rotor, for the X quad of the setup's
// vehicle (rotor positions, moment constant, mass, gravity) and actuators (motor constant, full rotor speed).
#pragma once

#include <marv/fsw/contracts.hpp>
#include <marv/fsw/params.hpp>

namespace marv {

class Allocation {
public:
    explicit Allocation(const param::VehicleParams& v = {}, const param::ActuatorParams& a = {});

    // Saturation priority: the collective up to the hover thrust m g, then roll/pitch (scaled together,
    // the direction kept), then the rest of the collective, then yaw (only ever cut, never paid for with
    // collective). Idle: every motor zero, disarmed.
    ActuatorCommand run(const ControlRequest& req, const State& nav, Mode mode);

private:
    // Rotor thrust per unit of (collective N, roll N m, pitch N m, yaw N m): the inverse of the mixer
    // [sum T; sum -y T; sum x T; sum km s T] built from the rotor geometry.
    float inv_[kMotorCount][4];
    float hover_thrust_;       // N per rotor, m g / 4
    float max_thrust_;         // N per rotor, k w_max^2
    float motor_constant_;     // N / (rad/s)^2
    float max_rot_velocity_;   // rad/s
};

}  // namespace marv
