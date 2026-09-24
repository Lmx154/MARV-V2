// What of the airframe is structure, not a parameter: the spin direction of each rotor of the X3 of
// sitl/gazebo/marv_quad.sdf. Its mass, inertia, rotor positions and constants are the setup's (params.def).
#pragma once

#include <marv/fsw/contracts.hpp>

namespace marv::vehicle {

// Yaw reaction on the body, FRD z, per unit rotor thrust, in units of the moment constant. Rotor i is Gazebo
// rotor i (actuator_number i). SDF:519/536/553 label rotors 0, 1 ccw and 2, 3 cw. Measured in the sim (2026-09-23,
// world paused, motors commanded open-loop, odometry read after 256 steps of 1 ms):
//   rotor speeds (750, 750, 650, 650) rad/s -> FLU angular z = -0.0815 rad/s (FRD +0.0815: nose right)
//   rotor speeds (650, 650, 750, 750) rad/s -> FLU angular z = +0.0815 rad/s (FRD -0.0815: nose left)
// The model predicts 0.016 * 2 * k * (750^2 - 650^2) / Izz = 0.39 rad/s^2, about 0.09 rad/s by then.
// So a ccw rotor (0, 1) pushes the body +z FRD and a cw rotor (2, 3) pushes it -z FRD.
inline constexpr float kRotorYaw[kMotorCount] = {1.f, 1.f, -1.f, -1.f};

}  // namespace marv::vehicle
