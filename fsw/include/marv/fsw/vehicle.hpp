// The airframe the flight software flies: the X3 of sitl/gazebo/marv_quad.sdf, every number taken from
// that file (line cited). Body frame FRD; the SDF's is FLU, so y and z change sign.
#pragma once

#include <marv/fsw/contracts.hpp>

namespace marv::vehicle {

inline constexpr float kGravity = 9.8066f;  // SDF:71 <gravity>0 0 -9.8066</gravity>

// base_link 1.5 kg (SDF:161) + four rotors of 0.005 kg (SDF:259, 324, 389, 454).
inline constexpr float kMass = 1.5f + 4.f * 0.005f;

// base_link principal inertia, kg m^2 (SDF:163, 166, 168). The rotors' point masses would add under 4 %
// on each axis and are left out.
inline constexpr float kIxx = 0.0347563f;
inline constexpr float kIyy = 0.07f;
inline constexpr float kIzz = 0.0977f;

// Rotor i is Gazebo rotor i (actuator_number i). Positions in FRD, m: the SDF poses (FLU) are
// rotor_0 (0.13, -0.22) SDF:256, rotor_1 (-0.13, 0.2) SDF:321, rotor_2 (0.13, 0.22) SDF:386,
// rotor_3 (-0.13, -0.2) SDF:451. The 0.023 m height is along the thrust axis and makes no moment.
inline constexpr float kRotorX[kMotorCount] = {0.13f, -0.13f, 0.13f, -0.13f};
inline constexpr float kRotorY[kMotorCount] = {0.22f, -0.2f, -0.22f, 0.2f};

// Yaw reaction on the body, FRD z, per unit rotor thrust, in units of kMomentConstant.
// SDF:519/536/553 label rotors 0, 1 ccw and 2, 3 cw. Measured in the sim (2026-09-23, world paused,
// motors commanded open-loop, odometry read after 256 steps of 1 ms):
//   rotor speeds (750, 750, 650, 650) rad/s -> FLU angular z = -0.0815 rad/s (FRD +0.0815: nose right)
//   rotor speeds (650, 650, 750, 750) rad/s -> FLU angular z = +0.0815 rad/s (FRD -0.0815: nose left)
// The model predicts 0.016 * 2 * k * (750^2 - 650^2) / kIzz = 0.39 rad/s^2, about 0.09 rad/s by then.
// So a ccw rotor (0, 1) pushes the body +z FRD and a cw rotor (2, 3) pushes it -z FRD.
inline constexpr float kRotorYaw[kMotorCount] = {1.f, 1.f, -1.f, -1.f};

inline constexpr float kMotorConstant = 8.54858e-06f;  // N / (rad/s)^2, thrust = k w^2, SDF:523
inline constexpr float kMomentConstant = 0.016f;       // m, yaw torque = km * thrust, SDF:524
inline constexpr float kMaxRotVelocity = 800.f;        // rad/s, SDF:522
inline constexpr float kMaxRotorThrust = kMotorConstant * kMaxRotVelocity * kMaxRotVelocity;  // 5.47 N

}  // namespace marv::vehicle
