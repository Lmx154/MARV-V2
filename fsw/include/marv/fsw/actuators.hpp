// Actuators, rotor-speed-fraction: each rotor's thrust fraction -> its motor command, in place on the ActuatorCommand
// the allocation produced. The command is spin_min + (spin_max - spin_min) x, x the inverse of the thrust curve
// thrust = (1 - e) x + e x^2 (AP_Motors_Thrust_Linearization.cpp:104-107). No battery compensation.
#pragma once

#include <marv/fsw/contracts.hpp>
#include <marv/fsw/params.hpp>

namespace marv {

class Actuators {
public:
    explicit Actuators(const param::ActuatorParams& a = {});

    // kIdle or disarmed: unchanged (every motor zero). kArmed: every motor spin_arm. kFly: the curve. The brake is not
    // touched.
    void run(ActuatorCommand& cmd, Mode mode) const;

private:
    param::ActuatorParams a_;
};

}  // namespace marv
