#include <marv/fsw/rocket_brake.hpp>

#include <cmath>

namespace marv {

ActuatorCommand RocketBrake::run(const ControlRequest& req, Mode mode) const {
    ActuatorCommand cmd{};
    // A rocket in kFly is armed even with the brake closed: the setup store keys its save lockout on `armed`, and a
    // ~50 ms flash erase during a flight would stall the brake loop. Motors stay zero under a rocket setup.
    cmd.armed = mode == Mode::kFly || mode == Mode::kArmed;
    if (mode == Mode::kFly) cmd.brake = std::fmin(std::fmax(req.brake, 0.f), 1.f);
    return cmd;
}

}  // namespace marv
