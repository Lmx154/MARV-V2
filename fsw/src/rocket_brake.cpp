#include <marv/fsw/rocket_brake.hpp>

#include <cmath>

namespace marv {

ActuatorCommand RocketBrake::run(const ControlRequest& req, Mode mode) const {
    ActuatorCommand cmd{};
    if (mode == Mode::kFly) cmd.brake = std::fmin(std::fmax(req.brake, 0.f), 1.f);
    return cmd;
}

}  // namespace marv
