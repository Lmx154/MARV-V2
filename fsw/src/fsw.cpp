#include <marv/fsw/fsw.hpp>

namespace marv {

ActuatorCommand Fsw::step(const SensorBus& bus) {
    ActuatorCommand cmd{};
    cmd.t_us = bus.t_us;
    cmd.armed = true;
    for (float& m : cmd.motor) m = motor_cmd_;
    return cmd;
}

}  // namespace marv
