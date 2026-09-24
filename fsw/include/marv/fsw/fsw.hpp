// One flight-software tick: sensors in, actuator commands out. Runs unchanged on the Pico and on the
// host. For now it is the pass-through used to prove the pipeline: armed, every motor at one fraction.
#pragma once

#include <marv/fsw/contracts.hpp>

namespace marv {

class Fsw {
public:
    explicit Fsw(float motor_cmd) : motor_cmd_(motor_cmd) {}

    ActuatorCommand step(const SensorBus& bus);

private:
    float motor_cmd_;
};

}  // namespace marv
