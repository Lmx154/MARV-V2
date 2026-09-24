#include <marv/fsw/actuators.hpp>

#include <cmath>

namespace marv {

Actuators::Actuators(const param::ActuatorParams& a) : a_(a) {}

void Actuators::run(ActuatorCommand& cmd, Mode mode) const {
    if (mode == Mode::kIdle || !cmd.armed) return;
    if (mode == Mode::kArmed) {
        for (float& m : cmd.motor) m = a_.spin_arm;
        return;
    }
    const float e = a_.thrust_expo;
    for (float& m : cmd.motor) {
        const float u = std::fmin(std::fmax(m, 0.f), 1.f);
        // The positive root of e x^2 + (1 - e) x - u = 0, written without the cancellation of the textbook form as e -> 0.
        const float x = u > 0.f ? 2.f * u / ((1.f - e) + std::sqrt((1.f - e) * (1.f - e) + 4.f * e * u)) : 0.f;
        m = a_.spin_min + (a_.spin_max - a_.spin_min) * x;
    }
}

}  // namespace marv
