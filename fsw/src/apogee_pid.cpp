#include <marv/fsw/apogee_pid.hpp>

#include <cmath>

namespace marv {

ApogeePid::ApogeePid(const param::ApogeePidParams& p) : p_(p) {}

ControlRequest ApogeePid::run(const Reference& ref, Mode mode, float dt) {
    ControlRequest req{};
    const float e = ref.apogee_pred_m - ref.apogee_m;
    if (mode != Mode::kFly || !(ref.has & kRefCoast) || !(ref.has & kRefApogee) || !std::isfinite(e)) {
        i_ = 0.f;
        return req;
    }
    i_ = std::fmin(std::fmax(i_ + e * dt, -p_.i_limit), p_.i_limit);
    req.brake = std::fmin(std::fmax(p_.kp * e + p_.ki * i_, 0.f), 1.f);
    return req;
}

}  // namespace marv
