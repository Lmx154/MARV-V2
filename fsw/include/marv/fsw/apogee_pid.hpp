// Controller, apogee-pid (toolbox controller.ts:141-170): brake deployment = clamp(kp e + ki I, 0, 1), e the predicted
// apogee with the brake closed minus the target, I = clamp(I + e dt, -i_limit, i_limit). Active only in kFly with the
// mission's kRefCoast and guidance's kRefApogee and a finite e; otherwise I = 0 and the brake closed. Every other field
// of the request is zero. float only, no heap.
#pragma once

#include <marv/fsw/contracts.hpp>
#include <marv/fsw/params.hpp>

namespace marv {

class ApogeePid {
public:
    explicit ApogeePid(const param::ApogeePidParams& p = {});

    ControlRequest run(const Reference& ref, Mode mode, float dt);

private:
    param::ApogeePidParams p_;
    float i_ = 0.f;  // m s, integrated apogee error
};

}  // namespace marv
