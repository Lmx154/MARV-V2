#include <marv/fsw/guidance.hpp>

#include <cmath>

namespace marv {
namespace {

constexpr float kRho0 = 1.225f;         // kg/m^3, toolbox plant.ts RHO0
constexpr float kScaleHeight = 8500.f;  // m, toolbox guidance.ts SCALE_HEIGHT
constexpr float kPredictCapS = 60.f;    // s, toolbox guidance.ts PREDICT_CAP

}  // namespace

ApogeePredictor::ApogeePredictor(const param::ApogeePredictorParams& p, const param::SensorParams& s)
    : dt_(p.predict_dt),
      drag_(0.5f * kRho0 * p.cd_base * p.ref_area / p.mass),
      gravity_(s.gravity),
      target_(p.target_apogee_m) {}

Reference ApogeePredictor::run(const Reference& ref, const State& nav, Mode mode, std::uint64_t t_us) {
    Reference out = ref;
    out.has = static_cast<std::uint8_t>(ref.has & ~kRefApogee);
    if (mode != Mode::kFly || !(ref.has & kRefCoast)) {
        have_ = false;
        return out;
    }
    if (!have_ || t_us - t_pred_us_ >= kPeriodUs) {
        a0_ = apogee(-nav.p_ned.z, -nav.v_ned.z, std::sqrt(nav.v_ned.x * nav.v_ned.x + nav.v_ned.y * nav.v_ned.y));
        t_pred_us_ = t_us;
        have_ = true;
    }
    out.has = static_cast<std::uint8_t>(out.has | kRefApogee);
    out.apogee_m = target_;
    out.apogee_pred_m = a0_;
    return out;
}

// guidance.ts:62-74 with the brake closed (u = 0).
float ApogeePredictor::apogee(float h, float vz, float vh) {
    float z = h, w = vz, x = vh, t = 0.f;
    steps_ = 0;
    for (; w > 0.f && t < kPredictCapS && steps_ < kMaxSteps; t += dt_) {
        ++steps_;
        const float k = drag_ * std::exp(-z / kScaleHeight) * std::sqrt(w * w + x * x);
        const float w1 = w - (gravity_ + k * w) * dt_;
        if (w1 <= 0.f) return z + 0.5f * w * w * dt_ / (w - w1);
        x -= k * x * dt_;
        z += 0.5f * (w + w1) * dt_;
        w = w1;
    }
    return z;
}

}  // namespace marv
