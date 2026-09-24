// Guidance, apogee-predictor (toolbox guidance.ts:40-127): while the mission flags the coast, rolls a point-mass model of
// the rocket from the navigation state's altitude, vertical speed and horizontal speed to apogee with the brake closed
// (predictApogee, guidance.ts:62-74), and hands the controller the prediction and the target apogee. The rollout runs on a
// fixed 20 ms cadence (the toolbox's FSW tick) and its result is held in between; one rollout is at most kMaxSteps
// integration steps. Model: drag 1/2 rho V^2 cd_base ref_area against the velocity, rho = 1.225 kg/m^3 exp(-h / 8500 m)
// (the toolbox's RHO0 and scale height), gravity from the sensors. float only, no heap.
#pragma once

#include <cstdint>

#include <marv/fsw/contracts.hpp>
#include <marv/fsw/params.hpp>

namespace marv {

class ApogeePredictor {
public:
    static constexpr int kMaxSteps = 400;
    static constexpr std::uint64_t kPeriodUs = 20000;

    explicit ApogeePredictor(const param::ApogeePredictorParams& p = {}, const param::SensorParams& s = {});

    // In kFly with kRefCoast: kRefApogee set, apogee_m the target, apogee_pred_m the latest rollout from nav (a new one
    // on the first coast tick and every kPeriodUs after). Otherwise the reference unchanged but for kRefApogee cleared.
    Reference run(const Reference& ref, const State& nav, Mode mode, std::uint64_t t_us);

    // Apogee (m above the start) from altitude h (m), vertical speed vz (m/s, up) and horizontal speed vh (m/s), brake
    // closed: semi-implicit Euler at predict_dt, the step where vz crosses zero interpolated, at most kMaxSteps steps
    // and 60 s of coast.
    float apogee(float h, float vz, float vh);
    // Integration steps of the last rollout.
    int steps() const { return steps_; }

private:
    float dt_;       // s, predict_dt
    float drag_;     // 1/m: 1/2 rho0 cd_base ref_area / mass
    float gravity_;  // m/s^2
    float target_;   // m, target_apogee_m
    float a0_ = 0.f;
    std::uint64_t t_pred_us_ = 0;
    bool have_ = false;  // a rollout of this coast is held
    int steps_ = 0;
};

}  // namespace marv
