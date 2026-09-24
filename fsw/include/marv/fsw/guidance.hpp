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
#include <marv/fsw/smoothing.hpp>

namespace marv {

// Guidance, trajectory (ADR-0011): the mission's position reference becomes a jerk-limited trajectory (PositionSmoothing,
// smoothing.hpp) on the navigation state the controller flies, as PX4's FlightTaskAuto drives it
// (src/modules/flight_mode_manager/tasks/Auto/FlightTaskAuto.cpp:183-226, 406-420, 764-809):
//   kRefPos, p_next == p   to p, ending at rest there; a kRefVel is added as a feed-forward velocity (the land leg)
//   kRefPos, p_next != p   along the triplet (previous, p, p_next), cornering inside accept_m; the previous waypoint is
//                          remembered here: the target the last change of p replaced (at the take-over, the vehicle)
//   no kRefPos, or mode != kFly   the reference unchanged, and the trajectory restarts from nav (position and velocity,
//                          acceleration zero) on the next kRefPos tick in kFly
// Horizontal speed: speed_mps, or cruise_speed when it is 0, at most xy_vel_max; vertical limits those of the direction
// the last unsmoothed velocity setpoint pointed. The integration slows while nav lags the trajectory by up to err_xy_max /
// err_z_max. Out: kRefPos, kRefVel and kRefAcc set to the trajectory. Heading: a kRefYaw passes through; with none the
// nose turns toward the horizontal trajectory velocity at up to yaw_rate_auto while it is faster than heading_min_speed,
// and holds otherwise (toolbox@3b7387b src/lib/sim/lab/blocks/guidance.ts:33), from the vehicle's heading at the take-over.
class TrajectoryGuidance {
public:
    explicit TrajectoryGuidance(const param::TrajectoryParams& p = {});

    Reference run(const Reference& ref, const State& nav, Mode mode, float dt);

private:
    param::TrajectoryParams p_;
    PositionSmoothing smoothing_;
    bool active_ = false;          // the trajectory runs (kFly with kRefPos since its last start)
    Vec3 prev_{0.f, 0.f, 0.f};     // the previous waypoint
    Vec3 target_{0.f, 0.f, 0.f};   // the last p
    float unsmoothed_z_ = 0.f;     // m/s, the last unsmoothed vertical velocity setpoint (down)
    float yaw_ = 0.f;              // rad, the heading reference
};

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
