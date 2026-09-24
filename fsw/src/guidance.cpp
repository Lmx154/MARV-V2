#include <marv/fsw/guidance.hpp>

#include <cmath>

#include <marv/fsw/math.hpp>

namespace marv {
namespace {

constexpr float kRho0 = 1.225f;         // kg/m^3, toolbox plant.ts RHO0
constexpr float kScaleHeight = 8500.f;  // m, toolbox guidance.ts SCALE_HEIGHT
constexpr float kPredictCapS = 60.f;    // s, toolbox guidance.ts PREDICT_CAP

// PX4@af2e7b4311 src/modules/mc_pos_control/multicopter_autonomous_params.yaml:111 MPC_XY_TRAJ_P default 0.5.
constexpr float kXyTrajP = 0.5f;
// PX4@af2e7b4311 src/modules/navigator/navigator_params.yaml:61 NAV_MC_ALT_RAD default 0.8 m.
constexpr float kAltAcceptRad = 0.8f;
// FlightTaskAuto.cpp:453-457: a target that moves less than this on every axis is the same target.
constexpr float kSameTarget = 0.001f;  // m
constexpr float kPi = 3.14159265358979f;

float wrap_pi(float a) {
    while (a > kPi) a -= 2.f * kPi;
    while (a < -kPi) a += 2.f * kPi;
    return a;
}

}  // namespace

TrajectoryGuidance::TrajectoryGuidance(const param::TrajectoryParams& p) : p_(p) {
    // FlightTaskAuto.cpp:767-777.
    smoothing_.set_max_allowed_horizontal_error(p.err_xy_max);
    smoothing_.set_max_allowed_vertical_error(p.err_z_max);
    smoothing_.set_vertical_acceptance_radius(kAltAcceptRad);
    smoothing_.set_horizontal_trajectory_gain(kXyTrajP);
    smoothing_.set_max_jerk(p.jerk[param::k_profile_hold]);
}

Reference TrajectoryGuidance::run(const Reference& ref, const State& nav, Mode mode, float dt) {
    if (mode != Mode::kFly || !(ref.has & kRefPos)) {
        active_ = false;
        return ref;
    }
    if (!active_) {
        smoothing_.reset({0.f, 0.f, 0.f}, nav.v_ned, nav.p_ned);
        prev_ = target_ = nav.p_ned;
        unsmoothed_z_ = 0.f;
        yaw_ = yaw_of(nav.q);
        active_ = true;
    }
    // FlightTaskAuto.cpp:453-481: a new target, and the one it replaces becomes the previous waypoint.
    if (!(std::fabs(ref.p_ned.x - target_.x) < kSameTarget && std::fabs(ref.p_ned.y - target_.y) < kSameTarget &&
          std::fabs(ref.p_ned.z - target_.z) < kSameTarget)) {
        prev_ = target_;
        target_ = ref.p_ned;
    }

    // FlightTaskAuto.cpp:406-420 and 770-808: the cruise speed, and the vertical limits of the direction the last
    // unsmoothed velocity setpoint pointed.
    smoothing_.set_cruise_speed(
        std::fmin(ref.speed_mps > 0.f ? ref.speed_mps : p_.cruise_speed[param::k_profile_hold], p_.xy_vel_max));
    smoothing_.set_target_acceptance_radius(ref.accept_m);
    const bool up = unsmoothed_z_ < 0.f;
    const std::uint8_t h = param::k_profile_hold;
    smoothing_.set_max_acceleration({p_.acc_xy[h], p_.acc_xy[h], up ? p_.acc_up[h] : p_.acc_dn[h]});
    smoothing_.set_max_velocity({p_.xy_vel_max, p_.xy_vel_max, up ? p_.z_vel_up : p_.z_vel_dn});

    const Vec3 ff = (ref.has & kRefVel) ? ref.v_ned : Vec3{0.f, 0.f, 0.f};
    PositionSmoothing::Setpoints sp;
    const bool single = ref.p_next_ned.x == ref.p_ned.x && ref.p_next_ned.y == ref.p_ned.y && ref.p_next_ned.z == ref.p_ned.z;
    if (single) {
        smoothing_.generate_setpoints(nav.p_ned, ref.p_ned, ff, dt, false, sp);
    } else {
        const Vec3 waypoints[3] = {prev_, ref.p_ned, ref.p_next_ned};
        smoothing_.generate_setpoints(nav.p_ned, waypoints, ff, dt, false, sp);
    }
    unsmoothed_z_ = sp.unsmoothed_velocity.z;

    Reference out = ref;
    out.has = static_cast<std::uint8_t>(ref.has | kRefPos | kRefVel | kRefAcc | kRefYaw);
    out.p_ned = sp.position;
    out.v_ned = sp.velocity;
    out.a_ned = sp.acceleration;
    if (ref.has & kRefYaw) {
        yaw_ = ref.yaw;
    } else if (std::sqrt(sp.velocity.x * sp.velocity.x + sp.velocity.y * sp.velocity.y) > p_.heading_min_speed) {
        const float step = p_.yaw_rate_auto[param::k_profile_hold] * dt;
        const float e = wrap_pi(std::atan2(sp.velocity.y, sp.velocity.x) - yaw_);
        yaw_ = wrap_pi(yaw_ + std::fmin(std::fmax(e, -step), step));
    }
    out.yaw = yaw_;
    return out;
}

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
