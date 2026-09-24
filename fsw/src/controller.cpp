#include <marv/fsw/controller.hpp>

#include <cmath>

#include <marv/fsw/math.hpp>

namespace marv {
namespace {

// The loops, with the factory gains of params.def. The translation loops are acceleration commands, turned into a
// thrust fraction by the hover thrust: thrust = (hover / g)(a - g) (PX4 PositionControl.cpp:220). The rate loop is a
// torque fraction per axis; its gains are the acceleration gains below times I / tau_max of the X3 (params.def), so
// each is still a bandwidth in the physical terms that follow.
//
// Rate loop, tau = Kp e + Ki int(e) - Kd dw/dt (rate_p, rate_i, rate_d). With I w' = tau_max tau the closed loop is
// first order at Kp tau_max / I rad/s. What limits it is the motor lag, 12.5 ms up / 25 ms down (SDF timeConstantUp / Down),
// i.e. poles at 40..80 rad/s, plus about 5 ms of the odometry's 10-sample rolling mean. Roll and pitch at 25 rad/s keep
// about 45 deg of phase margin against those; the integrator corner sits a decade lower (Ki = Kp * Kp / 10). Yaw at
// 10 rad/s: its authority is small (see yaw_torque_max), so more bandwidth only saturates. rate_int_max is the most
// the rate integrator may add; rate_max limits the rate setpoint.
// Input shaping (ArduPilot input_thrust_vector_heading_rad, AC_AttitudeControl.cpp:890-929, with ATC_RATE_FF_ENAB 1,
// AC_AttitudeControl.h:32): an attitude target moves toward the desired attitude, each axis of the rotation vector to it
// shaped by attitude_command_model (AC_AttitudeControl.cpp:1108-1130) within rate_max, accel_max and the jerk accel_max / input_tc.
// Entering fly it starts at the body, at rest; while the heading follows the body (below) its heading does too.
// Attitude loop, rate_sp = K e + w_t (att_p, e the rotation vector from the body to the target, w_t the target's rate in
// the body frame, AC_AttitudeControl.cpp:989-1029). On yaw K e is also capped at sqrt(2 a |e|) (a = yaw_decel), the
// speed from which a deceleration of a stops exactly at the target.
// Yaw torque limit (yaw_torque_max). At hover each rotor carries m g / 4 = 3.73 N and has 1.74 N left to its full
// thrust; the most yaw torque that fits in that headroom is 4 km 1.74 = 0.111 N m, and 0.1 N m (0.5712 of the X3's
// 0.17508 N m yaw authority) keeps 10 % of it for roll and pitch. The allocation's priority is collective up to the
// hover thrust, then roll/pitch, then the rest of the collective, then yaw: a yaw demand the rotors cannot carry is
// cut, never paid for with thrust.
// Yaw rate reference (kRefYawRate): while it is non-zero, or the body still turns faster than yaw_rate_still, the
// heading hold follows the vehicle and the rate loop tracks the reference; below it the heading is held.
// Translation, v_sp = Kp e_p, a = Kv (v_sp - v) + Ki int(v_sp - v) (pos_p, vel_p, vel_i). With the inner loops ideal
// the position error obeys e'' + Kv e' + Kv Kp e = 0: w_n = sqrt(Kv Kp) = 1.18 rad/s, about 7x below the attitude
// loop, and zeta = sqrt(Kv / Kp) / 2 = 0.85. Kv * vel_max = 4 m/s^2 bounds the acceleration a velocity step asks for
// to atan(4 / g) = 22 deg of tilt. The integrator (corner Ki / Kv = 0.25 rad/s, at most vel_int_accel horizontally) takes
// up what the thrust model and the rotor drag leave. Vertically it may reach 1 g, and holds while the thrust is at a limit
// the error pushes it toward (PX4 PositionControl.cpp:146, 157-162): a hover thrust seed far below the vehicle's is
// still lifted to the reference, where the hover thrust is learned.
// Thrust limits: tilt of the thrust vector within tilt_max_deg of vertical, vertical thrust between thrust_min_g
// hover thrusts and a total of thrust_max_frac of full thrust (the rest is left for the torques).
// Hover thrust: low-passed toward the vertical thrust asked for, time constant kHoverTc, only in level hover (no
// vertical speed or vertical velocity reference, roll and pitch within kHoverLevel), clamped to its parameter range.
// It also waits while the position loop's own vertical velocity setpoint is kHoverVsp or more: a position step asks
// for a climb with no velocity reference, and this keeps take-off and commanded climbs out of the estimate, the intent
// of ArduCopter's gate (learn only in steady hover).

constexpr float kRadPerDeg = 3.14159265358979f / 180.f;
constexpr float kHoverTc = 10.f;                     // s
constexpr float kHoverClimb = 0.6f;                  // m/s, ArduCopter/Attitude.cpp:57
constexpr float kHoverLevel = 5.f * kRadPerDeg;      // rad, ArduCopter/Attitude.cpp:58
constexpr float kHoverVsp = 0.1f;                    // m/s

float clampf(float v, float lo, float hi) { return std::fmin(std::fmax(v, lo), hi); }

Vec3 clamp_norm(Vec3 v, float max) {
    const float n = norm(v);
    return n > max ? (max / n) * v : v;
}

// ArduPilot's sqrt_controller (AP_Math/control.cpp:544-579): p e near zero, sqrt(2 a (|e| - a / 2p^2)) past a / p^2,
// never beyond the error in one tick.
float sqrt_controller(float e, float p, float a, float dt) {
    const float lin = a / (p * p);
    float r = e * p;
    if (e > lin) r = std::sqrt(2.f * a * (e - 0.5f * lin));
    else if (e < -lin) r = -std::sqrt(2.f * a * (-e - 0.5f * lin));
    return dt > 0.f ? clampf(r, -std::fabs(e) / dt, std::fabs(e) / dt) : r;
}

// Its rate of change while the state closes the error at w (sqrt_controller_accel, AP_Math/control.cpp:642-675).
float sqrt_controller_accel(float e, float r, float w, float p, float a) {
    if (!(r * w > 0.f)) return 0.f;
    if (std::fabs(e) <= a / (p * p)) return -p * w;
    return -(a / std::fabs(r)) * w;
}

// attitude_command_model with no rate input (AC_AttitudeControl.cpp:1108-1130, shape_angle_vel_accel and
// shape_pos_vel_accel, AP_Math/control.cpp:423-431, 259-336): the target's rate w and acceleration a on one axis, toward
// closing the angle e within the rate w_max, the acceleration a_max and the jerk a_max / tc.
void command_model(float e, float& w, float& a, float w_max, float a_max, float tc, float dt) {
    if (!(dt > 0.f)) return;
    const float k = 1.f / tc;  // jerk / a_max
    float v = sqrt_controller(e, k, a_max, dt);
    v = clampf(v + sqrt_controller_accel(e, v, w, k, a_max) / k, -w_max, w_max);
    const float a_cmd = clampf(k * (v - w), -a_max, a_max);
    a += clampf(a_cmd - a, -k * a_max * dt, k * a_max * dt);
    w += a * dt;
}

}  // namespace

Controller::Controller(const param::ControllerParams& c, const param::UavParams& v, const param::SensorParams& s)
    : c_(c),
      gravity_(s.gravity),
      tan_tilt_max_(std::tan(c.tilt_max_deg * kRadPerDeg)),
      hover_(v.hover_thrust),
      hover_min_(param::kParamMeta[param::k_vehicle_uav_hover_thrust].min),
      hover_max_(param::kParamMeta[param::k_vehicle_uav_hover_thrust].max),
      iv_max_(c.vel_int_accel / c.vel_i),
      iv_max_z_(s.gravity / c.vel_i),
      iw_max_{c.rate_int_max_x / c.rate_i_x, c.rate_int_max_y / c.rate_i_y, c.rate_int_max_z / c.rate_i_z} {}

ControlRequest Controller::run(const Reference& ref, const State& nav, Mode mode, float dt) {
    if (mode != Mode::kFly) {
        iv_ = iw_ = w_prev_ = {0.f, 0.f, 0.f};
        have_prev_ = false;
        have_hold_ = false;
        have_target_ = false;
        return {{0.f, 0.f, 0.f}, {0.f, 0.f, 0.f}, hover_, 0.f};
    }

    // Translation: position P -> velocity PI -> acceleration.
    const Vec3 p_ref = (ref.has & kRefPos) ? ref.p_ned : nav.p_ned;
    const Vec3 v_ref = (ref.has & kRefVel) ? ref.v_ned : Vec3{0.f, 0.f, 0.f};
    const Vec3 a_ref = (ref.has & kRefAcc) ? ref.a_ned : Vec3{0.f, 0.f, 0.f};
    // Yaw: a rate (kRefYawRate), else an absolute heading (kRefYaw), else free. With a rate the hold
    // target follows the heading while the pilot or the body turns and is captured on the first tick both
    // are still (and on entering fly or the rate reference), so a release while still turning brakes and
    // holds where the rotation stopped.
    float yaw = yaw_of(nav.q);
    bool yaw_rate_ref = false;
    bool follow_heading = false;
    if (ref.has & kRefYawRate) {
        yaw_rate_ref = std::fabs(ref.yaw_rate) > 0.f || std::fabs(nav.w_frd.z) > c_.yaw_rate_still;
        follow_heading = yaw_rate_ref || !have_hold_;
        if (follow_heading) yaw_hold_ = yaw;
        have_hold_ = !yaw_rate_ref;
        yaw = yaw_hold_;
    } else {
        have_hold_ = false;
        if (ref.has & kRefYaw) yaw = ref.yaw;
    }
    const Vec3 v_sp = clamp_norm(v_ref + c_.pos_p * (p_ref - nav.p_ned), c_.vel_max);
    const Vec3 ev = v_sp - nav.v_ned;
    const float iv_z_prev = iv_.z;
    iv_ += dt * ev;
    const Vec3 iv_h = clamp_norm({iv_.x, iv_.y, 0.f}, iv_max_);
    iv_ = {iv_h.x, iv_h.y, clampf(iv_.z, -iv_max_z_, iv_max_z_)};
    const Vec3 a_cmd = c_.vel_p * ev + c_.vel_i * iv_ + a_ref;

    // Thrust, vertical first: the horizontal part is cut to the tilt limit and to what the total leaves.
    Vec3 f = (hover_ / gravity_) * (a_cmd - Vec3{0.f, 0.f, gravity_});
    if ((f.z <= -c_.thrust_max_frac && ev.z <= 0.f) || (f.z >= -c_.thrust_min_g * hover_ && ev.z >= 0.f)) iv_.z = iv_z_prev;
    f.z = clampf(f.z, -c_.thrust_max_frac, -c_.thrust_min_g * hover_);
    const float fh = std::sqrt(f.x * f.x + f.y * f.y);
    const float fh_max = std::fmin(-f.z * tan_tilt_max_, std::sqrt(c_.thrust_max_frac * c_.thrust_max_frac - f.z * f.z));
    if (fh > fh_max) {
        f.x *= fh_max / fh;
        f.y *= fh_max / fh;
    }

    // Desired attitude: the reference yaw, then the tilt that turns body z (down) to -f.
    const Quat q_yaw = quat_from_euler(0.f, 0.f, yaw);
    const Vec3 z = rotate_inv(q_yaw, normalized(-f));
    const Vec3 axis = cross(Vec3{0.f, 0.f, 1.f}, z);
    const float tilt = std::atan2(norm(axis), z.z);
    const Quat q_des = q_yaw * quat_from_rotvec(tilt * normalized(axis));

    // Attitude target: advanced by its rate, then shaped toward the desired attitude.
    if (!have_target_) {
        q_t_ = nav.q;
        w_t_ = a_t_ = {0.f, 0.f, 0.f};
        have_target_ = true;
    }
    q_t_ = normalized(q_t_ * quat_from_rotvec(dt * w_t_));
    if (follow_heading) {
        q_t_ = q_yaw * (conj(quat_from_euler(0.f, 0.f, yaw_of(q_t_))) * q_t_);
        w_t_.z = a_t_.z = 0.f;
    }
    const Vec3 et = rotvec_from_quat(conj(q_t_) * q_des);
    command_model(et.x, w_t_.x, a_t_.x, c_.rate_max_x, c_.accel_max_x, c_.input_tc, dt);
    command_model(et.y, w_t_.y, a_t_.y, c_.rate_max_y, c_.accel_max_y, c_.input_tc, dt);
    command_model(et.z, w_t_.z, a_t_.z, c_.rate_max_z, c_.accel_max_z, c_.input_tc, dt);

    // Attitude P on the quaternion error to the target, in the body frame, plus the target's rate.
    const Quat q_bt = conj(nav.q) * q_t_;
    const Vec3 e = rotvec_from_quat(q_bt);
    const Vec3 ff = rotate(q_bt, w_t_);
    const float yaw_rate_max = std::fmin(c_.rate_max_z, std::sqrt(2.f * c_.yaw_decel * std::fabs(e.z)));
    Vec3 w_sp{clampf(c_.att_p_x * e.x + ff.x, -c_.rate_max_x, c_.rate_max_x),
              clampf(c_.att_p_y * e.y + ff.y, -c_.rate_max_y, c_.rate_max_y),
              clampf(clampf(c_.att_p_z * e.z, -yaw_rate_max, yaw_rate_max) + ff.z, -c_.rate_max_z, c_.rate_max_z)};
    if (yaw_rate_ref) w_sp.z = clampf(ref.yaw_rate, -c_.rate_max_z, c_.rate_max_z);

    // Hover thrust, learned in level hover only.
    const float roll = std::atan2(2.f * (nav.q.w * nav.q.x + nav.q.y * nav.q.z), 1.f - 2.f * (nav.q.x * nav.q.x + nav.q.y * nav.q.y));
    const float pitch = std::asin(clampf(2.f * (nav.q.w * nav.q.y - nav.q.z * nav.q.x), -1.f, 1.f));
    const bool climb_ref = (ref.has & kRefVel) && ref.v_ned.z != 0.f;
    if (!climb_ref && std::fabs(v_sp.z) < kHoverVsp && std::fabs(nav.v_ned.z) < kHoverClimb && std::fabs(roll) < kHoverLevel &&
        std::fabs(pitch) < kHoverLevel && -f.z > 0.f && dt > 0.f)
        hover_ = clampf(hover_ + (dt / (dt + kHoverTc)) * (-f.z - hover_), hover_min_, hover_max_);

    // Body-rate PID -> torque fraction.
    const Vec3 ew = w_sp - nav.w_frd;
    const float iw_z_prev = iw_.z;
    iw_ += dt * ew;
    iw_ = {clampf(iw_.x, -iw_max_.x, iw_max_.x), clampf(iw_.y, -iw_max_.y, iw_max_.y), clampf(iw_.z, -iw_max_.z, iw_max_.z)};
    const Vec3 dw = (have_prev_ && dt > 0.f) ? (1.f / dt) * (nav.w_frd - w_prev_) : Vec3{0.f, 0.f, 0.f};
    w_prev_ = nav.w_frd;
    have_prev_ = true;
    Vec3 tau{c_.rate_p_x * ew.x + c_.rate_i_x * iw_.x - c_.rate_d_x * dw.x,
             c_.rate_p_y * ew.y + c_.rate_i_y * iw_.y - c_.rate_d_y * dw.y,
             c_.rate_p_z * ew.z + c_.rate_i_z * iw_.z - c_.rate_d_z * dw.z};
    // Yaw anti-windup: while the yaw torque is at its limit the yaw integrator holds.
    if (std::fabs(tau.z) > c_.yaw_torque_max) {
        iw_.z = iw_z_prev;
        tau.z = clampf(tau.z, -c_.yaw_torque_max, c_.yaw_torque_max);
    }
    return {f, tau, hover_, 0.f};
}

}  // namespace marv
