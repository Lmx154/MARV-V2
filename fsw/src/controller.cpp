#include <marv/fsw/controller.hpp>

#include <cmath>

#include <marv/fsw/math.hpp>
#include <marv/fsw/vehicle.hpp>

namespace marv {
namespace {

using namespace vehicle;

// Gains. Every loop is written as an acceleration command, so the airframe enters only through
// F = m a and tau = I alpha, and each gain is a bandwidth (1/s) or a bandwidth squared.
//
// Rate loop, alpha = Kp e + Ki int(e) - Kd dw/dt. With w' = alpha the closed loop is first order at
// Kp / (1 + Kd) rad/s. What limits it is the motor lag, 12.5 ms up / 25 ms down (SDF timeConstantUp /
// Down), i.e. poles at 40..80 rad/s, plus about 5 ms of the odometry's 10-sample rolling mean. Roll and
// pitch at 25 rad/s keep about 45 deg of phase margin against those; the integrator corner sits a
// decade lower (Ki = Kp * Kp / 10). Yaw at 10 rad/s: its authority is small (see kYawTorqueMax), so
// more bandwidth only saturates.
constexpr Vec3 kRateP{25.f, 25.f, 10.f};                  // 1/s
constexpr Vec3 kRateI{62.5f, 62.5f, 10.f};                // 1/s^2
constexpr Vec3 kRateD{0.05f, 0.05f, 0.f};                 // dimensionless
constexpr float kRateIntAccel = 5.f;                      // rad/s^2, most the rate integrator may add
constexpr Vec3 kRateMax{3.f, 3.f, 1.5f};                  // rad/s, rate setpoint limit
// Attitude loop, rate_sp = K e (e the rotation vector to the desired attitude): a third of the rate
// bandwidth on roll and pitch. On yaw the rate setpoint is also capped at sqrt(2 a |e|), the speed from
// which a deceleration of a stops exactly at the target, so a large yaw step comes in without the
// overshoot a saturated linear loop would give; a is 70 % of the torque-limited 1.02 rad/s^2 (0.93
// rad/s^2 measured in the sim at that torque).
constexpr Vec3 kAttP{8.f, 8.f, 4.f};                      // 1/s
constexpr float kYawDecel = 0.7f;                         // rad/s^2
// Yaw torque limit. At hover each rotor carries m g / 4 = 3.73 N and has 1.74 N left to kMaxRotorThrust;
// the most yaw torque that fits in that headroom is 4 km 1.74 = 0.111 N m, and 0.1 N m keeps 10 % of it
// for roll and pitch. The allocation's priority is collective up to m g, then roll/pitch, then the rest
// of the collective, then yaw: a yaw demand the rotors cannot carry is cut, never paid for with thrust.
constexpr float kYawTorqueMax = 0.1f;                     // N m
// Translation, v_sp = Kp e_p, a = Kv (v_sp - v) + Ki int(v_sp - v). With the inner loops ideal the
// position error obeys e'' + Kv e' + Kv Kp e = 0: w_n = sqrt(Kv Kp) = 1.18 rad/s, about 7x below the
// attitude loop, and zeta = sqrt(Kv / Kp) / 2 = 0.85. Kv * kVelMax = 4 m/s^2 bounds the acceleration a
// velocity step asks for to atan(4 / g) = 22 deg of tilt. The integrator (corner Ki / Kv = 0.25 rad/s)
// takes up what the thrust model and the rotor drag leave.
constexpr float kPosP = 0.7f;                             // 1/s
constexpr float kVelP = 2.0f;                             // 1/s
constexpr float kVelI = 0.5f;                             // 1/s^2
constexpr float kVelIntAccel = 2.f;                       // m/s^2, most the velocity integrator may add
constexpr float kVelMax = 2.f;                            // m/s, norm of the velocity setpoint
// Force limits: tilt of the thrust vector within 35 deg of vertical, vertical force between 0.2 g and a
// total of 90 % of the four rotors' full thrust (the rest is left for the torques).
constexpr float kTanTiltMax = 0.70020754f;                // tan(35 deg)
constexpr float kThrustMin = 0.2f * kMass * kGravity;     // N
constexpr float kThrustMax = 0.9f * kMotorCount * kMaxRotorThrust;  // N

float clampf(float v, float lo, float hi) { return std::fmin(std::fmax(v, lo), hi); }

Vec3 clamp_norm(Vec3 v, float max) {
    const float n = norm(v);
    return n > max ? (max / n) * v : v;
}

}  // namespace

ControlRequest Controller::run(const Reference& ref, const State& nav, Mode mode, float dt) {
    if (mode != Mode::kFly) {
        iv_ = iw_ = w_prev_ = {0.f, 0.f, 0.f};
        have_prev_ = false;
        return {{0.f, 0.f, 0.f}, {0.f, 0.f, 0.f}};
    }

    // Translation: position P -> velocity PI -> acceleration.
    const Vec3 p_ref = (ref.has & kRefPos) ? ref.p_ned : nav.p_ned;
    const Vec3 v_ref = (ref.has & kRefVel) ? ref.v_ned : Vec3{0.f, 0.f, 0.f};
    const Vec3 a_ref = (ref.has & kRefAcc) ? ref.a_ned : Vec3{0.f, 0.f, 0.f};
    const float yaw = (ref.has & kRefYaw) ? ref.yaw : yaw_of(nav.q);
    const Vec3 v_sp = clamp_norm(v_ref + kPosP * (p_ref - nav.p_ned), kVelMax);
    const Vec3 ev = v_sp - nav.v_ned;
    iv_ = clamp_norm(iv_ + dt * ev, kVelIntAccel / kVelI);
    const Vec3 a_cmd = kVelP * ev + kVelI * iv_ + a_ref;

    // Force, vertical first: the horizontal part is cut to the tilt limit and to what the total leaves.
    Vec3 f = kMass * (a_cmd - Vec3{0.f, 0.f, kGravity});
    f.z = clampf(f.z, -kThrustMax, -kThrustMin);
    const float fh = std::sqrt(f.x * f.x + f.y * f.y);
    const float fh_max = std::fmin(-f.z * kTanTiltMax, std::sqrt(kThrustMax * kThrustMax - f.z * f.z));
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

    // Attitude P on the quaternion error, in the body frame.
    const Vec3 e = rotvec_from_quat(conj(nav.q) * q_des);
    Vec3 w_sp{kAttP.x * e.x, kAttP.y * e.y, kAttP.z * e.z};
    const float yaw_rate_max = std::fmin(kRateMax.z, std::sqrt(2.f * kYawDecel * std::fabs(e.z)));
    w_sp = {clampf(w_sp.x, -kRateMax.x, kRateMax.x), clampf(w_sp.y, -kRateMax.y, kRateMax.y),
            clampf(w_sp.z, -yaw_rate_max, yaw_rate_max)};

    // Body-rate PID -> angular acceleration -> torque.
    const Vec3 ew = w_sp - nav.w_frd;
    const float iw_z_prev = iw_.z;
    iw_ += dt * ew;
    iw_ = {clampf(iw_.x, -kRateIntAccel / kRateI.x, kRateIntAccel / kRateI.x),
           clampf(iw_.y, -kRateIntAccel / kRateI.y, kRateIntAccel / kRateI.y),
           clampf(iw_.z, -kRateIntAccel / kRateI.z, kRateIntAccel / kRateI.z)};
    const Vec3 dw = (have_prev_ && dt > 0.f) ? (1.f / dt) * (nav.w_frd - w_prev_) : Vec3{0.f, 0.f, 0.f};
    w_prev_ = nav.w_frd;
    have_prev_ = true;
    const Vec3 alpha{kRateP.x * ew.x + kRateI.x * iw_.x - kRateD.x * dw.x,
                     kRateP.y * ew.y + kRateI.y * iw_.y - kRateD.y * dw.y,
                     kRateP.z * ew.z + kRateI.z * iw_.z - kRateD.z * dw.z};
    const Vec3 w = nav.w_frd;
    const Vec3 iw{kIxx * w.x, kIyy * w.y, kIzz * w.z};
    Vec3 tau = Vec3{kIxx * alpha.x, kIyy * alpha.y, kIzz * alpha.z} + cross(w, iw);
    // Yaw anti-windup: while the yaw torque is at its limit the yaw integrator holds.
    if (std::fabs(tau.z) > kYawTorqueMax) {
        iw_.z = iw_z_prev;
        tau.z = clampf(tau.z, -kYawTorqueMax, kYawTorqueMax);
    }
    return {f, tau};
}

}  // namespace marv
