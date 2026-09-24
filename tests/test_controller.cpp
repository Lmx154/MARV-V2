// Controller yaw under a rate reference (kRefYawRate): while the pilot turns there is no yaw attitude
// error, a release while the body still turns brakes and holds the heading where the rotation stopped
// (not where the stick was released), and entering fly holds the heading it finds. kRefYaw is absolute.
// The thrust is (hover / g)(a - g) in fractions of full thrust, and the hover thrust is learned against truth: a
// vertical point mass whose true hover thrust differs from the setup's is held at altitude and the learned value
// converges to the truth, only in level hover, within the parameter's range. The vertical velocity integrator may add
// up to 1 g and holds at a thrust limit; the horizontal one stops at vel_int_accel.
// Input shaping against truth: a rigid body flown with a tight rate loop follows the shaped attitude target within its
// rate and acceleration limits, and the target's rate is fed forward.
#include <cmath>
#include <cstdio>
#include <initializer_list>

#include <marv/fsw/controller.hpp>
#include <marv/fsw/math.hpp>
#include <marv/fsw/params.hpp>

using namespace marv;

static int failures = 0;
#define CHECK(c)                                                         \
    do {                                                                 \
        if (!(c)) {                                                      \
            std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c);     \
            ++failures;                                                  \
        }                                                                \
    } while (0)

static State at(float yaw, float yaw_rate) {
    return {0, {0.f, 0.f, -2.f}, {0.f, 0.f, 0.f}, quat_from_euler(0.f, 0.f, yaw), {0.f, 0.f, yaw_rate}, true};
}

static Reference rate(float r) {
    Reference ref{};
    ref.has = kRefPos | kRefYawRate;
    ref.p_ned = {0.f, 0.f, -2.f};
    ref.yaw_rate = r;
    ref.q = {1.f, 0.f, 0.f, 0.f};
    return ref;
}

struct Shaped {
    Vec3 w_peak{0.f, 0.f, 0.f};  // peak |body rate|, rad/s
    Vec3 a_peak{0.f, 0.f, 0.f};  // peak |body angular acceleration|, rad/s^2
    float yaw_over = 0.f;        // most heading past the reference, rad
    float t_out = 0.f;           // last time the heading was more than 1 deg from the reference, s
};

// A rigid body whose angular acceleration is 200 rad/s^2 per unit of torque fraction on every axis, its attitude
// integrated from its body rate, the rest of its state held; flown at 1 kHz for the given time.
static Shaped fly_attitude(const param::ControllerParams& p, const Reference& ref, State x, float seconds) {
    constexpr float h = 0.001f;
    Controller c(p);
    Shaped r;
    for (int i = 1; i * h <= seconds; ++i) {
        const Vec3 a = 200.f * c.run(ref, x, Mode::kFly, h).torque_frd;
        x.w_frd += h * a;
        x.q = normalized(x.q * quat_from_rotvec(h * x.w_frd));
        r.w_peak = {std::fmax(r.w_peak.x, std::fabs(x.w_frd.x)), std::fmax(r.w_peak.y, std::fabs(x.w_frd.y)),
                    std::fmax(r.w_peak.z, std::fabs(x.w_frd.z))};
        r.a_peak = {std::fmax(r.a_peak.x, std::fabs(a.x)), std::fmax(r.a_peak.y, std::fabs(a.y)), std::fmax(r.a_peak.z, std::fabs(a.z))};
        const float d = yaw_of(x.q) - ref.yaw;
        const float ey = std::atan2(std::sin(d), std::cos(d));
        r.yaw_over = std::fmax(r.yaw_over, ey);
        if (std::fabs(ey) > 1.f * 3.14159265f / 180.f) r.t_out = i * h;
    }
    return r;
}

int main() {
    constexpr float dt = 0.004f;
    const float kSmall = param::ControllerParams{}.rate_p_z * 0.01f;  // the yaw torque of 0.01 rad/s of rate error
    const float kClear = param::ControllerParams{}.rate_p_z * 0.1f;   // the yaw torque of 0.1 rad/s of rate error

    {
        Controller c;
        CHECK(std::fabs(c.run(rate(0.f), at(0.f, 0.f), Mode::kFly, dt).torque_frd.z) < kSmall);  // holds 0
        // Turning at the commanded rate, a radian away from the heading held before: no pull back.
        CHECK(std::fabs(c.run(rate(0.5f), at(1.f, 0.5f), Mode::kFly, dt).torque_frd.z) < kSmall);
        // Stick released at 1.0 while the body turns at 0.5 rad/s: brake (negative torque), no heading target.
        CHECK(c.run(rate(0.f), at(1.2f, 0.5f), Mode::kFly, dt).torque_frd.z < -kClear);
        CHECK(c.run(rate(0.f), at(1.4f, 0.3f), Mode::kFly, dt).torque_frd.z < -kClear);
        // Below 0.1 rad/s the heading is captured: 1.5 is held, not 1.0.
        c.run(rate(0.f), at(1.5f, 0.05f), Mode::kFly, dt);
        CHECK(std::fabs(c.run(rate(0.f), at(1.5f, 0.f), Mode::kFly, dt).torque_frd.z) < kSmall);
        CHECK(c.run(rate(0.f), at(1.45f, 0.f), Mode::kFly, dt).torque_frd.z > kClear);
        CHECK(c.run(rate(0.f), at(1.55f, 0.f), Mode::kFly, dt).torque_frd.z < -kClear);
    }
    {
        // Entering fly holds the heading found there.
        Controller c;
        c.run(rate(0.f), at(0.f, 0.f), Mode::kIdle, dt);
        CHECK(std::fabs(c.run(rate(0.f), at(2.f, 0.f), Mode::kFly, dt).torque_frd.z) < kSmall);
        CHECK(c.run(rate(0.f), at(1.9f, 0.f), Mode::kFly, dt).torque_frd.z > kClear);
    }
    {
        // kRefYaw stays absolute. The attitude target starts at the body: no step on the first tick, a clear turn toward
        // the reference within 0.1 s.
        Controller c;
        Reference ref = rate(0.f);
        ref.has = kRefPos | kRefYaw;
        ref.yaw = 0.f;
        CHECK(std::fabs(c.run(ref, at(0.5f, 0.f), Mode::kFly, dt).torque_frd.z) < kSmall);
        float tz = 0.f;
        for (int i = 0; i < 25; ++i) tz = c.run(ref, at(0.5f, 0.f), Mode::kFly, dt).torque_frd.z;
        CHECK(tz < -kClear);
    }

    // Shaping, on the rigid body of fly_attitude flown with rate P 1 and no rate I or D (a 200 rad/s rate loop), still
    // at the position reference. A 90 deg heading step: the body's yaw rate stays within rate_max_z and its yaw
    // acceleration within accel_max_z (the unshaped loop asks 1.5 rad/s at once: 300 rad/s^2 on this body); it arrives
    // within 2 s without overshoot. With the attitude gains 0 it still arrives: the target's rate is fed forward. A
    // 1 m/s lateral velocity error (11.5 deg of roll asked at once) rolls it within accel_max_x and rate_max_x.
    {
        param::ControllerParams p{};
        p.rate_p_x = p.rate_p_y = p.rate_p_z = 1.f;
        p.rate_i_x = p.rate_i_y = p.rate_i_z = 0.f;
        p.rate_d_x = p.rate_d_y = p.rate_d_z = 0.f;
        Reference ref = rate(0.f);
        ref.has = kRefPos | kRefYaw;
        ref.yaw = 1.5707963f;
        const Shaped step = fly_attitude(p, ref, at(0.f, 0.f), 3.f);
        std::printf("90 deg heading step: peak yaw rate %.3f rad/s, peak yaw acceleration %.3f rad/s^2, overshoot %.3f deg, "
                    "within 1 deg after %.3f s\n",
                    (double)step.w_peak.z, (double)step.a_peak.z, (double)(step.yaw_over * 57.29578f), (double)step.t_out);
        CHECK(step.w_peak.z <= p.rate_max_z * 1.01f);
        CHECK(step.a_peak.z <= p.accel_max_z * 1.1f);
        CHECK(step.yaw_over < 1.f * 3.14159265f / 180.f);
        CHECK(step.t_out < 2.f);
        CHECK(step.w_peak.x < 1e-3f && step.w_peak.y < 1e-3f);

        param::ControllerParams ff = p;
        ff.att_p_x = ff.att_p_y = ff.att_p_z = 0.f;
        const Shaped fed = fly_attitude(ff, ref, at(0.f, 0.f), 3.f);
        std::printf("the same with the attitude gains 0: overshoot %.3f deg, within 1 deg after %.3f s\n",
                    (double)(fed.yaw_over * 57.29578f), (double)fed.t_out);
        CHECK(fed.yaw_over < 1.f * 3.14159265f / 180.f);
        CHECK(fed.t_out < 2.f);

        ref.yaw = 0.f;
        State sliding = at(0.f, 0.f);
        sliding.v_ned = {0.f, 1.f, 0.f};
        const Shaped roll = fly_attitude(p, ref, sliding, 1.f);
        std::printf("lateral velocity error: peak roll rate %.3f rad/s, peak roll acceleration %.3f rad/s^2\n",
                    (double)roll.w_peak.x, (double)roll.a_peak.x);
        CHECK(roll.w_peak.x > 0.1f && roll.w_peak.x <= p.rate_max_x * 1.01f);
        CHECK(roll.a_peak.x <= p.accel_max_x * 1.1f);
    }

    // Thrust at the seeded hover: level at the reference it is the hover thrust straight up; a velocity error adds
    // (hover / g) of the acceleration the velocity loop asks for.
    {
        const param::UavParams v{};
        const param::SensorParams s{};
        const param::ControllerParams p{};
        Controller c;
        const ControlRequest r = c.run(rate(0.f), at(0.f, 0.f), Mode::kFly, dt);
        CHECK(r.thrust_ned.x == 0.f && r.thrust_ned.y == 0.f && std::fabs(r.thrust_ned.z + v.hover_thrust) < 1e-6f);
        CHECK(r.brake == 0.f && std::fabs(r.thrust_hover - v.hover_thrust) < 1e-6f);
        Controller d;
        State moving = at(0.f, 0.f);
        moving.v_ned = {0.5f, 0.f, 0.f};
        const float ax = p.vel_p * -0.5f + p.vel_i * (dt * -0.5f);
        CHECK(std::fabs(d.run(rate(0.f), moving, Mode::kFly, dt).thrust_ned.x - v.hover_thrust / s.gravity * ax) < 1e-6f);
    }

    // Truth: a level vertical point mass, thrust T (fraction) giving a_down = g (1 - T / h_true), held at 2 m for 120 s.
    // The learned hover thrust converges to h_true, 0.06 either side of the seed and the X3's 0.6811 (3.9 m/s^2 of
    // vertical integrator at the seed 0.5), and the altitude holds.
    for (float h_true : {param::UavParams{}.hover_thrust - 0.06f, param::UavParams{}.hover_thrust + 0.06f, 0.6811f}) {
        const float g = param::SensorParams{}.gravity;
        Controller c;
        State x = at(0.f, 0.f);
        float hover = 0.f;
        for (int i = 0; i < 30000; ++i) {
            const ControlRequest r = c.run(rate(0.f), x, Mode::kFly, dt);
            hover = r.thrust_hover;
            const float a = g * (1.f + r.thrust_ned.z / h_true);
            x.v_ned.z += a * dt;
            x.p_ned.z += x.v_ned.z * dt;
        }
        std::printf("true hover %.4f: learned %.4f, altitude %.4f m\n", (double)h_true, (double)hover, (double)-x.p_ned.z);
        CHECK(std::fabs(hover - h_true) < 0.005f);
        CHECK(std::fabs(x.p_ned.z + 2.f) < 0.05f);
    }

    // The velocity integrator's limits. Seeded at 0.2, tilted 6 deg (nothing learned), at the reference and moving at
    // 1 m/s north and down for 60 s: the vertical integrator stops at 1 g, the horizontal one at vel_int_accel.
    {
        const float g = param::SensorParams{}.gravity;
        const param::ControllerParams p{};
        param::UavParams seeded{};
        seeded.hover_thrust = 0.2f;
        Controller c({}, seeded);
        State x = at(0.f, 0.f);
        x.q = quat_from_euler(6.f * 3.14159265f / 180.f, 0.f, 0.f);
        x.v_ned = {1.f, 0.f, 1.f};
        ControlRequest r{};
        for (int i = 0; i < 15000; ++i) r = c.run(rate(0.f), x, Mode::kFly, dt);
        CHECK(std::fabs(r.thrust_ned.z - 0.2f / g * (-p.vel_p - g - g)) < 1e-4f);
        CHECK(std::fabs(r.thrust_ned.x - 0.2f / g * (-p.vel_p - p.vel_int_accel)) < 1e-4f);
    }

    // At a thrust limit the vertical integrator holds (PX4 PositionControl.cpp:157-162). A point mass that cannot lift
    // (true hover thrust 0.95, above the 0.9 limit) sits on the ground 2 m below the reference for 20 s; put at the
    // reference, still, it asks less than the limit (it wound only until the thrust saturated, to about 5 m/s^2; wound
    // to 1 g it would ask the seed's 2 hover thrusts, past the limit).
    {
        const float g = param::SensorParams{}.gravity;
        Controller c;
        State x = at(0.f, 0.f);
        x.p_ned.z = 0.f;
        for (int i = 0; i < 5000; ++i) {
            const ControlRequest r = c.run(rate(0.f), x, Mode::kFly, dt);
            x.v_ned.z = std::fmin(0.f, x.v_ned.z + g * (1.f + r.thrust_ned.z / 0.95f) * dt);
            x.p_ned.z = std::fmin(0.f, x.p_ned.z + x.v_ned.z * dt);
        }
        CHECK(x.p_ned.z == 0.f);
        const float fz = c.run(rate(0.f), at(0.f, 0.f), Mode::kFly, dt).thrust_ned.z;
        std::printf("after 20 s on the ground at the thrust limit, at the reference: vertical thrust %.4f\n", (double)fz);
        CHECK(fz > -0.85f);
    }

    // The gates: tilted past 5 deg, climbing at 0.6 m/s or more, a vertical velocity reference, a position error that
    // asks the velocity loop for 0.1 m/s or more, or idle: nothing learned. Level, still and 0.12 m below the reference
    // (0.084 m/s asked): learned (toward more thrust).
    {
        const float seed = param::UavParams{}.hover_thrust;
        const auto learned = [&](State x, Reference ref, Mode mode) {
            Controller c;
            float h = 0.f;
            for (int i = 0; i < 1250; ++i) h = c.run(ref, x, mode, dt).thrust_hover;
            return h;
        };
        State low = at(0.f, 0.f);
        low.p_ned.z = -1.88f;
        State far = low;
        far.p_ned.z = -1.f;
        State tilted = low;
        tilted.q = quat_from_euler(6.f * 3.14159265f / 180.f, 0.f, 0.f);
        State pitched = low;
        pitched.q = quat_from_euler(0.f, -6.f * 3.14159265f / 180.f, 0.f);
        State climbing = low;
        climbing.v_ned.z = -0.6f;
        Reference climb = rate(0.f);
        climb.has |= kRefVel;
        climb.v_ned = {0.f, 0.f, -0.5f};
        CHECK(learned(tilted, rate(0.f), Mode::kFly) == seed);
        CHECK(learned(pitched, rate(0.f), Mode::kFly) == seed);
        CHECK(learned(climbing, rate(0.f), Mode::kFly) == seed);
        CHECK(learned(low, climb, Mode::kFly) == seed);
        CHECK(learned(far, rate(0.f), Mode::kFly) == seed);
        CHECK(learned(low, rate(0.f), Mode::kIdle) == seed);
        CHECK(learned(low, rate(0.f), Mode::kArmed) == seed);
        CHECK(learned(low, rate(0.f), Mode::kFly) > seed + 0.002f);

        // kArmed behaves as kIdle: a zero wrench at the hover thrust, and the integrators reset, so the first kFly tick
        // after it matches a fresh controller's (the flight before it at 1 m error learns nothing but winds them).
        Controller wound;
        for (int i = 0; i < 1250; ++i) wound.run(rate(0.f), far, Mode::kFly, dt);
        const ControlRequest z = wound.run(rate(0.f), far, Mode::kArmed, dt);
        CHECK(z.thrust_ned.x == 0.f && z.thrust_ned.y == 0.f && z.thrust_ned.z == 0.f);
        CHECK(z.torque_frd.x == 0.f && z.torque_frd.y == 0.f && z.torque_frd.z == 0.f);
        CHECK(z.thrust_hover == seed && z.brake == 0.f);
        const ControlRequest after = wound.run(rate(0.f), far, Mode::kFly, dt);
        Controller fresh;
        const ControlRequest first = fresh.run(rate(0.f), far, Mode::kFly, dt);
        CHECK(after.thrust_ned.x == first.thrust_ned.x && after.thrust_ned.y == first.thrust_ned.y &&
              after.thrust_ned.z == first.thrust_ned.z);
        CHECK(after.torque_frd.x == first.torque_frd.x && after.torque_frd.y == first.torque_frd.y &&
              after.torque_frd.z == first.torque_frd.z);
    }

    // A climb from a position step (2 m below the reference, true hover thrust the seed's): the take-off does not
    // move the learned value more than 0.002 from the truth.
    {
        const float g = param::SensorParams{}.gravity;
        const float h_true = param::UavParams{}.hover_thrust;
        Controller c;
        State x = at(0.f, 0.f);
        x.p_ned.z = 0.f;
        float above = 0.f, below = 0.f;
        for (int i = 0; i < 5000; ++i) {
            const ControlRequest r = c.run(rate(0.f), x, Mode::kFly, dt);
            above = std::fmax(above, r.thrust_hover - h_true);
            below = std::fmax(below, h_true - r.thrust_hover);
            x.v_ned.z += g * (1.f + r.thrust_ned.z / h_true) * dt;
            x.p_ned.z += x.v_ned.z * dt;
        }
        std::printf("climb from a 2 m position step: learned hover at most %.5f above, %.5f below the truth\n",
                    (double)above, (double)below);
        CHECK(above <= 0.002f);
    }

    // Clamped to the parameter's range: a point mass whose true hover thrust is 0.81, held at the reference for 200 s
    // from a setup seeded at 0.6811, stops the learned value at the maximum, 0.8.
    {
        const float g = param::SensorParams{}.gravity;
        param::UavParams seeded{};
        seeded.hover_thrust = 0.6811f;
        Controller c({}, seeded);
        State x = at(0.f, 0.f);
        float h = 0.f;
        for (int i = 0; i < 50000; ++i) {
            const ControlRequest r = c.run(rate(0.f), x, Mode::kFly, dt);
            h = r.thrust_hover;
            x.v_ned.z += g * (1.f + r.thrust_ned.z / 0.81f) * dt;
            x.p_ned.z += x.v_ned.z * dt;
        }
        std::printf("true hover 0.81: learned %.4f, altitude %.4f m\n", (double)h, (double)-x.p_ned.z);
        CHECK(h == param::kParamMeta[param::k_vehicle_uav_hover_thrust].max);
    }

    if (failures) std::printf("%d failure(s)\n", failures);
    else std::printf("test_controller: all passed\n");
    return failures ? 1 : 0;
}
