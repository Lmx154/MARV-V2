// Controller yaw under a rate reference (kRefYawRate): while the pilot turns there is no yaw attitude
// error, a release while the body still turns brakes and holds the heading where the rotation stopped
// (not where the stick was released), and entering fly holds the heading it finds. kRefYaw is absolute.
// The thrust is (hover / g)(a - g) in fractions of full thrust, and the hover thrust is learned against truth: a
// vertical point mass whose true hover thrust differs from the setup's is held at altitude and the learned value
// converges to the truth, only in level hover, within the parameter's range.
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

int main() {
    constexpr float dt = 0.004f;
    constexpr float kYawMax = 0.17508f;       // N m, full yaw authority of the X3 (params.def)
    constexpr float kSmall = 0.01f / kYawMax;  // 0.01 N m; the torque limit is 0.1 N m
    constexpr float kClear = 0.05f / kYawMax;  // 0.05 N m

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
        // kRefYaw stays absolute.
        Controller c;
        Reference ref = rate(0.f);
        ref.has = kRefPos | kRefYaw;
        ref.yaw = 0.f;
        CHECK(c.run(ref, at(0.5f, 0.f), Mode::kFly, dt).torque_frd.z < -kClear);
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

    // Truth: a level vertical point mass, thrust T (fraction) giving a_down = g (1 - T / h_true), held at 2 m for 60 s.
    // The learned hover thrust converges to h_true from the seed 0.6811, and the altitude holds.
    for (float h_true : {0.62f, 0.74f}) {
        const float g = param::SensorParams{}.gravity;
        Controller c;
        State x = at(0.f, 0.f);
        float hover = 0.f;
        for (int i = 0; i < 15000; ++i) {
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
        CHECK(learned(low, rate(0.f), Mode::kFly) > seed + 0.002f);
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

    // Clamped to the parameter's range: a point mass whose true hover thrust is 0.81, held at the reference for 200 s,
    // stops the learned value at the maximum, 0.8.
    {
        const float g = param::SensorParams{}.gravity;
        Controller c;
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
