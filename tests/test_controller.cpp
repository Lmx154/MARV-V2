// Controller yaw under a rate reference (kRefYawRate): while the pilot turns there is no yaw attitude
// error, a release while the body still turns brakes and holds the heading where the rotation stopped
// (not where the stick was released), and entering fly holds the heading it finds. kRefYaw is absolute.
#include <cmath>
#include <cstdio>

#include <marv/fsw/controller.hpp>
#include <marv/fsw/math.hpp>

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
    constexpr float kSmall = 0.01f;  // N m; the torque limit is 0.1

    {
        Controller c;
        CHECK(std::fabs(c.run(rate(0.f), at(0.f, 0.f), Mode::kFly, dt).torque_frd.z) < kSmall);  // holds 0
        // Turning at the commanded rate, a radian away from the heading held before: no pull back.
        CHECK(std::fabs(c.run(rate(0.5f), at(1.f, 0.5f), Mode::kFly, dt).torque_frd.z) < kSmall);
        // Stick released at 1.0 while the body turns at 0.5 rad/s: brake (negative torque), no heading target.
        CHECK(c.run(rate(0.f), at(1.2f, 0.5f), Mode::kFly, dt).torque_frd.z < -0.05f);
        CHECK(c.run(rate(0.f), at(1.4f, 0.3f), Mode::kFly, dt).torque_frd.z < -0.05f);
        // Below 0.1 rad/s the heading is captured: 1.5 is held, not 1.0.
        c.run(rate(0.f), at(1.5f, 0.05f), Mode::kFly, dt);
        CHECK(std::fabs(c.run(rate(0.f), at(1.5f, 0.f), Mode::kFly, dt).torque_frd.z) < kSmall);
        CHECK(c.run(rate(0.f), at(1.45f, 0.f), Mode::kFly, dt).torque_frd.z > 0.05f);
        CHECK(c.run(rate(0.f), at(1.55f, 0.f), Mode::kFly, dt).torque_frd.z < -0.05f);
    }
    {
        // Entering fly holds the heading found there.
        Controller c;
        c.run(rate(0.f), at(0.f, 0.f), Mode::kIdle, dt);
        CHECK(std::fabs(c.run(rate(0.f), at(2.f, 0.f), Mode::kFly, dt).torque_frd.z) < kSmall);
        CHECK(c.run(rate(0.f), at(1.9f, 0.f), Mode::kFly, dt).torque_frd.z > 0.05f);
    }
    {
        // kRefYaw stays absolute.
        Controller c;
        Reference ref = rate(0.f);
        ref.has = kRefPos | kRefYaw;
        ref.yaw = 0.f;
        CHECK(c.run(ref, at(0.5f, 0.f), Mode::kFly, dt).torque_frd.z < -0.05f);
    }

    if (failures) std::printf("%d failure(s)\n", failures);
    else std::printf("test_controller: all passed\n");
    return failures ? 1 : 0;
}
