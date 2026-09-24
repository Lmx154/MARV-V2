// Allocation against the mixer: the rotor thrusts it commands reproduce the requested collective and
// torque when nothing saturates, and saturation gives up yaw and collective before roll and pitch.
#include <cmath>
#include <cstdio>

#include <marv/fsw/allocation.hpp>
#include <marv/fsw/math.hpp>
#include <marv/fsw/vehicle.hpp>

using namespace marv;
using namespace marv::vehicle;

static int failures = 0;
#define CHECK(c)                                                         \
    do {                                                                 \
        if (!(c)) {                                                      \
            std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c);     \
            ++failures;                                                  \
        }                                                                \
    } while (0)
#define NEAR(a, b, tol)                                                                                       \
    do {                                                                                                      \
        if (!(std::fabs((a) - (b)) <= (tol))) {                                                               \
            std::printf("FAIL %s:%d  %s = %.7g, want %.7g\n", __FILE__, __LINE__, #a, (double)(a), (double)(b)); \
            ++failures;                                                                                       \
        }                                                                                                     \
    } while (0)

// The mixer: motor fractions -> rotor thrusts -> (collective, torque about FRD).
struct Wrench {
    float t;
    Vec3 tau;
};
static Wrench mix(const ActuatorCommand& c) {
    Wrench w{0.f, {0.f, 0.f, 0.f}};
    for (int i = 0; i < kMotorCount; ++i) {
        const float omega = c.motor[i] * kMaxRotVelocity;
        const float t = kMotorConstant * omega * omega;
        w.t += t;
        w.tau += Vec3{-kRotorY[i] * t, kRotorX[i] * t, kMomentConstant * kRotorYaw[i] * t};
    }
    return w;
}

static State level() {
    State s{};
    s.q = kQuatIdentity;
    s.valid = true;
    return s;
}

static ActuatorCommand run(Allocation& a, float collective, Vec3 tau, const State& nav) {
    const ControlRequest req{rotate(nav.q, Vec3{0.f, 0.f, -collective}), tau};
    const ActuatorCommand c = a.run(req, nav, Mode::kFly);
    CHECK(c.armed);
    for (float m : c.motor) CHECK(m >= 0.f && m <= 1.f);
    return c;
}

int main() {
    Allocation a;

    // Round trip, unsaturated: level, and tilted (the collective is the force along -z body).
    State tilted = level();
    tilted.q = quat_from_euler(0.2f, -0.1f, 1.0f);
    const struct {
        float t;
        Vec3 tau;
        const State nav;
    } cases[] = {
        {14.9f, {0.05f, -0.08f, 0.03f}, level()},
        {12.f, {-0.2f, 0.1f, -0.05f}, level()},
        {18.f, {0.f, 0.f, 0.04f}, level()},
        {8.f, {0.1f, 0.15f, -0.02f}, tilted},
    };
    for (const auto& k : cases) {
        const Wrench w = mix(run(a, k.t, k.tau, k.nav));
        NEAR(w.t, k.t, 1e-4f);
        NEAR(w.tau.x, k.tau.x, 1e-4f);
        NEAR(w.tau.y, k.tau.y, 1e-4f);
        NEAR(w.tau.z, k.tau.z, 1e-4f);
    }

    // Pitch plus a yaw demand the rotors cannot also carry (spread 6.6 N > 5.47 N): roll and pitch
    // are kept, yaw is cut (same sign, smaller), the collective does not grow.
    {
        const Wrench w = mix(run(a, 14.9f, {0.f, 0.9f, 0.1f}, level()));
        NEAR(w.tau.x, 0.f, 1e-4f);
        NEAR(w.tau.y, 0.9f, 1e-4f);
        CHECK(w.tau.z > 0.f && w.tau.z < 0.1f - 1e-3f);
        CHECK(w.t <= 14.9f + 1e-4f);
    }
    // A collective above the rotors' limit: the torques are kept and the collective gives way.
    {
        const Wrench w = mix(run(a, 21.5f, {0.3f, 0.2f, -0.05f}, level()));
        NEAR(w.tau.x, 0.3f, 1e-4f);
        NEAR(w.tau.y, 0.2f, 1e-4f);
        NEAR(w.tau.z, -0.05f, 1e-4f);
        CHECK(w.t < 21.5f - 0.1f);
    }
    // Roll/pitch alone beyond the rotors' spread: scaled down together, the direction kept.
    {
        const Wrench w = mix(run(a, 14.9f, {3.f, 2.f, 0.f}, level()));
        CHECK(w.tau.x > 0.f && w.tau.x < 3.f);
        NEAR(w.tau.y / w.tau.x, 2.f / 3.f, 1e-4f);
    }

    // Idle: disarmed, every motor zero.
    {
        const ActuatorCommand c = a.run({{0.f, 0.f, -14.9f}, {0.1f, 0.f, 0.f}}, level(), Mode::kIdle);
        CHECK(!c.armed);
        for (float m : c.motor) CHECK(m == 0.f);
    }

    if (failures) std::printf("%d failure(s)\n", failures);
    else std::printf("test_allocation: all passed\n");
    return failures ? 1 : 0;
}
