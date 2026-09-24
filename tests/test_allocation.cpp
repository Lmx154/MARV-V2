// Allocation against the X3: the fixed-factor allocation's rotor thrusts, through the X3's own mixer rebuilt here from
// its old geometry (rotor positions, yaw torque per thrust, full rotor thrust), reproduce the requested collective and
// torque when nothing saturates, and equal the X3's inverse mixer wherever the factors can; saturation keeps the
// collective up to the request's hover thrust, then roll and pitch, then the rest of the collective, and gives up yaw
// first. And the actuators' thrust curve and spin range.
#include <cmath>
#include <cstdio>
#include <initializer_list>

#include <marv/fsw/actuators.hpp>
#include <marv/fsw/allocation.hpp>
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
#define NEAR(a, b, tol)                                                                                       \
    do {                                                                                                      \
        if (!(std::fabs((a) - (b)) <= (tol))) {                                                               \
            std::printf("FAIL %s:%d  %s = %.7g, want %.7g\n", __FILE__, __LINE__, #a, (double)(a), (double)(b)); \
            ++failures;                                                                                       \
        }                                                                                                     \
    } while (0)

// The X3 of sitl/gazebo/marv_quad.sdf as the old setup described it: rotor positions FRD (SDF:256, 321, 386, 451), yaw
// torque per thrust (SDF:524), spin +1 ccw / -1 cw (SDF:519/536/553), full rotor thrust k w_max^2 (SDF:522-523).
constexpr float kX[kMotorCount] = {0.13f, -0.13f, 0.13f, -0.13f};
constexpr float kY[kMotorCount] = {0.22f, -0.2f, -0.22f, 0.2f};
constexpr float kKm = 0.016f;
constexpr float kSpin[kMotorCount] = {1.f, 1.f, -1.f, -1.f};
constexpr float kTmax = 8.54858e-06f * 800.f * 800.f;
// Full authority per axis of the airframe decision (Q5), N m; checked against the geometry below.
constexpr float kTauMax[3] = {2.2979f, 1.4225f, 0.17508f};

// The X3's mixer: rotor thrusts, N -> (collective N, roll, pitch, yaw N m).
static void mixer(const float t[kMotorCount], float w[4]) {
    w[0] = w[1] = w[2] = w[3] = 0.f;
    for (int i = 0; i < kMotorCount; ++i) {
        w[0] += t[i];
        w[1] += -kY[i] * t[i];
        w[2] += kX[i] * t[i];
        w[3] += kKm * kSpin[i] * t[i];
    }
}

// The X3's inverse mixer, as the old allocation built it: Gauss-Jordan with partial pivoting on [mixer | identity].
struct Inverse {
    float m[kMotorCount][4];
    Inverse() {
        float a[4][2 * kMotorCount];
        for (int i = 0; i < kMotorCount; ++i) {
            a[0][i] = 1.f;
            a[1][i] = -kY[i];
            a[2][i] = kX[i];
            a[3][i] = kKm * kSpin[i];
            for (int j = 0; j < 4; ++j) a[j][kMotorCount + i] = j == i ? 1.f : 0.f;
        }
        for (int c = 0; c < 4; ++c) {
            int p = c;
            for (int r = c + 1; r < 4; ++r)
                if (std::fabs(a[r][c]) > std::fabs(a[p][c])) p = r;
            for (int k = 0; k < 2 * kMotorCount; ++k) {
                const float t = a[c][k];
                a[c][k] = a[p][k];
                a[p][k] = t;
            }
            const float d = 1.f / a[c][c];
            for (int k = 0; k < 2 * kMotorCount; ++k) a[c][k] *= d;
            for (int r = 0; r < 4; ++r) {
                if (r == c) continue;
                const float f = a[r][c];
                for (int k = 0; k < 2 * kMotorCount; ++k) a[r][k] -= f * a[c][k];
            }
        }
        for (int i = 0; i < kMotorCount; ++i)
            for (int j = 0; j < 4; ++j) m[i][j] = a[i][kMotorCount + j];
    }
};

// A request in fractions and the physical wrench it stands for.
struct Wrench {
    float t;
    Vec3 tau;
};
static Wrench physical(float collective, Vec3 n) {
    return {collective * kMotorCount * kTmax, {n.x * kTauMax[0], n.y * kTauMax[1], n.z * kTauMax[2]}};
}
static Wrench of(const ActuatorCommand& c) {
    float t[kMotorCount], w[4];
    for (int i = 0; i < kMotorCount; ++i) t[i] = c.motor[i] * kTmax;
    mixer(t, w);
    return {w[0], {w[1], w[2], w[3]}};
}

constexpr float kHover = 0.6811f;

static State level() {
    State s{};
    s.q = kQuatIdentity;
    s.valid = true;
    return s;
}

static ActuatorCommand run(float collective, Vec3 n, const State& nav, float hover = kHover) {
    Allocation a;
    const ControlRequest req{rotate(nav.q, Vec3{0.f, 0.f, -collective}), n, hover, 0.f};
    const ActuatorCommand c = a.run(req, nav, Mode::kFly);
    CHECK(c.armed && c.brake == 0.f);
    for (float m : c.motor) CHECK(m >= 0.f && m <= 1.f);
    return c;
}

int main() {
    // The authority of the factors on the X3: 0.5 T_max per rotor on each lever arm.
    {
        const float roll = 0.5f * kTmax * (std::fabs(kY[0]) + std::fabs(kY[1]) + std::fabs(kY[2]) + std::fabs(kY[3]));
        const float pitch = 0.5f * kTmax * 4.f * 0.13f;
        const float yaw = 0.5f * kTmax * 4.f * kKm;
        NEAR(roll, kTauMax[0], 1e-4f);
        NEAR(pitch, kTauMax[1], 1e-4f);
        NEAR(yaw, kTauMax[2], 1e-5f);
    }

    const Inverse inv;
    State tilted = level();
    tilted.q = quat_from_euler(0.2f, -0.1f, 1.0f);

    // Round trip, unsaturated, level and tilted (the collective is the thrust along -z body). Without yaw the factors'
    // rotor thrusts are the inverse mixer's; yaw also rolls the X3 by -0.02 T_max per unit, since its rear rotors sit
    // 0.02 m nearer the centre line than its front ones (the factors assume a symmetric X).
    const struct {
        float t;
        Vec3 n;
        const State nav;
    } cases[] = {
        {0.6809f, {0.0218f, -0.0562f, 0.f}, level()},
        {0.5483f, {-0.0870f, 0.0703f, 0.f}, level()},
        {0.6809f, {0.0218f, -0.0562f, 0.1713f}, level()},
        {0.5483f, {-0.0870f, 0.0703f, -0.2856f}, level()},
        {0.8225f, {0.f, 0.f, 0.2285f}, level()},
        {0.3656f, {0.0435f, 0.1054f, -0.1142f}, tilted},
    };
    for (const auto& k : cases) {
        const ActuatorCommand c = run(k.t, k.n, k.nav);
        const Wrench w = of(c), want = physical(k.t, k.n);
        NEAR(w.t, want.t, 1e-4f);
        NEAR(w.tau.x, want.tau.x - 0.02f * kTmax * k.n.z, 1e-4f);
        NEAR(w.tau.y, want.tau.y, 1e-4f);
        NEAR(w.tau.z, want.tau.z, 1e-5f);
        if (k.n.z == 0.f) {
            const float wr[4] = {want.t, want.tau.x, want.tau.y, want.tau.z};
            for (int i = 0; i < kMotorCount; ++i) {
                float t = 0.f;
                for (int j = 0; j < 4; ++j) t += inv.m[i][j] * wr[j];
                NEAR(c.motor[i] * kTmax, t, 1e-4f);
            }
        }
    }

    // Pitch plus a yaw demand the rotors cannot also carry: roll and pitch are kept, yaw is cut (same sign, smaller),
    // the collective does not grow.
    {
        const ActuatorCommand c = run(0.6809f, {0.f, 0.6327f, 0.5712f}, level());
        const Wrench w = of(c), want = physical(0.6809f, {0.f, 0.6327f, 0.5712f});
        const float yaw = w.tau.z / kTauMax[2];
        NEAR(w.tau.x + 0.02f * kTmax * yaw, 0.f, 1e-4f);
        NEAR(w.tau.y, want.tau.y, 1e-4f);
        CHECK(yaw > 0.f && yaw < 0.5712f - 1e-3f);
        CHECK(w.t <= want.t + 1e-4f);
    }
    // A collective above the rotors' limit: roll and pitch are kept, the collective gives way down to what they leave
    // free (still above the hover thrust), and yaw, last, is cut to nothing.
    {
        const ActuatorCommand c = run(0.9825f, {0.1306f, 0.1406f, -0.2856f}, level());
        const Wrench w = of(c), want = physical(0.9825f, {0.1306f, 0.1406f, -0.2856f});
        std::printf("0.9825 + (0.1306, 0.1406, -0.2856): collective %.4f, tau (%.4f %.4f %.4f) N m\n",
                    (double)(w.t / (kMotorCount * kTmax)), (double)w.tau.x, (double)w.tau.y, (double)w.tau.z);
        NEAR(w.tau.x, want.tau.x, 1e-4f);
        NEAR(w.tau.y, want.tau.y, 1e-4f);
        NEAR(w.tau.z, 0.f, 1e-5f);
        CHECK(w.t < want.t - 0.1f && w.t > kHover * kMotorCount * kTmax);
    }
    // Climb with a large pitch and yaw demand: the collective stays at or above the hover thrust, roll and pitch are
    // exact, yaw is cut.
    {
        const ActuatorCommand c = run(0.9002f, {-0.0870f, -0.4218f, 0.5712f}, level());
        const Wrench w = of(c), want = physical(0.9002f, {-0.0870f, -0.4218f, 0.5712f});
        const float yaw = w.tau.z / kTauMax[2];
        CHECK(w.t >= kHover * kMotorCount * kTmax - 1e-3f);
        NEAR(w.tau.x + 0.02f * kTmax * yaw, want.tau.x, 1e-4f);
        NEAR(w.tau.y, want.tau.y, 1e-4f);
        CHECK(yaw < 0.5712f);
    }
    // Yaw alone at a high collective: the collective is kept whole, yaw takes only the headroom above it.
    {
        const ActuatorCommand c = run(0.9002f, {0.f, 0.f, 0.5712f}, level());
        const Wrench w = of(c);
        NEAR(w.t, physical(0.9002f, {0.f, 0.f, 0.f}).t, 1e-4f);
        CHECK(w.tau.z / kTauMax[2] <= 0.2f);
    }
    // Hover with roll/pitch beyond the headroom: the collective holds, roll/pitch are scaled together.
    {
        const ActuatorCommand c = run(0.6809f, {0.4352f, 1.0545f, 0.f}, level());
        const Wrench w = of(c);
        NEAR(w.t, physical(0.6809f, {0.f, 0.f, 0.f}).t, 1e-4f);
        NEAR((w.tau.y / kTauMax[1]) / (w.tau.x / kTauMax[0]), 1.0545f / 0.4352f, 1e-3f);
        CHECK(w.tau.x > 0.f && w.tau.x / kTauMax[0] < 0.4352f);
    }
    // Roll/pitch alone beyond the rotors' spread: scaled down together, the direction kept.
    {
        const ActuatorCommand c = run(0.6809f, {1.3055f, 1.406f, 0.f}, level());
        const Wrench w = of(c);
        CHECK(w.tau.x > 0.f && w.tau.x / kTauMax[0] < 1.3055f);
        NEAR((w.tau.y / kTauMax[1]) / (w.tau.x / kTauMax[0]), 1.406f / 1.3055f, 1e-3f);
    }
    // The collective floor is the request's hover thrust, whatever it is: a climb with a roll demand past the headroom
    // keeps exactly the hover thrust, at 0.6811 and at 0.5.
    for (float hover : {kHover, 0.5f}) {
        const ActuatorCommand c = run(0.9f, {1.3f, 0.f, 0.f}, level(), hover);
        NEAR(of(c).t / (kMotorCount * kTmax), hover, 1e-5f);
    }

    // Idle: disarmed, every motor zero, the brake zero.
    {
        Allocation a;
        const ActuatorCommand c = a.run({{0.f, 0.f, -kHover}, {0.1f, 0.f, 0.f}, kHover, 0.5f}, level(), Mode::kIdle);
        CHECK(!c.armed && c.brake == 0.f);
        for (float m : c.motor) CHECK(m == 0.f);
    }

    // Actuators: the inverse of thrust = (1 - e) x + e x^2 mapped into [spin_min, spin_max]; disarmed untouched.
    {
        const float thrust[kMotorCount] = {0.f, 0.25f, 0.6811f, 1.f};
        ActuatorCommand c{};
        c.armed = true;
        for (int i = 0; i < kMotorCount; ++i) c.motor[i] = thrust[i];
        Actuators{}.run(c);  // factory: e = 1, spin 0..1: the rotor-speed fraction sqrt(thrust)
        for (int i = 0; i < kMotorCount; ++i) NEAR(c.motor[i], std::sqrt(thrust[i]), 1e-6f);

        param::ActuatorParams p{};
        p.thrust_expo = 0.f;
        c.armed = true;
        for (int i = 0; i < kMotorCount; ++i) c.motor[i] = thrust[i];
        Actuators{p}.run(c);  // linear
        for (int i = 0; i < kMotorCount; ++i) NEAR(c.motor[i], thrust[i], 1e-6f);

        p.thrust_expo = 0.65f;
        p.spin_min = 0.15f;
        p.spin_max = 0.95f;
        for (int i = 0; i < kMotorCount; ++i) c.motor[i] = thrust[i];
        Actuators{p}.run(c);
        NEAR(c.motor[0], 0.15f, 1e-6f);
        NEAR(c.motor[3], 0.95f, 1e-6f);
        for (int i = 0; i < kMotorCount; ++i) {
            const float x = (c.motor[i] - p.spin_min) / (p.spin_max - p.spin_min);
            NEAR((1.f - p.thrust_expo) * x + p.thrust_expo * x * x, thrust[i], 1e-5f);
        }

        ActuatorCommand idle{};
        Actuators{p}.run(idle);
        for (float m : idle.motor) CHECK(m == 0.f);
    }

    if (failures) std::printf("%d failure(s)\n", failures);
    else std::printf("test_allocation: all passed\n");
    return failures ? 1 : 0;
}
