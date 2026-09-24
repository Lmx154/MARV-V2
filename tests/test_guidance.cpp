// Trajectory guidance (ADR-0011) and the PX4 smoothing it is built on.
// 1. PX4@af2e7b43's VelocitySmoothingTest.cpp and PositionSmoothingTest.cpp cases on the port, with their own assertions,
//    and as fixtures the values PX4's code gives on the same cases (VelocitySmoothing.cpp and PositionSmoothing.cpp
//    compiled as they are, g++ x86-64 -O0): the phase durations, the states at iterations 50 and 200 and the
//    iteration each case converges at.
// 2. TrajectoryGuidance at the params.def defaults, the vehicle tracking the reference perfectly unless it says otherwise,
//    at 1 kHz: every tick within the velocity, acceleration and jerk limits (and the same check with the jerk limit halved
//    finds violations: the negative control); a single waypoint held unchanged (a silent sender) ends at rest at p and
//    stays there; a 20 m square flown as triplets (the sender advancing at accept_m) passes each 90 deg corner near
//    sqrt(a d tan(alpha / 2)) (TrajMath.hpp:87-93, a = MPC_XY_TRAJ_P acc_xy, d = accept_m), faster than stopping at each
//    corner; a vehicle that does not follow holds the trajectory within err_xy_max / err_z_max of it (err 0 disables that:
//    the control); a land leg's kRefVel is fed forward; no kRefPos, or a mode other than kFly, passes the reference
//    unchanged and restarts the trajectory from nav; the heading turns along the path at yaw_rate_auto when the mission
//    leaves it free. Host ns/tick measured.
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <initializer_list>

#include <marv/fsw/guidance.hpp>
#include <marv/fsw/math.hpp>
#include <marv/fsw/smoothing.hpp>

using namespace marv;

static int failures = 0;
#define CHECK(c)                                                     \
    do {                                                             \
        if (!(c)) {                                                  \
            std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); \
            ++failures;                                              \
        }                                                            \
    } while (0)

namespace {

// gtest's EXPECT_FLOAT_EQ: within 4 ULPs.
bool float_eq(float a, float b) {
    if (a == b) return true;
    std::int32_t ia, ib;
    std::memcpy(&ia, &a, 4);
    std::memcpy(&ib, &b, 4);
    if ((ia < 0) != (ib < 0)) return false;
    return std::abs(ia - ib) <= 4;
}
// A fixture value printed by the PX4 run with 9 significant digits.
bool near(float a, float b) { return std::fabs(a - b) <= 1e-6f * std::fmax(1.f, std::fabs(b)); }
bool near(Vec3 a, Vec3 b) { return near(a.x, b.x) && near(a.y, b.y) && near(a.z, b.z); }
float at(Vec3 v, int i) { return i == 0 ? v.x : (i == 1 ? v.y : v.z); }
float norm_xy(Vec3 v) { return std::sqrt(v.x * v.x + v.y * v.y); }

// ---- 1. PX4 VelocitySmoothingTest.cpp ------------------------------------------------------------------------------------

// VelocitySmoothingTest.cpp:105-147.
struct VelocityCase {
    VelocitySmoothing t[3];
    void constraints(float j_max, float a_max, float v_max) {
        for (VelocitySmoothing& x : t) {
            x.set_max_jerk(j_max);
            x.set_max_accel(a_max);
            x.set_max_vel(v_max);
        }
    }
    void initial(Vec3 a0, Vec3 v0, Vec3 x0) {
        for (int i = 0; i < 3; i++) {
            t[i].set_current_acceleration(at(a0, i));
            t[i].set_current_velocity(at(v0, i));
            t[i].set_current_position(at(x0, i));
        }
    }
    void update(float dt, Vec3 sp) {
        for (VelocitySmoothing& x : t) {
            x.update_traj(dt);
            CHECK(std::fabs(x.current_jerk()) <= x.max_jerk());
            CHECK(std::fabs(x.current_acceleration()) <= x.max_accel());
            CHECK(std::fabs(x.current_velocity()) <= x.max_vel());
        }
        for (int i = 0; i < 3; i++) t[i].update_durations(at(sp, i));
        VelocitySmoothing::time_synchronization(t, 2);
    }
};

void velocity_smoothing() {
    // VelocitySmoothingTest.cpp:46-103: unconfigured, zero jerk with an input, zero jerk with an initial acceleration.
    {
        VelocitySmoothing a, b, c(1.f, 0.f, 0.f);
        a.update_durations(0.f);
        a.update_traj(0.f);
        for (VelocitySmoothing* x : {&b, &c}) {
            x->set_max_jerk(0.f);
            x->set_max_accel(0.f);
            x->set_max_vel(1.f);
            x->update_durations(1.f);
            x->update_traj(1.f);
        }
        for (VelocitySmoothing* x : {&a, &b, &c})
            CHECK(float_eq(x->total_time(), 0.f) && float_eq(x->current_jerk(), 0.f) &&
                  float_eq(x->current_acceleration(), 0.f) && float_eq(x->current_velocity(), 0.f) &&
                  float_eq(x->current_position(), 0.f));
    }
    // VelocitySmoothingTest.cpp:149-172 testTimeSynchronization; PX4: x T 0.11268115 0.80304414 0.108695641,
    // y 0.018000463 0.98842001 0.018000463.
    {
        VelocityCase v;
        v.constraints(55.2f, 6.f, 6.f);
        v.initial({0.22f, 0.f, 0.22f}, {2.47f, -5.59e-6f, 2.47f}, {0.f, 0.f, 0.f});
        v.update(0.f, {-3.f, 1.f, 0.f});
        CHECK(std::fabs(v.t[0].total_time() - v.t[1].total_time()) <= 0.0001f);
        CHECK(near(v.t[0].t1(), 0.11268115f) && near(v.t[0].t2(), 0.80304414f) && near(v.t[0].t3(), 0.108695641f));
        CHECK(near(v.t[1].t1(), 0.018000463f) && near(v.t[1].t2(), 0.98842001f) && near(v.t[1].t3(), 0.018000463f));
    }
    // VelocitySmoothingTest.cpp:174-204 testTimeSynchronizationSameDelta (no constraints set: every duration 0 in PX4).
    {
        VelocityCase v;
        v.initial({0.f, 0.f, 0.f}, {0.5f, -0.2f, 0.f}, {0.f, 0.f, 0.f});
        const Vec3 sp{0.5f + 0.3f, -0.2f + 0.3f, 0.f};
        for (int i = 0; i < 3; i++) v.t[i].update_durations(at(sp, i));
        VelocitySmoothing::time_synchronization(v.t, 3);
        CHECK(float_eq(v.t[0].total_time(), v.t[1].total_time()) && float_eq(v.t[0].t1(), v.t[1].t1()) &&
              float_eq(v.t[0].t2(), v.t[1].t2()) && float_eq(v.t[0].t3(), v.t[1].t3()));
        CHECK(float_eq(v.t[2].total_time(), v.t[0].total_time()) && float_eq(v.t[2].t1(), 0.f) &&
              float_eq(v.t[2].t2(), v.t[0].total_time()) && float_eq(v.t[2].t3(), 0.f));
    }
    // VelocitySmoothingTest.cpp:206-242 testConstantSetpoint; PX4: T 0.108695649 0.391304344 0.108695649, 61 steps,
    // then x at v -2.99999976 x -0.916956306, z at v -1 x -0.472318679, zero acceleration.
    {
        VelocityCase v;
        v.constraints(55.2f, 6.f, 6.f);
        v.initial({0.f, 0.f, 0.f}, {0.f, 0.f, 0.f}, {0.f, 0.f, 0.f});
        const Vec3 sp{-3.f, 0.f, -1.f};
        v.update(0.f, sp);
        const float t123 = v.t[0].total_time();
        const int steps = static_cast<int>(std::ceil(t123 / 0.01f));
        CHECK(near(v.t[0].t1(), 0.108695649f) && near(v.t[0].t2(), 0.391304344f) && near(v.t[0].t3(), 0.108695649f));
        CHECK(steps == 61);
        for (int i = 0; i < steps; i++) v.update(0.01f, sp);
        for (int i = 0; i < 3; i++)
            CHECK(std::fabs(v.t[i].current_velocity() - at(sp, i)) <= 0.01f && std::fabs(v.t[i].current_acceleration()) <= 0.0001f);
        CHECK(near(v.t[0].current_velocity(), -2.99999976f) && near(v.t[0].current_position(), -0.916956306f));
        CHECK(near(v.t[2].current_velocity(), -1.f) && near(v.t[2].current_position(), -0.472318679f));
    }
    // VelocitySmoothingTest.cpp:244-269 testZeroSetpoint.
    {
        VelocityCase v;
        v.initial({0.f, 0.f, 0.f}, {0.f, 0.f, 0.f}, {0.f, 0.f, 0.f});
        for (int i = 0; i < 60; i++) v.update(0.01f, {0.f, 0.f, 0.f});
        for (VelocitySmoothing& x : v.t)
            CHECK(float_eq(x.current_jerk(), 0.f) && float_eq(x.current_acceleration(), 0.f) &&
                  float_eq(x.current_velocity(), 0.f) && float_eq(x.current_position(), 0.f));
    }
}

// ---- 1. PX4 PositionSmoothingTest.cpp ------------------------------------------------------------------------------------

// PositionSmoothingTest.cpp:26-35, 44-56.
constexpr float kMaxJerk = 4.f, kMaxAcc = 3.f, kMaxVel = 5.f;
PositionSmoothing px4_smoothing() {
    PositionSmoothing p;
    p.set_max_jerk(kMaxJerk);
    p.set_max_acceleration({kMaxAcc, kMaxAcc, kMaxAcc});
    p.set_max_velocity({kMaxVel, kMaxVel, kMaxVel});
    p.set_max_allowed_horizontal_error(2.f);
    p.set_vertical_acceptance_radius(0.8f);
    p.set_cruise_speed(5.f);
    p.set_horizontal_trajectory_gain(0.5f);
    p.set_target_acceptance_radius(0.5f);
    p.reset({0.f, 0.f, 0.f}, {0.f, 0.f, 0.f}, {0.f, 0.f, 0.f});
    return p;
}

// PositionSmoothingTest.cpp:58-69.
bool within_limits(const PositionSmoothing::Setpoints& s) {
    bool ok = true;
    for (int i = 0; i < 3; i++)
        ok = ok && std::fabs(at(s.velocity, i)) <= kMaxVel && std::fabs(at(s.acceleration, i)) <= kMaxAcc &&
             std::fabs(at(s.jerk, i)) <= kMaxJerk;
    return ok;
}

// PositionSmoothingTest.cpp:73-195, mode 0 reachesTargetPositionSetpoint, 1 reachesTargetVelocityIntegration,
// 2 reachesTargetInitialVelocity. Returns the iteration it converged at.
int position_case(int mode) {
    PositionSmoothing p = px4_smoothing();
    const Vec3 target{12.f, 17.f, 8.f};
    const Vec3 waypoints[3] = {{0.f, 0.f, 0.f}, target, target};
    Vec3 pos{0.f, 0.f, 0.f};
    Vec3 ff = mode == 2 ? Vec3{1.f, 0.1f, 0.3f} : Vec3{0.f, 0.f, 0.f};
    const int n = mode == 2 ? 20000 : 2000;
    PositionSmoothing::Setpoints out{};
    bool limits = true;
    int it = 0;
    for (; it < n; it++) {
        p.generate_setpoints(pos, waypoints, ff, 0.02f, false, out);
        pos = mode == 1 ? pos + 0.02f * out.velocity : out.position;
        if (mode == 2) ff = {0.f, 0.f, 0.f};
        limits = limits && within_limits(out);
        // PX4: it 50 pos 0.541671336 0.65624994 0.630582035 vel 1.34483063 1.87499952 1.71381962 acc 1.71059859 3
        // 2.48691082; it 200 pos 8.15902424 11.5375319 7.03677464 vel 2.07523131 2.90185761 0.635635674
        // acc -0.988642097 -1.21635711 -0.40864253 (the three cases alike).
        if (it == 50)
            CHECK(near(out.position, {0.541671336f, 0.65624994f, 0.630582035f}) &&
                  near(out.velocity, {1.34483063f, 1.87499952f, 1.71381962f}) &&
                  near(out.acceleration, {1.71059859f, 3.f, 2.48691082f}));
        if (it == 200)
            CHECK(near(out.position, {8.15902424f, 11.5375319f, 7.03677464f}) &&
                  near(out.velocity, {2.07523131f, 2.90185761f, 0.635635674f}) &&
                  near(out.acceleration, {-0.988642097f, -1.21635711f, -0.40864253f}));
        const Vec3 e = pos - target;
        if (mode == 2 ? norm_xy(e) < 10.f && std::fabs(e.z) < 0.8f
                      : std::fabs(e.x) <= 1e-4f && std::fabs(e.y) <= 1e-4f && std::fabs(e.z) <= 1e-4f)  // matrix isEqual
            break;
    }
    CHECK(limits && it < n);
    return it;
}

void position_smoothing() {
    // PositionSmoothingTest.cpp:5-24 AllZeroCase.
    {
        PositionSmoothing p;
        PositionSmoothing::Setpoints out{};
        const Vec3 z{0.f, 0.f, 0.f};
        const Vec3 w[3] = {z, z, z};
        p.generate_setpoints(z, w, z, 0.f, false, out);
        const Vec3 all[5] = {out.jerk, out.acceleration, out.velocity, out.position, out.unsmoothed_velocity};
        for (const Vec3& v : all) CHECK(v.x == 0.f && v.y == 0.f && v.z == 0.f);
    }
    // PX4 converges at iterations 1035, 985 and 215.
    const int it0 = position_case(0), it1 = position_case(1), it2 = position_case(2);
    std::printf("PositionSmoothingTest cases converge at iterations %d %d %d (PX4: 1035 985 215)\n", it0, it1, it2);
    CHECK(it0 == 1035 && it1 == 985 && it2 == 215);
}

// ---- 2. TrajectoryGuidance -----------------------------------------------------------------------------------------------

constexpr float kDt = 0.001f;
const param::TrajectoryParams kP{};

// Per-tick limits: per axis as PositionSmoothing constrains them (PositionSmoothingTest.cpp:58-69), the jerk as the change
// of the acceleration over the tick. Tolerances: float rounding of a over 1 ms.
struct Limits {
    float v_xy, v_up, v_dn, a_xy, a_z, jerk;
};
const Limits kLimits{kP.xy_vel_max, kP.z_vel_up, kP.z_vel_dn, kP.acc_xy[param::k_profile_hold], std::fmax(kP.acc_up[param::k_profile_hold], kP.acc_dn[param::k_profile_hold]), kP.jerk[param::k_profile_hold]};

struct Peaks {
    int violations = 0;
    float v_xy = 0.f, v_up = 0.f, v_dn = 0.f, a_axis = 0.f, a_z = 0.f, jerk = 0.f;  // v_xy: the horizontal norm
    Reference last{};
    bool have = false;
    void add(const Reference& r, const Limits& l) {
        v_xy = std::fmax(v_xy, norm_xy(r.v_ned));
        v_up = std::fmax(v_up, -r.v_ned.z);
        v_dn = std::fmax(v_dn, r.v_ned.z);
        a_axis = std::fmax(a_axis, std::fmax(std::fabs(r.a_ned.x), std::fabs(r.a_ned.y)));
        a_z = std::fmax(a_z, std::fabs(r.a_ned.z));
        bool ok = std::fabs(r.v_ned.x) <= l.v_xy + 1e-4f && std::fabs(r.v_ned.y) <= l.v_xy + 1e-4f &&
                  -r.v_ned.z <= l.v_up + 1e-4f && r.v_ned.z <= l.v_dn + 1e-4f && std::fabs(r.a_ned.x) <= l.a_xy + 1e-4f &&
                  std::fabs(r.a_ned.y) <= l.a_xy + 1e-4f && std::fabs(r.a_ned.z) <= l.a_z + 1e-4f;
        if (have)
            for (int i = 0; i < 3; i++) {
                const float j = std::fabs(at(r.a_ned, i) - at(last.a_ned, i)) / kDt;
                jerk = std::fmax(jerk, j);
                ok = ok && j <= l.jerk * 1.001f + 0.01f;
            }
        violations += ok ? 0 : 1;
        last = r;
        have = true;
    }
};

Reference position_ref(Vec3 p, Vec3 p_next, float accept_m) {
    Reference r{};
    r.has = kRefPos;
    r.p_ned = p;
    r.p_next_ned = p_next;
    r.accept_m = accept_m;
    return r;
}

State at_rest(Vec3 p, float yaw = 0.f) {
    State s{};
    s.p_ned = p;
    s.q = quat_from_euler(0.f, 0.f, yaw);
    s.valid = true;
    return s;
}

// A single waypoint held unchanged for 60 s: the vehicle follows perfectly. Checks the limits every tick. "At rest at p"
// is within kRest: at 1 kHz PX4's own PositionSmoothing (run on this case, bit for bit the port's output) stops
// 1.4 mm short of a target 20 m out with 1 mm/s left, where v dt falls below half a float ulp of the position. The
// horizontal speed may pass the cruise speed by the per-axis shaping of the turn toward the target (5.007 m/s here, PX4
// alike); the limits PositionSmoothing keeps are per axis.
constexpr float kRest = 2e-3f;  // m, m/s
void single_waypoint() {
    TrajectoryGuidance g{kP};
    State nav = at_rest({0.f, 0.f, -2.f});
    const Vec3 p{20.f, 10.f, -5.f};
    const Reference ref = position_ref(p, p, 0.5f);
    Peaks pk, strict;
    Limits half = kLimits;
    half.jerk = 0.5f * kLimits.jerk;
    float drift = 0.f, arrive = -1.f;
    Reference out{};
    Vec3 held{};
    for (int k = 0; k < 60000; ++k) {
        out = g.run(ref, nav, Mode::kFly, kDt);
        pk.add(out, kLimits);
        strict.add(out, half);
        nav.p_ned = out.p_ned;
        nav.v_ned = out.v_ned;
        if (arrive < 0.f && norm(out.p_ned - p) < kRest && norm(out.v_ned) < kRest) arrive = static_cast<float>(k) * kDt;
        if (k == 40000) held = out.p_ned;
        if (k > 40000) drift = std::fmax(drift, norm(out.p_ned - held));
    }
    std::printf("single waypoint 22.6 m away: at rest at p after %.2f s; peaks |v_xy| %.3f up %.3f down %.3f m/s, "
                "|a| axis %.3f z %.3f m/s^2, jerk %.3f m/s^3; %d ticks over the limits; drift over the last 20 s %.2g m\n",
                static_cast<double>(arrive), static_cast<double>(pk.v_xy), static_cast<double>(pk.v_up),
                static_cast<double>(pk.v_dn), static_cast<double>(pk.a_axis), static_cast<double>(pk.a_z),
                static_cast<double>(pk.jerk), pk.violations, static_cast<double>(drift));
    CHECK(pk.violations == 0 && arrive > 0.f);
    CHECK(norm(out.p_ned - p) < kRest && norm(out.v_ned) < kRest && norm(out.a_ned) < 1e-4f && drift < 1e-6f);
    CHECK(pk.v_xy <= 1.01f * kP.cruise_speed[param::k_profile_hold] && pk.v_xy > 0.9f * kP.cruise_speed[param::k_profile_hold]);
    // Negative control: the same trajectory checked against half the jerk limit is caught.
    std::printf("negative control, jerk limit halved to %.1f m/s^3: %d ticks over the limits\n",
                static_cast<double>(half.jerk), strict.violations);
    CHECK(strict.violations > 0);
}

struct SquareRun {
    float time = 0.f;         // s, until within 0.1 m of the last corner
    float corner_speed[3]{};  // m/s, at the tick the sender advanced
    float min_speed[3]{};     // m/s, the least horizontal speed from the advance to 2 s after it
    float closest[3]{};       // m, closest approach to the corner
    Peaks pk;
    float ns_per_tick = 0.f;
};

// A 20 m square at 2 m, the sender advancing at accept_m (3-D distance from the vehicle to p): triplets (p_next the
// corner after) or single waypoints (p_next = p).
SquareRun square(bool triplets, float accept_m) {
    const Vec3 wp[5] = {{0.f, 0.f, -2.f}, {20.f, 0.f, -2.f}, {20.f, 20.f, -2.f}, {0.f, 20.f, -2.f}, {0.f, 0.f, -2.f}};
    TrajectoryGuidance g{kP};
    State nav = at_rest(wp[0]);
    SquareRun r;
    for (float& c : r.closest) c = 1e9f;
    for (float& c : r.min_speed) c = 1e9f;
    int leg = 1, advanced_at[3] = {-1, -1, -1};
    std::uint64_t ns = 0;
    for (int k = 0; k < 120000; ++k) {
        const Vec3 next = triplets && leg < 4 ? wp[leg + 1] : wp[leg];
        const Reference ref = position_ref(wp[leg], next, accept_m);
        const auto t0 = std::chrono::steady_clock::now();
        const Reference out = g.run(ref, nav, Mode::kFly, kDt);
        ns += static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - t0).count());
        r.pk.add(out, kLimits);
        nav.p_ned = out.p_ned;
        nav.v_ned = out.v_ned;
        const float speed = norm_xy(out.v_ned);
        for (int c = 0; c < 3; ++c) {
            r.closest[c] = std::fmin(r.closest[c], norm(nav.p_ned - wp[c + 1]));
            if (advanced_at[c] >= 0 && k - advanced_at[c] <= 2000) r.min_speed[c] = std::fmin(r.min_speed[c], speed);
        }
        if (leg < 4 && norm(nav.p_ned - wp[leg]) < accept_m) {
            r.corner_speed[leg - 1] = speed;
            advanced_at[leg - 1] = k;
            ++leg;
        } else if (leg == 4 && norm(nav.p_ned - wp[4]) < 0.1f) {
            r.time = static_cast<float>(k) * kDt;
            r.ns_per_tick = static_cast<float>(ns) / static_cast<float>(k + 1);
            break;
        }
    }
    return r;
}

void corners() {
    const float accept = 2.f;  // ADR-0011 kArriveWp, ArduPilot AC_WPNav.cpp:10 WP_RADIUS_M_DEFAULT
    const float expect = traj::max_speed_in_waypoint(3.14159265f / 2.f, 0.5f * kP.acc_xy[param::k_profile_hold], accept);
    const SquareRun t = square(true, accept), s = square(false, accept);
    std::printf("20 m square, accept_m %.1f: TrajMath corner speed sqrt(a d tan(45 deg)) = %.3f m/s\n",
                static_cast<double>(accept), static_cast<double>(expect));
    for (int c = 0; c < 3; ++c)
        std::printf("  corner %d: triplet %.3f m/s at the advance, least %.3f in the turn, closest %.2f m; single %.3f m/s, "
                    "closest %.2f m\n",
                    c + 1, static_cast<double>(t.corner_speed[c]), static_cast<double>(t.min_speed[c]),
                    static_cast<double>(t.closest[c]), static_cast<double>(s.corner_speed[c]),
                    static_cast<double>(s.closest[c]));
    std::printf("  completion: triplets %.2f s, single waypoints %.2f s; triplet peaks |v_xy| %.3f m/s |a| axis %.3f "
                "jerk %.3f, %d ticks over the limits; host %.0f ns/tick\n",
                static_cast<double>(t.time), static_cast<double>(s.time), static_cast<double>(t.pk.v_xy),
                static_cast<double>(t.pk.a_axis), static_cast<double>(t.pk.jerk), t.pk.violations,
                static_cast<double>(t.ns_per_tick));
    CHECK(t.time > 0.f && s.time > 0.f && t.time < s.time);
    CHECK(t.pk.violations == 0 && s.pk.violations == 0);
    for (int c = 0; c < 3; ++c) {
        CHECK(std::fabs(t.corner_speed[c] - expect) <= 0.25f * expect);
        CHECK(t.min_speed[c] >= 0.5f * expect && t.closest[c] <= accept);
    }
}

// The vehicle stays where it started: the trajectory waits within err_xy_max (horizontally) and err_z_max (vertically)
// of it; with the errors 0 (hold-back off, the control) it runs to the target.
void hold_back() {
    for (float err : {1.f, 0.f}) {
        param::TrajectoryParams p = kP;
        p.err_xy_max = err * kP.err_xy_max;
        p.err_z_max = err * kP.err_z_max;
        TrajectoryGuidance g{p};
        const State nav = at_rest({0.f, 0.f, -2.f});
        const Vec3 target{20.f, 0.f, -7.f};
        const Reference ref = position_ref(target, target, 0.5f);
        float lead_xy = 0.f, lead_z = 0.f;
        Reference out{};
        for (int k = 0; k < 30000; ++k) {
            out = g.run(ref, nav, Mode::kFly, kDt);
            lead_xy = std::fmax(lead_xy, norm_xy(out.p_ned - nav.p_ned));
            lead_z = std::fmax(lead_z, std::fabs(out.p_ned.z - nav.p_ned.z));
        }
        std::printf("vehicle stuck, err_xy_max %.1f err_z_max %.1f: trajectory lead at most %.3f m horizontal, %.3f m "
                    "vertical after 30 s\n",
                    static_cast<double>(p.err_xy_max), static_cast<double>(p.err_z_max), static_cast<double>(lead_xy),
                    static_cast<double>(lead_z));
        if (err > 0.f) CHECK(lead_xy <= p.err_xy_max && lead_z <= p.err_z_max);
        else CHECK(norm(out.p_ned - target) < kRest);
    }
}

// Land leg (gcs/src/mission.cpp:219): kRefPos at the vehicle's own height, kRefVel 0.5 m/s down.
void land_leg() {
    TrajectoryGuidance g{kP};
    State nav = at_rest({3.f, 4.f, -10.f});
    Reference out{};
    for (int k = 0; k < 5000; ++k) {
        Reference ref = position_ref({3.f, 4.f, nav.p_ned.z}, {3.f, 4.f, nav.p_ned.z}, 0.5f);
        ref.has |= kRefVel;
        ref.v_ned = {0.f, 0.f, 0.5f};
        out = g.run(ref, nav, Mode::kFly, kDt);
        nav.p_ned = out.p_ned;
        nav.v_ned = out.v_ned;
    }
    std::printf("land leg: descent %.3f m/s after 5 s (feed-forward 0.5)\n", static_cast<double>(out.v_ned.z));
    CHECK(std::fabs(out.v_ned.z - 0.5f) < 0.05f && norm_xy(out.v_ned) < 1e-4f && norm_xy(out.p_ned - Vec3{3.f, 4.f, 0.f}) < 1e-4f);
}

// No kRefPos passes the reference unchanged, bit for bit; the next kRefPos starts from nav; so does kFly after kIdle.
void restart() {
    TrajectoryGuidance g{kP};
    State nav = at_rest({3.f, 4.f, -2.f});
    nav.v_ned = {1.f, 0.f, 0.f};
    Reference manual{};
    manual.has = kRefVel | kRefYawRate;
    manual.v_ned = {1.f, 0.f, 0.f};
    manual.yaw_rate = 0.2f;
    const Reference m1 = g.run(manual, nav, Mode::kFly, kDt);
    CHECK(std::memcmp(&m1, &manual, sizeof(Reference)) == 0);
    const Reference to{position_ref({30.f, 4.f, -2.f}, {30.f, 4.f, -2.f}, 0.5f)};
    Reference out = g.run(to, nav, Mode::kFly, kDt);
    CHECK(near(out.p_ned, nav.p_ned + kDt * nav.v_ned) && near(out.v_ned, nav.v_ned));
    for (int k = 0; k < 2000; ++k) out = g.run(to, nav, Mode::kFly, kDt);
    const Reference idle = g.run(to, nav, Mode::kIdle, kDt);
    CHECK(std::memcmp(&idle, &to, sizeof(Reference)) == 0);
    const State moved = at_rest({-5.f, 1.f, -3.f});
    out = g.run(to, moved, Mode::kFly, kDt);
    CHECK(near(out.p_ned, moved.p_ned) && near(out.v_ned, moved.v_ned) && near(out.a_ned, {0.f, 0.f, 0.f}));
}

// Heading: a kRefYaw passes through; with none it starts at the vehicle's (0.5 rad), holds below heading_min_speed, and
// turns toward the path (due east) at no more than yaw_rate_auto.
void heading() {
    TrajectoryGuidance g{kP};
    State nav = at_rest({0.f, 0.f, -2.f}, 0.5f);
    Reference fixed = position_ref({0.f, 20.f, -2.f}, {0.f, 20.f, -2.f}, 0.5f);
    fixed.has |= kRefYaw;
    fixed.yaw = -1.f;
    CHECK(g.run(fixed, nav, Mode::kFly, kDt).yaw == -1.f);
    TrajectoryGuidance h{kP};
    const Reference free = position_ref({0.f, 20.f, -2.f}, {0.f, 20.f, -2.f}, 0.5f);
    float last = 0.5f, rate = 0.f, held_until = -1.f;
    bool bits = true;
    for (int k = 0; k < 6000; ++k) {
        const Reference out = h.run(free, nav, Mode::kFly, kDt);
        bits = bits && (out.has & kRefYaw);
        if (norm_xy(out.v_ned) <= kP.heading_min_speed) {
            bits = bits && out.yaw == 0.5f;
            held_until = static_cast<float>(k) * kDt;
        }
        rate = std::fmax(rate, std::fabs(out.yaw - last) / kDt);
        last = out.yaw;
        nav.p_ned = out.p_ned;
        nav.v_ned = out.v_ned;
    }
    std::printf("free heading: held at 0.5 rad for %.3f s, turned at most %.3f rad/s (limit %.3f), heading after 6 s "
                "%.4f rad\n",
                static_cast<double>(held_until), static_cast<double>(rate), static_cast<double>(kP.yaw_rate_auto[param::k_profile_hold]),
                static_cast<double>(last));
    CHECK(bits && held_until > 0.f && rate <= kP.yaw_rate_auto[param::k_profile_hold] * 1.001f && std::fabs(last - 1.5707963f) < 1e-3f);
}

}  // namespace

int main() {
    velocity_smoothing();
    position_smoothing();
    single_waypoint();
    corners();
    hold_back();
    land_leg();
    restart();
    heading();
    std::printf(failures ? "FAIL (%d)\n" : "PASS\n", failures);
    return failures ? EXIT_FAILURE : EXIT_SUCCESS;
}
