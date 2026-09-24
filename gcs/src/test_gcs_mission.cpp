// The mission executor (mission.hpp, ADR-0010 (b), (c)) on its own: telemetry, commands and time in, frames and
// status out.
//   1. the full sequence: arm, climb, hold, a 3-waypoint mission, rth to hold over home, land, landed.
//   2. rth_alt = max(alt_now, climb_alt), both ways; rth mid-mission.
//   3. every refusal: arm (link, stale, estimate, home, already armed), commands outside their states, bounds.
//   4. landed after 1.0 s of the landed conditions, not at 0.9 s; a gap in telemetry restarts that second.
//   5. airborne disarm: kIdle at 20 Hz for 1 s, then silence; disarm when disarmed is accepted.
//   6. stale telemetry: state and frame kept, reason "telemetry stale", no automatic transition.
//   7. ADR-0011: p_next chained through the mission (the last waypoint's is itself); the executor advances at exactly
//      the accept_m it sends, in every state that advances; speed_mps bounds and where it is sent; the heading
//      re-captured on entering climb, hold and land, and held there.
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include <marv/fsw/geo.hpp>
#include <marv/fsw/math.hpp>

#include "mission.hpp"

namespace {

int g_fails = 0;
#define CHECK(c)                                                                  \
    do {                                                                          \
        if (!(c)) {                                                               \
            std::fprintf(stderr, "%s:%d: CHECK(%s) failed\n", __FILE__, __LINE__, #c); \
            ++g_fails;                                                            \
        }                                                                         \
    } while (0)

using namespace marv;
using gcs::Mission;
using gcs::Waypoint;
using S = Mission::State;

const GeoPoint kHome{473763880, 85477780, 408.f};  // the SITL world's origin (sitl/gazebo/marv_quad.sdf)
constexpr float kYaw = 0.6f;
constexpr Quat kQ{0.9553365f, 0.f, 0.f, 0.2955202f};  // yaw 0.6 rad

bool near(const Vec3& a, const Vec3& b, float tol = 1e-4f) {
    return std::fabs(a.x - b.x) <= tol && std::fabs(a.y - b.y) <= tol && std::fabs(a.z - b.z) <= tol;
}

// A vehicle, its telemetry and one clock in whole milliseconds, driving an executor.
struct Rig {
    Mission m;
    std::int64_t ms = 100000;
    Vec3 p{0.2f, -0.1f, 0.f};
    Vec3 v{0.f, 0.f, 0.f};
    Quat q = kQ;
    bool valid = true, home = true, fc_armed = false;
    MissionCommand cmd{};
    bool sent = false;

    double t() const { return static_cast<double>(ms) / 1000.0; }
    void feed() {
        Telemetry tl{};
        tl.t_us = static_cast<std::uint64_t>(ms) * 1000u;
        tl.est.p_ned = p;
        tl.est.v_ned = v;
        tl.est.q = q;
        tl.est.valid = valid;
        tl.home_valid = home;
        tl.home = home ? kHome : GeoPoint{};
        m.telemetry(tl, fc_armed, t());
    }
    bool tick() {
        sent = m.tick(t(), cmd);
        return sent;
    }
    // One 20 Hz period: fresh telemetry, then the tick.
    bool step() {
        ms += 50;
        feed();
        return tick();
    }
    // One period with no telemetry.
    bool quiet() {
        ms += 50;
        return tick();
    }
    S state() const { return m.status(t()).state; }
    std::string reason() const { return m.status(t()).reason; }
    // Flies to the frame's position setpoint (off metres north of it) and takes one period there.
    void reach(float off = 0.f) {
        p = {cmd.ref.p_ned.x + off, cmd.ref.p_ned.y, cmd.ref.p_ned.z};
        step();
    }
    // Moves the vehicle; the executor hears it at once.
    void at(const Vec3& q) {
        p = q;
        feed();
    }
};

bool is_fly(const MissionCommand& c, const Vec3& p, float tol = 1e-4f) {
    return c.mode == Mode::kFly && c.nav == NavSource::kEstimate && c.ref.has == (kRefPos | kRefYaw) &&
           near(c.ref.p_ned, p, tol) && c.ref.yaw == yaw_of(kQ);
}

// A mission or rth leg: position only, the nose along the path.
bool is_leg(const MissionCommand& c, const Vec3& p, float tol = 1e-4f) {
    return c.mode == Mode::kFly && c.nav == NavSource::kEstimate && c.ref.has == kRefPos && near(c.ref.p_ned, p, tol);
}

// Degrees of a point NED about kHome.
Waypoint wp(float n, float e, double alt) {
    LocalFrame f;
    f.set(kHome);
    const GeoPoint g = f.to_geo({n, e, 0.f});
    return {g.lat_e7 * 1e-7, g.lon_e7 * 1e-7, alt};
}

// Arms, climbs to alt and holds there, over (0.2, -0.1).
void to_hold(Rig& r, double alt) {
    r.feed();
    CHECK(r.m.arm(true, r.t()).empty());
    CHECK(r.step() && r.cmd.mode == Mode::kArmed && r.state() == S::kArmed);
    CHECK(r.m.climb(alt).empty());
    r.step();
    r.reach();
    CHECK(r.state() == S::kHold);
}

void test_sequence() {
    Rig r;
    r.feed();
    CHECK(!r.tick());  // disarmed: silent
    Mission::Status s = r.m.status(r.t());
    CHECK(s.state == S::kDisarmed && s.wp_index == -1 && !s.has_home && !s.has_target && s.reason.empty());
    CHECK(r.m.arm(true, r.t()).empty());
    CHECK(r.step() && r.cmd.mode == Mode::kArmed && r.cmd.ref.has == 0);
    s = r.m.status(r.t());
    CHECK(s.state == S::kArmed && s.has_home && !s.has_target && !s.has_climb_alt);
    CHECK(std::fabs(s.home_lat - 47.3763898) < 1e-7 && std::fabs(s.home_lon - 8.5477767) < 1e-7);  // 0.2 m N, 0.1 m W

    // climb 5: straight up from the armed point, then hold on arrival within 0.5 m 3-D.
    CHECK(r.m.climb(5.0).empty());
    CHECK(r.step() && is_fly(r.cmd, {0.2f, -0.1f, -5.f}) && r.state() == S::kClimb && r.reason().empty());
    r.p.z = -4.45f;
    r.step();
    CHECK(r.state() == S::kClimb);
    r.p.z = -4.55f;
    r.step();
    CHECK(r.state() == S::kHold && r.reason() == "altitude reached" && is_fly(r.cmd, {0.2f, -0.1f, -5.f}));
    s = r.m.status(r.t());
    CHECK(s.has_climb_alt && s.climb_alt_m == 5.f && s.has_target && s.target.alt_m == 5.0);

    // A mission: three waypoints in order, the nose along the path; each advances at 2.0 m 3-D.
    const std::vector<Waypoint> wps{wp(12.f, 0.f, 6.0), wp(12.f, 12.f, 8.0), wp(0.f, 12.f, 7.0)};
    CHECK(r.m.start(wps).empty());
    r.step();
    s = r.m.status(r.t());
    CHECK(s.state == S::kMission && s.reason.empty() && s.wp_index == 0 && s.wp_count == 3);
    CHECK(std::fabs(s.target.lat - wps[0].lat) < 1e-9 && std::fabs(s.target.lon - wps[0].lon) < 1e-9);
    CHECK(is_leg(r.cmd, {12.f, 0.f, -6.f}, 0.02f) && r.cmd.ref.p_ned.z == -6.f);
    r.reach(2.02f);  // outside the radius
    CHECK(r.m.status(r.t()).wp_index == 0);
    r.p.x -= 0.1f;  // 1.92 m: inside
    r.step();
    CHECK(r.m.status(r.t()).wp_index == 1 && is_leg(r.cmd, {12.f, 12.f, -8.f}, 0.02f));
    r.reach();
    CHECK(r.m.status(r.t()).wp_index == 2 && is_leg(r.cmd, {0.f, 12.f, -7.f}, 0.02f));
    const Vec3 last = r.cmd.ref.p_ned;

    // The last waypoint: rth at max(alt_now = 7, climb_alt = 5), up over the last waypoint, then over home, then hold.
    r.reach();
    s = r.m.status(r.t());
    CHECK(s.state == S::kRth && s.reason == "mission complete" && s.wp_index == -1 && s.wp_count == 0);
    CHECK(is_leg(r.cmd, {last.x, last.y, -7.f}));
    r.reach();
    CHECK(r.state() == S::kRth && is_leg(r.cmd, {0.2f, -0.1f, -7.f}));
    r.reach();
    CHECK(r.state() == S::kHold && r.reason() == "home reached" && is_fly(r.cmd, {0.2f, -0.1f, -7.f}));

    // land: position at the command's xy with the estimate's z, 0.5 m/s down.
    r.at({0.3f, -0.2f, -7.f});
    CHECK(r.m.land().empty());
    r.step();
    CHECK(r.state() == S::kLand && r.reason().empty());
    CHECK(r.cmd.mode == Mode::kFly && r.cmd.ref.has == (kRefPos | kRefVel | kRefYaw) && r.cmd.ref.yaw == yaw_of(kQ));
    CHECK(near(r.cmd.ref.p_ned, {0.3f, -0.2f, -7.f}) && near(r.cmd.ref.v_ned, {0.f, 0.f, 0.5f}));
    s = r.m.status(r.t());
    CHECK(s.has_target && s.target.alt_m == 0.0 && std::fabs(s.dist_m - 7.f) < 1e-4f);
    r.p = {0.5f, -0.2f, -3.f};
    r.step();
    CHECK(near(r.cmd.ref.p_ned, {0.3f, -0.2f, -3.f}));  // z refreshed, xy kept
    r.p.z = -0.05f;
    r.v = {0.f, 0.f, 0.05f};
    for (int i = 0; i < 40 && r.state() == S::kLand; ++i) r.step();
    s = r.m.status(r.t());
    CHECK(s.state == S::kDisarmed && s.reason == "landed" && r.sent && r.cmd.mode == Mode::kIdle && !s.has_target);
}

void test_rth_alt() {
    // Below the climb altitude: rth climbs back to it, from mid-mission.
    Rig r;
    to_hold(r, 10.0);
    CHECK(r.m.start({wp(20.f, 0.f, 3.0), wp(20.f, 20.f, 3.0)}).empty());
    r.step();
    r.at({8.f, 0.f, -3.f});
    CHECK(r.m.rth().empty());
    r.step();
    CHECK(r.state() == S::kRth && r.reason().empty() && is_leg(r.cmd, {8.f, 0.f, -10.f}));
    CHECK(r.m.status(r.t()).wp_index == -1);
    std::printf("rth_alt: at 3 m with climb 10: %.2f m\n", static_cast<double>(-r.cmd.ref.p_ned.z));
    // Above it: rth keeps the altitude it has. Already over home: both legs at once, then hold.
    Rig q;
    to_hold(q, 5.0);
    q.at({0.2f, -0.1f, -9.f});
    CHECK(q.m.rth().empty());
    q.step();
    CHECK(q.state() == S::kRth && is_leg(q.cmd, {0.2f, -0.1f, -9.f}));
    std::printf("rth_alt: at 9 m with climb 5: %.2f m\n", static_cast<double>(-q.cmd.ref.p_ned.z));
    q.step();
    CHECK(q.state() == S::kHold && q.reason() == "home reached");
}

void test_refusals() {
    {
        Rig r;
        r.home = false;
        r.feed();
        CHECK(r.m.arm(true, r.t()).find("no home") != std::string::npos);
        r.home = true;
        r.fc_armed = true;
        r.feed();
        CHECK(r.m.arm(true, r.t()).find("already armed") != std::string::npos);
        r.fc_armed = false;
        r.valid = false;
        r.feed();
        CHECK(r.m.arm(true, r.t()).find("estimate") != std::string::npos);
        r.valid = true;
        r.feed();
        CHECK(r.m.arm(false, r.t()) == "link not open");
        CHECK(!r.m.arm(true, r.t() + 1.01).empty());  // stale
        CHECK(!r.tick() && r.state() == S::kDisarmed);
        Mission fresh;
        CHECK(!fresh.arm(true, 0.0).empty());  // never heard
        // Every other command while disarmed.
        CHECK(!r.m.climb(5.0).empty() && !r.m.start({wp(1.f, 1.f, 2.0)}).empty() && !r.m.rth().empty() &&
              !r.m.land().empty());
        CHECK(r.m.arm(true, r.t()).empty());
        CHECK(!r.m.arm(true, r.t()).empty());  // armed: not again
        // Armed: no mission_start, rth or land; climb bounds.
        CHECK(!r.m.start({wp(1.f, 1.f, 2.0)}).empty() && !r.m.rth().empty() && !r.m.land().empty());
        CHECK(!r.m.climb(0.49).empty() && !r.m.climb(200.1).empty() && !r.m.climb(std::nan("")).empty());
        CHECK(r.state() == S::kArmed);
        CHECK(r.m.climb(0.5).empty());
        r.step();
        // Climb: no mission_start, no second climb.
        CHECK(!r.m.start({wp(1.f, 1.f, 2.0)}).empty() && !r.m.climb(3.0).empty() && r.state() == S::kClimb);
    }
    Rig r;
    to_hold(r, 5.0);
    const auto back_to_hold = [&r] {
        r.step();
        CHECK(r.m.rth().empty());
        r.step();
        r.reach();
        CHECK(r.state() == S::kHold);
    };
    CHECK(r.m.start({}).find("1..64") != std::string::npos);
    CHECK(r.m.start(std::vector<Waypoint>(65, wp(1.f, 1.f, 2.0))).find("1..64") != std::string::npos);
    CHECK(r.m.start(std::vector<Waypoint>(64, wp(1.f, 1.f, 2.0))).empty());  // 64: accepted
    back_to_hold();
    CHECK(r.m.start({wp(1.f, 1.f, 2.0), wp(1.f, 1.f, 0.4)}).find("waypoint 1: alt_m") != std::string::npos);
    CHECK(!r.m.start({wp(1.f, 1.f, 200.5)}).empty());
    CHECK(!r.m.start({wp(1.f, 1.f, std::nan(""))}).empty());
    CHECK(r.m.start({wp(1.f, 1.f, 0.5), wp(1.f, 1.f, 200.0)}).empty());  // the bounds themselves: accepted
    back_to_hold();
    CHECK(r.m.start({wp(4990.f, 0.f, 10.0)}).empty());  // 4990 m: accepted
    back_to_hold();
    CHECK(r.m.start({wp(1.f, 1.f, 2.0), wp(3000.f, 4100.f, 10.0)}).find("5 km") != std::string::npos);  // 5080 m
    CHECK(!r.m.start({{91.0, 8.5, 2.0}}).empty() && !r.m.start({{47.3, std::nan(""), 2.0}}).empty());
    CHECK(r.state() == S::kHold && r.m.status(r.t()).wp_count == 0);
    // Mission: no mission_start, no climb.
    CHECK(r.m.start({wp(10.f, 0.f, 5.0)}).empty());
    r.step();
    CHECK(!r.m.start({wp(10.f, 0.f, 5.0)}).empty() && !r.m.climb(8.0).empty());
    // Land: no land again, no climb, no mission_start; rth is allowed.
    CHECK(r.m.land().empty());
    r.step();
    CHECK(!r.m.land().empty() && !r.m.climb(8.0).empty() && !r.m.start({wp(10.f, 0.f, 5.0)}).empty());
    CHECK(r.m.rth().empty() && r.state() == S::kRth);
    // Rth: no rth again, no climb, no mission_start; land is allowed.
    CHECK(!r.m.rth().empty() && !r.m.climb(8.0).empty() && !r.m.start({wp(10.f, 0.f, 5.0)}).empty());
    CHECK(r.m.land().empty() && r.state() == S::kLand);
}

void test_landed() {
    Rig r;
    to_hold(r, 5.0);
    CHECK(r.m.land().empty());
    r.step();
    r.p.z = -0.4f;
    r.v = {0.2f, 0.1f, 0.1f};  // |vxy| 0.22, |vz| 0.1, 0.4 m: the landed conditions, from this period on
    r.step();
    const std::int64_t t0 = r.ms;
    for (int i = 0; i < 18; ++i) r.step();
    CHECK(r.state() == S::kLand);
    std::printf("landed: at %.2f s %s\n", static_cast<double>(r.ms - t0) * 1e-3, Mission::name(r.state()));
    r.step();
    CHECK(r.state() == S::kLand);
    r.step();
    CHECK(r.state() == S::kDisarmed && r.reason() == "landed");
    std::printf("landed: at %.2f s %s\n", static_cast<double>(r.ms - t0) * 1e-3, Mission::name(r.state()));

    // Each condition, just outside, keeps it landing: vz, vxy, altitude.
    const Vec3 vs[] = {{0.f, 0.f, 0.16f}, {0.25f, 0.2f, 0.f}, {0.f, 0.f, 0.f}};
    for (int k = 0; k < 3; ++k) {
        Rig q;
        to_hold(q, 5.0);
        CHECK(q.m.land().empty());
        q.p.z = k == 2 ? -1.01f : -0.2f;
        q.v = vs[k];
        for (int i = 0; i < 60; ++i) q.step();
        CHECK(q.state() == S::kLand);
    }

    // A gap in telemetry restarts the second: 0.5 s heard, 1.1 s silent, then the full second again.
    Rig g;
    to_hold(g, 5.0);
    CHECK(g.m.land().empty());
    g.p.z = -0.1f;
    for (int i = 0; i < 10; ++i) g.step();
    for (int i = 0; i < 22; ++i) g.quiet();
    CHECK(g.state() == S::kLand && g.reason() == "telemetry stale");
    for (int i = 0; i < 20; ++i) g.step();
    CHECK(g.state() == S::kLand && g.reason().empty());
    g.step();
    CHECK(g.state() == S::kDisarmed);
}

void test_disarm() {
    Rig r;
    to_hold(r, 5.0);
    r.ms = 400000;
    r.feed();
    CHECK(r.m.disarm(r.t()).empty());
    int idle = 0;
    for (int i = 0; i < 20; ++i) {  // 0 .. 0.95 s
        if (i > 0) r.ms += 50;
        if (r.tick() && r.cmd.mode == Mode::kIdle && r.cmd.ref.has == 0) ++idle;
    }
    CHECK(idle == 20 && r.m.engaged(r.t()));
    CHECK(!r.quiet() && !r.m.engaged(r.t()));  // 1.0 s: silent
    CHECK(!r.step() && r.state() == S::kDisarmed && r.reason().empty());
    std::printf("disarm: %d kIdle frames in 1 s, then silent\n", idle);

    // Disarm while disarmed: accepted, the same second of kIdle.
    CHECK(r.m.disarm(r.t()).empty());
    CHECK(r.step() && r.cmd.mode == Mode::kIdle);
    for (int i = 0; i < 25; ++i) r.step();
    CHECK(!r.sent);

    // Disarm from armed on the ground, and from mission.
    Rig a;
    a.feed();
    CHECK(a.m.arm(true, a.t()).empty());
    CHECK(a.m.disarm(a.t()).empty() && a.step() && a.cmd.mode == Mode::kIdle);
    Rig b;
    to_hold(b, 5.0);
    CHECK(b.m.start({wp(10.f, 0.f, 5.0)}).empty());
    b.step();
    CHECK(b.m.disarm(b.t()).empty() && b.step() && b.cmd.mode == Mode::kIdle && b.state() == S::kDisarmed);
    CHECK(b.m.status(b.t()).wp_index == -1 && !b.m.status(b.t()).has_target);
}

void test_stale() {
    Rig r;
    to_hold(r, 5.0);
    CHECK(r.m.start({wp(10.f, 0.f, 5.0)}).empty());
    r.step();
    const MissionCommand before = r.cmd;
    // No telemetry: at 0.95 s still fresh, at 1.05 s stale; the frame goes on unchanged.
    r.ms += 950;
    CHECK(r.tick() && r.reason().empty());
    r.ms += 100;
    CHECK(r.tick() && r.state() == S::kMission && r.reason() == "telemetry stale" && r.m.engaged(r.t()));
    CHECK(r.cmd.mode == before.mode && r.cmd.ref.has == before.ref.has && near(r.cmd.ref.p_ned, before.ref.p_ned, 0.f));
    // At the waypoint while stale: no advance.
    r.p = before.ref.p_ned;
    r.quiet();
    CHECK(r.m.status(r.t()).wp_index == 0 && r.state() == S::kMission);
    // Heard again: the mission goes on (its one waypoint reached: rth).
    r.step();
    CHECK(r.state() == S::kRth && r.reason() == "mission complete");

    // Stale in land: the frame's z is the last one heard.
    Rig l;
    to_hold(l, 5.0);
    l.at({0.2f, -0.1f, -4.f});
    CHECK(l.m.land().empty());
    l.step();
    l.ms += 1500;
    CHECK(l.tick() && l.cmd.ref.p_ned.z == -4.f && l.reason() == "telemetry stale" && l.state() == S::kLand);
}

// The executor advances at exactly the accept_m it sends: 1 cm outside that radius of the frame's p it stays, 1 cm
// inside it moves on (another state, waypoint or target).
bool advances_at_accept(Rig& r) {
    const Vec3 p = r.cmd.ref.p_ned;
    const float a = r.cmd.ref.accept_m;
    const Mission::Status s0 = r.m.status(r.t());
    const auto moved = [&] {
        const Mission::Status s = r.m.status(r.t());
        return s.state != s0.state || s.wp_index != s0.wp_index || !near(r.cmd.ref.p_ned, p, 0.f);
    };
    r.at({p.x + a + 0.01f, p.y, p.z});
    r.tick();
    if (moved()) return false;
    r.at({p.x + a - 0.01f, p.y, p.z});
    r.tick();
    return moved();
}

void test_chaining() {
    Rig r;
    r.feed();
    CHECK(r.m.arm(true, r.t()).empty());
    CHECK(r.m.climb(5.0).empty());
    r.step();
    CHECK(r.cmd.ref.accept_m == 0.5f && near(r.cmd.ref.p_next_ned, r.cmd.ref.p_ned, 0.f) && r.cmd.ref.speed_mps == 0.f);
    CHECK(advances_at_accept(r) && r.state() == S::kHold);
    CHECK(r.cmd.ref.accept_m == 0.5f && near(r.cmd.ref.p_next_ned, r.cmd.ref.p_ned, 0.f) && r.cmd.ref.speed_mps == 0.f);

    // Each leg's p_next is the following waypoint, the last one's itself; 2.0 m, speed_mps on every leg.
    const Vec3 ned[] = {{20.f, 0.f, -5.f}, {20.f, 20.f, -6.f}, {0.f, 20.f, -5.f}};
    CHECK(r.m.start({wp(20.f, 0.f, 5.0), wp(20.f, 20.f, 6.0), wp(0.f, 20.f, 5.0)}, 3.0).empty());
    r.step();
    for (int w = 0; w < 3; ++w) {
        const Vec3& next = ned[w < 2 ? w + 1 : 2];
        CHECK(r.m.status(r.t()).wp_index == w && is_leg(r.cmd, ned[w], 0.02f));
        CHECK(near(r.cmd.ref.p_next_ned, next, 0.02f) && r.cmd.ref.accept_m == 2.f && r.cmd.ref.speed_mps == 3.f);
        std::printf("chaining: leg %d p (%.2f, %.2f, %.2f) p_next (%.2f, %.2f, %.2f) accept %.1f m speed %.1f m/s\n", w,
                    static_cast<double>(r.cmd.ref.p_ned.x), static_cast<double>(r.cmd.ref.p_ned.y),
                    static_cast<double>(r.cmd.ref.p_ned.z), static_cast<double>(r.cmd.ref.p_next_ned.x),
                    static_cast<double>(r.cmd.ref.p_next_ned.y), static_cast<double>(r.cmd.ref.p_next_ned.z),
                    static_cast<double>(r.cmd.ref.accept_m), static_cast<double>(r.cmd.ref.speed_mps));
        CHECK(advances_at_accept(r));
    }
    CHECK(near(r.cmd.ref.p_next_ned, r.cmd.ref.p_ned, 0.f));

    // rth: the climb over the stopping point at 0.5 m, the return over home at 2.0 m, then hold; single, cruise speed.
    CHECK(r.state() == S::kRth && r.cmd.ref.has == kRefPos && r.cmd.ref.accept_m == 0.5f && r.cmd.ref.speed_mps == 0.f);
    CHECK(near(r.cmd.ref.p_next_ned, r.cmd.ref.p_ned, 0.f) && advances_at_accept(r));
    CHECK(is_leg(r.cmd, {0.2f, -0.1f, -5.f}) && r.cmd.ref.accept_m == 2.f && r.cmd.ref.speed_mps == 0.f);
    CHECK(near(r.cmd.ref.p_next_ned, r.cmd.ref.p_ned, 0.f) && advances_at_accept(r));
    CHECK(r.state() == S::kHold && r.reason() == "home reached" && r.cmd.ref.accept_m == 0.5f);

    // land: single, 0.5 m, cruise speed.
    CHECK(r.m.land().empty());
    r.step();
    CHECK(near(r.cmd.ref.p_next_ned, r.cmd.ref.p_ned, 0.f) && r.cmd.ref.accept_m == 0.5f && r.cmd.ref.speed_mps == 0.f);
}

void test_speed() {
    Rig r;
    to_hold(r, 5.0);
    const std::vector<Waypoint> w{wp(10.f, 0.f, 5.0)};
    for (double bad : {0.49, 20.01, -1.0, -0.0001, std::nan(""), HUGE_VAL, 1e300})
        CHECK(r.m.start(w, bad).find("speed_mps") != std::string::npos && r.state() == S::kHold);
    for (double ok : {0.0, 0.5, 20.0}) {
        CHECK(r.m.start(w, ok).empty());
        r.step();
        CHECK(r.state() == S::kMission && r.cmd.ref.speed_mps == static_cast<float>(ok));
        CHECK(r.m.rth().empty());
        r.step();
        CHECK(r.cmd.ref.speed_mps == 0.f);
        r.reach();
        r.reach();
        CHECK(r.state() == S::kHold);
    }
    CHECK(r.m.start(w).empty());  // absent: 0, the cruise speed
    r.step();
    CHECK(r.cmd.ref.speed_mps == 0.f);
}

void test_yaw() {
    const auto yawed = [](float y) { return quat_from_euler(0.f, 0.f, y); };
    Rig r;
    r.feed();
    CHECK(r.m.arm(true, r.t()).empty());
    r.q = yawed(0.9f);
    r.feed();
    CHECK(r.m.climb(5.0).empty());  // entering climb: 0.9
    r.step();
    CHECK(r.cmd.ref.has == (kRefPos | kRefYaw) && r.cmd.ref.yaw == yaw_of(yawed(0.9f)));
    r.q = yawed(1.1f);
    r.step();
    CHECK(r.cmd.ref.yaw == yaw_of(yawed(0.9f)));  // held
    r.reach();  // entering hold: 1.1
    CHECK(r.state() == S::kHold && r.cmd.ref.has == (kRefPos | kRefYaw) && r.cmd.ref.yaw == yaw_of(yawed(1.1f)));
    r.q = yawed(1.3f);
    r.step();
    CHECK(r.cmd.ref.yaw == yaw_of(yawed(1.1f)));  // held
    CHECK(r.m.start({wp(10.f, 0.f, 5.0)}).empty());
    r.step();
    CHECK(r.cmd.ref.has == kRefPos);
    r.q = yawed(-0.5f);  // the nose along the path
    r.reach();
    CHECK(r.state() == S::kRth && r.cmd.ref.has == kRefPos);
    r.reach();
    r.reach();  // entering hold over home: -0.5
    CHECK(r.state() == S::kHold && r.reason() == "home reached" && r.cmd.ref.yaw == yaw_of(yawed(-0.5f)));
    r.q = yawed(2.f);
    r.feed();
    CHECK(r.m.land().empty());  // entering land: 2.0
    r.step();
    CHECK(r.cmd.ref.has == (kRefPos | kRefVel | kRefYaw) && r.cmd.ref.yaw == yaw_of(yawed(2.f)));
    std::printf("yaw: climb %.2f, hold %.2f, hold after rth %.2f, land %.2f rad\n", 0.9, 1.1, -0.5, 2.0);
}

}  // namespace

int main() {
    test_sequence();
    test_rth_alt();
    test_refusals();
    test_landed();
    test_disarm();
    test_stale();
    test_chaining();
    test_speed();
    test_yaw();
    if (g_fails) std::fprintf(stderr, "test_gcs_mission: %d failures\n", g_fails);
    else std::printf("test_gcs_mission: OK\n");
    return g_fails ? 1 : 0;
}
