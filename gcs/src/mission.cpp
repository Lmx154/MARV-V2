#include "mission.hpp"

#include <algorithm>
#include <cmath>

#include <marv/fsw/math.hpp>
#include <marv/fsw/params.hpp>

#include "setpoint.hpp"

namespace marv::gcs {
namespace {

// ADR-0010 (b), (c).
constexpr double kStale = 1.0;        // s: telemetry older than this is stale
constexpr double kIdleFor = 1.0;      // s of kIdle frames after a disarm or a landing
constexpr float kArrive = 0.5f;       // m, 3-D in the estimate frame: climb, the rth climb, hold
// m, 3-D, by profile: mission legs and the rth return (ArduPilot WP_RADIUS_M_DEFAULT 2.0; agile 1.0, ADR-0012)
constexpr float kArriveWp[param::kProfileCount] = {2.0f, 2.0f, 2.0f, 1.0f};
constexpr float kSpeedMin = 0.5f, kSpeedMax = 20.f;  // m/s, mission_start's speed_mps when not 0
constexpr float kAltMin = 0.5f, kAltMax = 200.f;  // m above home: climb and waypoint altitudes
constexpr float kRangeMax = 5000.f;   // m, a waypoint from home
constexpr std::size_t kWaypointsMax = 64;
constexpr float kLandSpeed = 0.5f;    // m/s down
constexpr std::uint64_t kLandedFor = 1000000;  // us of the vehicle's clock the landed conditions hold continuously
constexpr float kLandedVz = 0.15f, kLandedVxy = 0.3f, kLandedAlt = 1.f;  // m/s, m/s, m above home

float dist(const Vec3& a, const Vec3& b) {
    const float x = a.x - b.x, y = a.y - b.y, z = a.z - b.z;
    return std::sqrt(x * x + y * y + z * z);
}

bool in_alt(double alt_m) { return std::isfinite(alt_m) && alt_m >= kAltMin && alt_m <= kAltMax; }

}  // namespace

const char* Mission::name(State s) {
    switch (s) {
    case State::kDisarmed: return "disarmed";
    case State::kArmed: return "armed";
    case State::kClimb: return "climb";
    case State::kHold: return "hold";
    case State::kMission: return "mission";
    case State::kRth: return "rth";
    case State::kLand: return "land";
    }
    return "?";
}

void Mission::telemetry(const Telemetry& t, bool armed, double now) {
    tlm_ = t;
    tlm_armed_ = armed;
    tlm_at_ = now;
    have_tlm_ = true;
}

bool Mission::stale(double now) const { return !have_tlm_ || now - tlm_at_ > kStale; }

std::string Mission::arm(bool link_open, double now) {
    if (state_ != State::kDisarmed) return std::string("already armed (") + name(state_) + ")";
    if (!link_open) return "link not open";
    if (stale(now)) return "no telemetry for 1 s";
    if (!tlm_.est.valid) return "estimate not valid";
    if (!tlm_.home_valid) return "no home: the flight controller has no GNSS fix yet";
    if (tlm_armed_) return "the flight controller is already armed";
    frame_.set(tlm_.home);
    home_ned_ = {tlm_.est.p_ned.x, tlm_.est.p_ned.y, 0.f};
    yaw_ = yaw_of(tlm_.est.q);
    has_climb_ = false;
    wps_.clear();
    wp_ = -1;
    idle_until_ = -1.0;
    reason_.clear();
    state_ = State::kArmed;
    return {};
}

std::string Mission::disarm(double now) {
    state_ = State::kDisarmed;
    wp_ = -1;
    profile_ = param::k_profile_hold;
    idle_until_ = now + kIdleFor;
    reason_.clear();
    return {};
}

std::string Mission::climb(double alt_m) {
    if (state_ != State::kArmed && state_ != State::kHold)
        return std::string("not in ") + name(state_) + " (climb: armed or hold)";
    if (!in_alt(alt_m)) return "alt_m must be 0.5..200 m above home";
    has_climb_ = true;
    climb_alt_ = static_cast<float>(alt_m);
    target_ = {tlm_.est.p_ned.x, tlm_.est.p_ned.y, -climb_alt_};
    reason_.clear();
    state_ = State::kClimb;
    hold_heading();
    return {};
}

std::string Mission::start(const std::vector<Waypoint>& wps, double speed_mps) {
    if (state_ != State::kHold) return std::string("not in ") + name(state_) + " (mission_start: hold)";
    if (wps.empty() || wps.size() > kWaypointsMax) return "want 1..64 waypoints";
    if (!(speed_mps == 0.0 || (speed_mps >= kSpeedMin && speed_mps <= kSpeedMax)))
        return "speed_mps must be 0.5..20 m/s (0: the cruise speed)";
    std::vector<Vec3> ned;
    for (std::size_t i = 0; i < wps.size(); ++i) {
        const Waypoint& w = wps[i];
        const std::string at = "waypoint " + std::to_string(i) + ": ";
        if (!std::isfinite(w.lat) || !std::isfinite(w.lon) || std::fabs(w.lat) > 90.0 || std::fabs(w.lon) > 180.0)
            return at + "lat/lon out of range";
        if (!in_alt(w.alt_m)) return at + "alt_m must be 0.5..200 m above home";
        const Vec3 p = ground::setpoint(frame_, w.lat, w.lon, w.alt_m);
        if (!(std::hypot(p.x - home_ned_.x, p.y - home_ned_.y) <= kRangeMax)) return at + "more than 5 km from home";
        ned.push_back(p);
    }
    wps_ = std::move(ned);
    wp_ = 0;
    speed_ = static_cast<float>(speed_mps);
    target_ = wps_[0];
    reason_.clear();
    state_ = State::kMission;
    return {};
}

void Mission::start_rth() {
    const float alt_now = -tlm_.est.p_ned.z;
    const float rth_alt = std::max(alt_now, has_climb_ ? climb_alt_ : 0.f);
    target_ = {tlm_.est.p_ned.x, tlm_.est.p_ned.y, -rth_alt};
    rth_leg_ = 0;
    wp_ = -1;
    state_ = State::kRth;
}

std::string Mission::rth() {
    if (state_ != State::kClimb && state_ != State::kHold && state_ != State::kMission && state_ != State::kLand)
        return std::string("not in ") + name(state_) + " (rth: climb, hold, mission or land)";
    reason_.clear();
    start_rth();
    return {};
}

std::string Mission::land() {
    if (state_ != State::kClimb && state_ != State::kHold && state_ != State::kMission && state_ != State::kRth)
        return std::string("not in ") + name(state_) + " (land: climb, hold, mission or rth)";
    land_ = {tlm_.est.p_ned.x, tlm_.est.p_ned.y, 0.f};
    still_ = false;
    wp_ = -1;
    reason_.clear();
    state_ = State::kLand;
    hold_heading();
    return {};
}

std::string Mission::set_profile(std::uint8_t profile) {
    if (profile >= param::kProfileCount) return "unknown profile";
    profile_ = profile;
    return {};
}

void Mission::hold_heading() { yaw_ = yaw_of(tlm_.est.q); }

float Mission::accept() const {
    if (state_ == State::kMission || (state_ == State::kRth && rth_leg_ == 1)) return kArriveWp[profile_];
    return kArrive;
}

void Mission::advance(double now) {
    const marv::State& e = tlm_.est;
    switch (state_) {
    case State::kClimb:
        if (dist(e.p_ned, target_) >= accept()) break;
        state_ = State::kHold;
        hold_heading();
        reason_ = "altitude reached";
        break;
    case State::kMission:
        if (dist(e.p_ned, target_) >= accept()) break;
        if (++wp_ < static_cast<int>(wps_.size())) {
            target_ = wps_[static_cast<std::size_t>(wp_)];
        } else {
            start_rth();
            reason_ = "mission complete";
        }
        break;
    case State::kRth:
        if (dist(e.p_ned, target_) >= accept()) break;
        if (rth_leg_ == 0) {
            rth_leg_ = 1;
            target_ = {home_ned_.x, home_ned_.y, target_.z};
        } else {
            state_ = State::kHold;
            hold_heading();
            reason_ = "home reached";
        }
        break;
    case State::kLand: {
        const bool still = std::fabs(e.v_ned.z) < kLandedVz && std::hypot(e.v_ned.x, e.v_ned.y) < kLandedVxy &&
                           -e.p_ned.z < kLandedAlt;
        if (!still) {
            still_ = false;
        } else if (!still_ || tlm_.t_us < landed_since_) {
            still_ = true;
            landed_since_ = tlm_.t_us;
        } else if (tlm_.t_us - landed_since_ >= kLandedFor) {
            state_ = State::kDisarmed;
            profile_ = param::k_profile_hold;
            idle_until_ = now + kIdleFor;
            reason_ = "landed";
        }
        break;
    }
    case State::kDisarmed:
    case State::kArmed:
    case State::kHold:
        break;
    }
}

MissionCommand Mission::frame(Mode mode, std::uint8_t has, const Vec3& p, const Vec3& v) const {
    MissionCommand c{};
    c.mode = mode;
    c.nav = NavSource::kEstimate;
    c.ref.has = has;
    c.ref.p_ned = p;
    c.ref.p_next_ned = p;
    if (state_ == State::kMission && wp_ + 1 < static_cast<int>(wps_.size()))
        c.ref.p_next_ned = wps_[static_cast<std::size_t>(wp_ + 1)];
    c.ref.speed_mps = state_ == State::kMission ? speed_ : 0.f;
    c.ref.accept_m = accept();
    c.ref.v_ned = v;
    c.ref.yaw = yaw_;
    c.ref.q = {1.f, 0.f, 0.f, 0.f};
    c.profile = profile_;
    return c;
}

bool Mission::tick(double now, MissionCommand& out) {
    if (stale(now)) still_ = false;  // the landed second is observed: a gap in telemetry restarts it
    else if (state_ != State::kDisarmed) advance(now);
    switch (state_) {
    case State::kDisarmed:
        if (now >= idle_until_) return false;
        out = frame(Mode::kIdle, 0, {}, {});
        return true;
    case State::kArmed:
        out = frame(Mode::kArmed, 0, {}, {});
        return true;
    case State::kLand:
        out = frame(Mode::kFly, kRefPos | kRefVel | kRefYaw, {land_.x, land_.y, tlm_.est.p_ned.z}, {0.f, 0.f, kLandSpeed});
        return true;
    case State::kClimb:
    case State::kHold:
        out = frame(Mode::kFly, kRefPos | kRefYaw, target_, {});
        return true;
    case State::kMission:
    case State::kRth:
        out = frame(Mode::kFly, kRefPos, target_, {});
        return true;
    }
    return false;
}

bool Mission::engaged(double now) const { return state_ != State::kDisarmed || now < idle_until_; }

Mission::Status Mission::status(double now) const {
    Status s{};
    s.state = state_;
    s.wp_index = state_ == State::kMission ? wp_ : -1;
    s.wp_count = state_ == State::kMission ? static_cast<int>(wps_.size()) : 0;
    s.profile = profile_;
    s.reason = state_ != State::kDisarmed && stale(now) ? "telemetry stale" : reason_;
    if (state_ == State::kDisarmed) return s;
    const auto to_geo = [this](const Vec3& ned) {
        const GeoPoint g = frame_.to_geo(ned);
        return Waypoint{static_cast<double>(g.lat_e7) * 1e-7, static_cast<double>(g.lon_e7) * 1e-7,
                        static_cast<double>(-ned.z)};
    };
    const Waypoint home = to_geo(home_ned_);
    s.has_home = true;
    s.home_lat = home.lat;
    s.home_lon = home.lon;
    s.has_climb_alt = has_climb_;
    s.climb_alt_m = climb_alt_;
    if (state_ == State::kArmed) return s;
    const Vec3 target = state_ == State::kLand ? land_ : target_;
    s.has_target = true;
    s.target = to_geo(target);
    s.dist_m = dist(tlm_.est.p_ned, target);
    return s;
}

}  // namespace marv::gcs
