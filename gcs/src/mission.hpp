// The GCS mission executor (ADR-0010 (b), (c)): arm, climb, hold, a waypoint mission, return to home and land, flown
// as MissionCommand frames at 20 Hz. Pure: fed telemetry, commands and one monotonic clock (seconds); no I/O. Link
// sends the frame tick() returns and broadcasts status() as mission_state.
//
//   disarmed -arm-> armed -climb-> climb -reached-> hold -mission_start-> mission -last wp-> rth -over home-> hold
//   rth: climb/hold/mission/land; land: climb/hold/mission/rth; landed -> disarmed; disarm: any state.
//
// Frames (ADR-0011): climb, hold and land hold a heading (kRefYaw), re-captured from telemetry on entering each; mission
// and rth legs send kRefPos without kRefYaw (the guidance points the nose along the path). A mission leg's p_next is the
// following waypoint; the last waypoint's p_next is itself (the vehicle stops there: the rth that follows starts with a
// climb over the point it stopped at). Every other frame's p_next is its p. accept_m is the radius the executor
// advances at, one function for both: the profile's on mission legs and the rth return over home (hold, freestyle,
// stabilized 2.0 m; agile 1.0 m), 0.5 m for climb, the rth climb and hold. speed_mps is mission_start's on mission legs,
// 0 (the cruise parameter) elsewhere.
//
// Profile (ADR-0012): every frame carries the selected flight profile. It is set in any state; set while disarmed it
// waits for the next arm. A disarm or a landing returns it to hold, so a flight arms in hold unless one was set.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <marv/fsw/contracts.hpp>
#include <marv/fsw/geo.hpp>

namespace marv::gcs {

// Degrees, and metres above home.
struct Waypoint {
    double lat, lon, alt_m;
};

class Mission {
public:
    enum class State : std::uint8_t { kDisarmed, kArmed, kClimb, kHold, kMission, kRth, kLand };

    struct Status {
        State state;
        int wp_index;  // 0-based, -1 outside a mission
        int wp_count;  // 0 outside a mission
        bool has_target;
        Waypoint target;  // the leg flown now (land: the touchdown point)
        float dist_m;     // 3-D, estimate to target
        bool has_climb_alt;
        float climb_alt_m;
        bool has_home;
        double home_lat, home_lon;
        std::uint8_t profile;  // param::Profile, the one the frames carry
        std::string reason;
    };

    // The latest telemetry, and whether the flight controller reports itself armed.
    void telemetry(const Telemetry& t, bool armed, double now);

    // Each returns the refusal, empty when accepted.
    std::string arm(bool link_open, double now);
    std::string disarm(double now);
    std::string climb(double alt_m);
    // speed_mps: 0 (the guidance's cruise parameter) or 0.5..20 m/s.
    std::string start(const std::vector<Waypoint>& wps, double speed_mps = 0.0);
    std::string rth();
    std::string land();
    // Any state; profile < param::kProfileCount.
    std::string set_profile(std::uint8_t profile);

    // One 20 Hz period: the automatic transitions, then true with the frame to send, or false to stay silent.
    bool tick(double now, MissionCommand& out);

    // Frames are going out: a state other than disarmed, or the second of kIdle after one.
    bool engaged(double now) const;
    Status status(double now) const;

    static const char* name(State s);

private:
    bool stale(double now) const;
    void advance(double now);
    void start_rth();
    void hold_heading();
    float accept() const;
    MissionCommand frame(Mode mode, std::uint8_t has, const Vec3& p, const Vec3& v) const;

    State state_ = State::kDisarmed;
    Telemetry tlm_{};
    bool have_tlm_ = false;
    bool tlm_armed_ = false;
    double tlm_at_ = 0.0;

    LocalFrame frame_;     // about telemetry.home at arm
    Vec3 home_ned_{};      // the estimate's xy at arm
    float yaw_ = 0.f;      // the heading at arm or on entering climb, hold or land: flown there
    bool has_climb_ = false;
    float climb_alt_ = 0.f;
    Vec3 target_{};        // climb, hold, the mission's waypoint, the rth leg
    std::vector<Vec3> wps_;
    int wp_ = -1;
    float speed_ = 0.f;    // mission_start's speed_mps
    std::uint8_t profile_ = 0;  // param::Profile: hold until set
    int rth_leg_ = 0;      // 0: up to rth_alt over the start point, 1: over home
    Vec3 land_{};          // land: xy at the command
    bool still_ = false;                // land: the landed conditions hold since landed_since_ (telemetry t_us)
    std::uint64_t landed_since_ = 0;
    double idle_until_ = -1.0;  // kIdle frames until then, after a disarm or a landing
    std::string reason_;
};

}  // namespace marv::gcs
