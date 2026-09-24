// The contracts between flight-software modules. Every module reads and writes only these types, so any
// estimator can feed any controller, and the firmware never sees how its inputs were produced (Gazebo,
// a replayed log, or real sensors).
//
// Conventions, fixed for every module:
//   world frame  NED (north, east, down), metres
//   body frame   FRD (forward, right, down)
//   attitude     Hamilton quaternion, body -> NED, scalar first
//   time         microseconds since the start of the run
//   units        SI, except the magnetometer (microtesla) and GNSS position (degrees * 1e7)
#pragma once

#include <cstdint>

namespace marv {

struct Vec3 {
    float x, y, z;
};

struct Quat {
    float w, x, y, z;
};

// ---- sensors: what the firmware is given ---------------------------------------------------------

struct ImuSample {
    Vec3 accel_frd;  // specific force, m/s^2 (reads (0, 0, -g) level at rest)
    Vec3 gyro_frd;   // angular rate, rad/s
};

struct BaroSample {
    float pressure_pa;
    float temperature_c;
};

struct MagSample {
    Vec3 field_frd_ut;  // microtesla
};

// A geodetic position.
struct GeoPoint {
    std::int32_t lat_e7;  // degrees * 1e7
    std::int32_t lon_e7;  // degrees * 1e7
    float alt_m;          // above the WGS84 ellipsoid
};

struct GnssSample {
    std::int32_t lat_e7;  // degrees * 1e7
    std::int32_t lon_e7;  // degrees * 1e7
    float alt_m;          // above the WGS84 ellipsoid
    Vec3 vel_ned;         // m/s
    bool fix;
};

// Bits of SensorBus::fresh: which samples are new this tick. A stale sample keeps its last value.
enum SensorBit : std::uint8_t {
    kImu = 1u << 0,
    kBaro = 1u << 1,
    kMag = 1u << 2,
    kGnss = 1u << 3,
};

struct SensorBus {
    std::uint64_t t_us;
    std::uint8_t fresh;  // SensorBit mask
    ImuSample imu;
    BaroSample baro;
    MagSample mag;
    GnssSample gnss;
};

// ---- estimator output ----------------------------------------------------------------------------

struct State {
    std::uint64_t t_us;
    Vec3 p_ned;    // m, relative to the start position
    Vec3 v_ned;    // m/s
    Quat q;        // body FRD -> NED
    Vec3 w_frd;    // rad/s
    bool valid;    // every field is fit to fly on: attitude and rate aligned, p_ned/v_ned about the estimator's
                   // GNSS origin; latches, so it never drops within a run
};

// ---- mission and guidance ------------------------------------------------------------------------

enum class Mode : std::uint8_t {
    kIdle,  // disarmed: every motor command is zero
    kFly,
    kArmed,  // armed on the ground: no control, every motor at the actuators' spin_arm (a rocket: armed, brake closed)
};

// What the mission wants at an instant. A cleared bit means the field is not specified.
enum RefBit : std::uint8_t {
    kRefPos = 1u << 0,
    kRefVel = 1u << 1,
    kRefAcc = 1u << 2,
    kRefYaw = 1u << 3,
    kRefAtt = 1u << 4,
    kRefYawRate = 1u << 5,
    kRefCoast = 1u << 6,   // the mission: the rocket coasts (after burnout, before apogee)
    kRefApogee = 1u << 7,  // guidance: apogee_m and apogee_pred_m hold
};

struct Reference {
    std::uint8_t has;  // RefBit mask
    Vec3 p_ned;
    Vec3 v_ned;
    Vec3 a_ned;
    float yaw;  // rad
    float yaw_rate;  // rad/s about body z (FRD: positive = clockwise seen from above)
    Quat q;
    float apogee_m;       // m above the start, the target apogee (kRefApogee)
    float apogee_pred_m;  // m above the start, the apogee predicted with the brake closed (kRefApogee)
    Vec3 p_next_ned;  // the waypoint after p_ned (kRefPos); equal to p_ned when there is no next waypoint
    float speed_mps;  // m/s, the horizontal speed toward p_ned; 0 = the guidance's cruise speed parameter
    float accept_m;   // m, the radius around p_ned the sender advances to p_next_ned at; the guidance corners inside it
};

// Where the controller takes its state from. kTruth is the lab's A/B switch (the toolbox's
// fswSource: 'truth'): the simulator's true state flies the vehicle while the estimator runs in shadow.
enum class NavSource : std::uint8_t {
    kEstimate,
    kTruth,
};

// The pilot's sticks, each normalized to -1..1 (0 centred); the flight software scales them by the active profile.
struct Sticks {
    float fwd;    // + forward
    float right;  // + right
    float up;     // + up
    float yaw;    // + clockwise seen from above
};

// What the mission software sends: arm state, navigation source, the current reference, the flight profile and the
// pilot's sticks. A sender that sets none of the last three sends hold, auto and centred sticks.
struct MissionCommand {
    Mode mode;
    NavSource nav;
    Reference ref;
    std::uint8_t profile = 0;  // param::Profile, 0 hold; an unknown one reads as hold
    std::uint8_t manual = 0;   // 1: the sticks fly; 0 (and any unknown value): auto, the reference flies
    Sticks sticks{};           // with manual == 1
};

// ---- controller output ---------------------------------------------------------------------------

// Normalized: the airframe's size enters only through the hover thrust the controller learns.
struct ControlRequest {
    Vec3 thrust_ned;     // thrust vector as a fraction of full collective, |.| <= 1
    Vec3 torque_frd;     // each axis as a fraction of its full authority, -1..1
    float thrust_hover;  // the controller's learned hover fraction: the collective allocation keeps before roll/pitch
    float brake;         // air-brake deployment, 0..1
};

// ---- allocation / actuator output ----------------------------------------------------------------

inline constexpr int kMotorCount = 4;

// One command per motor as a fraction of full scale, 0..1 (motor i is Gazebo rotor i), and the air-brake servo.
struct ActuatorCommand {
    std::uint64_t t_us;  // the SensorBus tick this answers
    float motor[kMotorCount];
    float brake;  // air-brake deployment, 0..1
    bool armed;
};

// ---- telemetry: what the flight software reports each tick ------------------------------------

struct Telemetry {
    std::uint64_t t_us;
    State est;           // the estimator's output, whichever source is flying
    ControlRequest req;  // what the controller asked for
    std::uint8_t preset; // the module preset running (see presets.hpp)
    bool home_valid;     // home = the first GNSS fix, the origin of every p_ned
    GeoPoint home;
    std::uint8_t profile;  // the flight profile applied (param::Profile)
};

}  // namespace marv
