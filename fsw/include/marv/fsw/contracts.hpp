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
    bool valid;
};

// ---- mission and guidance ------------------------------------------------------------------------

enum class Mode : std::uint8_t {
    kIdle,  // disarmed: every motor command is zero
    kFly,
};

// What the mission wants at an instant. A cleared bit means the field is not specified.
enum RefBit : std::uint8_t {
    kRefPos = 1u << 0,
    kRefVel = 1u << 1,
    kRefAcc = 1u << 2,
    kRefYaw = 1u << 3,
    kRefAtt = 1u << 4,
};

struct Reference {
    std::uint8_t has;  // RefBit mask
    Vec3 p_ned;
    Vec3 v_ned;
    Vec3 a_ned;
    float yaw;  // rad
    Quat q;
};

// ---- controller output ---------------------------------------------------------------------------

struct ControlRequest {
    Vec3 force_ned;   // N
    Vec3 torque_frd;  // N m
};

// ---- allocation / actuator output ----------------------------------------------------------------

inline constexpr int kMotorCount = 4;

// One command per motor as a fraction of full scale, 0..1. Motor i is Gazebo rotor i.
struct ActuatorCommand {
    std::uint64_t t_us;  // the SensorBus tick this answers
    float motor[kMotorCount];
    bool armed;
};

}  // namespace marv
