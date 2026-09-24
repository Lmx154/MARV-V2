// Complementary filter, ported from the avionics toolbox src/lib/sim/lab/blocks/estimator.ts (complementary block and
// translationBlend). Attitude: the integrated gyro, pulled toward the accelerometer's gravity direction while
// | |a_m| - g | is within the gate and toward the magnetometer heading. Position and velocity: dead-reckoned rotated
// specific force plus gravity, blended toward GNSS and the barometer with first-order gains. The gyro bias is taken
// as zero, not estimated. float only, no heap.
#pragma once

#include <cstdint>

#include <marv/fsw/contracts.hpp>
#include <marv/fsw/ekf.hpp>
#include <marv/fsw/eskf.hpp>
#include <marv/fsw/geo.hpp>

namespace marv {

// Gains of translationBlend (complementary block defaults, tuned in the lab on quad-square), 1/s.
struct BlendGains {
    float k_pos = 1.f;
    float k_vel = 3.f;
    float k_baro = 1.f;
};

// translationBlend: position and velocity of both complementary filters.
class TranslationBlend {
public:
    explicit TranslationBlend(const BlendGains& g, float gravity) : g_(g), gravity_(gravity) {}
    void reset(std::uint64_t t_us);
    // Dead-reckons one IMU sample of length dt at attitude q.
    void predict(Quat q, Vec3 am, float dt);
    // Blends toward the fresh GNSS fix (NED about the origin) and velocity, and the fresh barometric height (up).
    void correct(const SensorBus& bus, const LocalFrame& frame, float baro_h);
    Vec3 pos() const { return pos_; }
    Vec3 vel() const { return vel_; }

private:
    BlendGains g_;
    float gravity_;
    Vec3 pos_{0.f, 0.f, 0.f};
    Vec3 vel_{0.f, 0.f, 0.f};
    std::uint64_t last_gnss_us_ = 0;
    std::uint64_t last_baro_us_ = 0;
};

struct ComplementaryParams {
    float k_acc = 0.75f;       // 1/s, tilt correction
    float accel_gate = 0.25f;  // m/s^2
    float k_mag = 1.f;         // 1/s, heading correction
    BlendGains blend{};
};

class Complementary {
public:
    // env: gravity, the reference field and the alignment window (Eskf's).
    explicit Complementary(const ComplementaryParams& p = {}, const EskfParams& env = {});

    void update(const SensorBus& bus);
    State state() const;
    bool aligned() const { return aligned_; }

private:
    ComplementaryParams prm_;
    float gravity_;
    float mag_decl_;
    StationaryAlignment alignment_;
    LocalFrame frame_;
    TranslationBlend tr_;
    bool aligned_ = false;
    std::uint64_t t_us_ = 0;
    std::uint64_t last_imu_us_ = 0;
    std::uint64_t last_mag_us_ = 0;
    float baro_h0_ = 0.f;
    Quat q_{1.f, 0.f, 0.f, 0.f};
    Vec3 w_meas_{0.f, 0.f, 0.f};
};

}  // namespace marv
