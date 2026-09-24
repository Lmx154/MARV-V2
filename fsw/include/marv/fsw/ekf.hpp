// Full-state additive EKF, ported from the avionics toolbox:
//   filter        src/lib/calc/ekf-full.ts            (ekfInit, ekfPredict, ekfCorrect)
//   bus wiring    src/lib/sim/lab/blocks/estimator.ts (kalmanEstimator: fsw.ts's createEstimator for another core)
// 16 states x = [p, v, q (4), a_b, w_b], the quaternion corrected additively and re-normalised, the covariance left
// as it is. Same process noise, priors and measurements as Eskf. float only, fixed storage, no heap.
//
// This header also holds the stationary alignment and the magnetometer-heading and barometer front end that the
// EKF, Mahony and complementary estimators share: the toolbox initialises its estimators from truth, so they all
// take Eskf's alignment (eskf.cpp, Eskf::align) instead.
#pragma once

#include <cstdint>

#include <marv/fsw/contracts.hpp>
#include <marv/fsw/eskf.hpp>
#include <marv/fsw/geo.hpp>
#include <marv/fsw/params.hpp>

namespace marv {

// Height above the ISA sea-level pressure (gz-sim's air-pressure model), m.
float isa_height(float pressure_pa);

// Heading innovation yaw_mag - yaw_est of a magnetometer sample against attitude q: the field rotated to NED with q
// lies at (declination + yaw_est - yaw_mag), as in Eskf::fuse_mag.
float heading_innovation(Quat q, Vec3 m_frd, float declination);

// What a stationary alignment found.
struct Alignment {
    Quat q;            // body -> NED: roll and pitch from the mean specific force, yaw from the tilt-compensated field
    Vec3 gyro_mean;    // rad/s, the pad gyro calibration
    float baro_h0;     // m, mean ISA height: the barometric reference
    int n_imu;         // IMU samples averaged
    std::uint64_t t_us;  // the tick it completed on
};

// The same rule as Eskf::accumulate_alignment and Eskf::align: IMU, magnetometer and barometer samples are
// averaged over align_window_s of unbroken stillness (the SensorParams still_* thresholds); any moving IMU
// sample restarts the window. It completes only once the Earth field is known: the setup's mag_ref_ned_ut when any
// component is non-zero, else the WMM at the first fix (locate).
class StationaryAlignment {
public:
    StationaryAlignment(const param::SensorParams& s, float gravity);
    // On the first GNSS fix: the field there, unless the setup overrides it.
    void locate(std::int32_t lat_e7, std::int32_t lon_e7);
    // Once per tick until it returns true: then out holds the alignment.
    bool feed(const SensorBus& bus, Alignment& out);
    Vec3 field() const { return field_; }  // NED, microtesla
    float declination() const { return declination_; }

private:
    param::SensorParams p_;
    float gravity_;
    Vec3 field_;
    float declination_;
    bool field_known_;
    std::uint64_t t_win_us_ = 0;  // first IMU sample of the still window
    Vec3 sum_f_{0.f, 0.f, 0.f};
    Vec3 sum_w_{0.f, 0.f, 0.f};
    Vec3 sum_m_{0.f, 0.f, 0.f};
    float sum_h_ = 0.f;
    int n_imu_ = 0, n_mag_ = 0, n_baro_ = 0;
};

class Ekf {
public:
    static constexpr int kN = 16;

    explicit Ekf(const param::EskfPriors& p = {}, const param::SensorParams& s = {});

    // Once per tick. Predicts on a fresh IMU sample, then fuses GNSS position and velocity, baro and mag as fresh.
    void update(const SensorBus& bus);

    // valid only after alignment. p_ned is relative to the first GNSS fix.
    State state() const;

    bool aligned() const { return aligned_; }
    Vec3 accel_bias() const { return {x_[10], x_[11], x_[12]}; }
    Vec3 gyro_bias() const { return {x_[13], x_[14], x_[15]}; }

private:
    void align(const Alignment& a);
    void predict(Vec3 am, Vec3 wm, float dt);
    void scalar_update(const float* h, float y, float r);
    void normalise_q();

    param::EskfPriors pri_;
    param::SensorParams sns_;
    float gravity_;   // m/s^2
    StationaryAlignment alignment_;  // also the Earth field reference and its declination
    LocalFrame frame_;  // GNSS origin: the first fix
    bool aligned_ = false;
    std::uint64_t t_us_ = 0;
    std::uint64_t last_imu_us_ = 0;
    float baro_h0_ = 0.f;
    Vec3 w_meas_{0.f, 0.f, 0.f};

    float x_[kN];      // p, v, q (w x y z), a_b, w_b
    float P_[kN][kN];  // covariance
    float M_[kN][kN];  // scratch (kept off the stack)
};

}  // namespace marv
