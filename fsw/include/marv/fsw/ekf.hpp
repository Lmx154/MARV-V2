// Full-state additive EKF, ported from the avionics toolbox:
//   filter        src/lib/calc/ekf-full.ts            (ekfInit, ekfPredict, ekfCorrect)
//   bus wiring    src/lib/sim/lab/blocks/estimator.ts (kalmanEstimator: fsw.ts's createEstimator for another core)
// 16 states x = [p, v, q (4), a_b, w_b], the quaternion corrected additively and re-normalised, the covariance left
// as it is. Same process noise, priors and measurements as Eskf (EskfParams). float only, fixed storage, no heap.
//
// This header also holds the stationary alignment and the magnetometer-heading and barometer front end that the
// EKF, UKF, Mahony and complementary estimators share: the toolbox initialises its estimators from truth, so they all
// take Eskf's alignment (eskf.cpp, Eskf::align) instead.
#pragma once

#include <cstdint>

#include <marv/fsw/contracts.hpp>
#include <marv/fsw/eskf.hpp>
#include <marv/fsw/geo.hpp>

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

// Eskf::accumulate_alignment and Eskf::align: IMU, magnetometer and barometer samples with
// align_skip_s <= t - t_first < align_skip_s + align_window_s are averaged.
class StationaryAlignment {
public:
    explicit StationaryAlignment(const EskfParams& p);
    // Once per tick until it returns true: then out holds the alignment.
    bool feed(const SensorBus& bus, Alignment& out);

private:
    float skip_s_, window_s_, declination_;
    bool started_ = false;
    std::uint64_t t_first_us_ = 0;
    Vec3 sum_f_{0.f, 0.f, 0.f};
    Vec3 sum_w_{0.f, 0.f, 0.f};
    Vec3 sum_m_{0.f, 0.f, 0.f};
    float sum_h_ = 0.f;
    int n_imu_ = 0, n_mag_ = 0, n_baro_ = 0;
};

class Ekf {
public:
    static constexpr int kN = 16;

    explicit Ekf(const EskfParams& p = {});

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

    EskfParams prm_;
    float mag_decl_;
    StationaryAlignment alignment_;
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
