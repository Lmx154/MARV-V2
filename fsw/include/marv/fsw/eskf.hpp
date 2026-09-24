// 15-state error-state Kalman filter (Solà 2017), ported from the avionics toolbox:
//   filter        src/lib/calc/eskf.ts         (eskfInit, eskfPredict, eskfCorrect)
//   bus wiring    src/lib/sim/sixdof/fsw.ts    (createEstimator: priors, pad gyro calibration, measurement order)
// Nominal state p, v (NED, m, m/s), q (body FRD -> NED), a_b, w_b (body); error state
// dx = [dp, dv, dtheta, da_b, dw_b] with the local (body-frame) attitude error dtheta.
// float only, fixed-size storage, no heap.
#pragma once

#include <cstdint>

#include <marv/fsw/contracts.hpp>
#include <marv/fsw/geo.hpp>
#include <marv/fsw/params.hpp>

namespace marv {

// Constants from the setup (params.def): the sensors' noise, reference field and alignment thresholds, the priors and
// the sensors' gravity.
//  - Process noise: white noise per IMU sample (eskf.ts: Q_v = sigma_accel^2 dt^2, Q_theta = sigma_gyro^2 dt^2); bias
//    random walks per sqrt(s) (Q = sigma^2 dt).
//  - Priors at alignment: sigma_p0, sigma_v0, sigma_theta0, sigma_ab0. The gyro-bias prior follows the pad
//    calibration: max(1e-4, sigma_gyro / sqrt(n)) over the n averaged samples.
//  - Stationary alignment: IMU, magnetometer and barometer samples are averaged over align_window_s of unbroken
//    stillness. An IMU sample is still when |norm(f) - g| < still_g_err and norm(w) < still_rate, and f and w lie within
//    still_accel_dev and still_gyro_dev of the window's mean so far. Any other sample discards the window (the start-up
//    contact transient too).
class Eskf {
public:
    static constexpr int kN = 15;

    explicit Eskf(const param::EskfPriors& p = {}, const param::SensorParams& s = {});

    // Once per tick. Predicts on a fresh IMU sample, then fuses baro and mag as fresh, GNSS position on the tick its
    // fix arrives and that fix's velocity on the next tick.
    void update(const SensorBus& bus);

    // valid only after alignment. p_ned is relative to the first GNSS fix (the vehicle is at rest at start).
    State state() const;

    bool aligned() const { return aligned_; }
    Vec3 accel_bias() const { return ab_; }
    Vec3 gyro_bias() const { return wb_; }

private:
    void accumulate_alignment(const SensorBus& bus);
    void align();
    void predict(Vec3 am, Vec3 wm, float dt);
    void scalar_update(const float* h, float y, float r);
    void inject_and_reset();
    void fuse_gnss_pos(const GnssSample& g);
    void fuse_gnss_vel();
    void fuse_baro(float pressure_pa);
    void fuse_mag(Vec3 m_frd);

    param::EskfPriors pri_;
    param::SensorParams sns_;
    float gravity_;   // m/s^2
    Vec3 mag_ref_;    // Earth field at the origin, NED, microtesla
    float mag_decl_;  // declination of the reference field, rad

    bool aligned_ = false;
    std::uint64_t t_win_us_ = 0;  // first IMU sample of the still window
    std::uint64_t t_us_ = 0;
    std::uint64_t last_imu_us_ = 0;

    // Alignment accumulators.
    Vec3 sum_f_{0.f, 0.f, 0.f};
    Vec3 sum_w_{0.f, 0.f, 0.f};
    Vec3 sum_m_{0.f, 0.f, 0.f};
    float sum_h_ = 0.f;
    int n_imu_ = 0;
    int n_mag_ = 0;
    int n_baro_ = 0;

    // Local frame: GNSS origin (first fix) and barometric reference height.
    LocalFrame frame_;
    float baro_h0_ = 0.f;

    // GNSS velocity of the last fix, fused on the tick after it.
    bool vel_pending_ = false;
    Vec3 vel_meas_{0.f, 0.f, 0.f};

    // Nominal state.
    Vec3 p_{0.f, 0.f, 0.f};
    Vec3 v_{0.f, 0.f, 0.f};
    Quat q_{1.f, 0.f, 0.f, 0.f};
    Vec3 ab_{0.f, 0.f, 0.f};
    Vec3 wb_{0.f, 0.f, 0.f};
    Vec3 w_meas_{0.f, 0.f, 0.f};

    float P_[kN][kN];  // error-state covariance
    float M_[kN][kN];  // scratch (kept off the stack)
    float dx_[kN];     // error-state correction accumulated by scalar_update, injected by inject_and_reset
};

}  // namespace marv
