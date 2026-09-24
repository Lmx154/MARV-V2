// 15-state error-state Kalman filter (Solà 2017), ported from the avionics toolbox:
//   filter        src/lib/calc/eskf.ts         (eskfInit, eskfPredict, eskfCorrect)
//   bus wiring    src/lib/sim/sixdof/fsw.ts    (createEstimator: priors, pad gyro calibration, measurement order)
// Nominal state p, v (NED, m, m/s), q (body FRD -> NED), a_b, w_b (body); error state
// dx = [dp, dv, dtheta, da_b, dw_b] with the local (body-frame) attitude error dtheta.
// float only, fixed-size storage, no heap.
#pragma once

#include <cstdint>

#include <marv/fsw/contracts.hpp>

namespace marv {

struct EskfParams {
    // Process noise: fsw.ts ESKF preset. White noise per IMU sample (eskf.ts: Q_v = sigma_accel^2 dt^2,
    // Q_theta = sigma_gyro^2 dt^2) and bias random walks per sqrt(s) (Q = sigma^2 dt).
    float sigma_accel = 0.05f;        // m/s^2
    float sigma_gyro = 0.003f;        // rad/s
    float sigma_accel_walk = 0.002f;  // m/s^2/sqrt(s)
    float sigma_gyro_walk = 0.0002f;  // rad/s/sqrt(s)

    // Measurement noise (1 sigma): fsw.ts SENSORS (gnssPosSigma, gnssVelSigma, baroSigma) and quad.ts magSigma.
    float sigma_gnss_pos = 1.0f;  // m
    float sigma_gnss_vel = 0.2f;  // m/s
    float sigma_baro = 0.3f;      // m
    float sigma_heading = 0.05f;  // rad

    // Priors at alignment: fsw.ts createEstimator (sigmaP 1, sigmaV 1, sigmaTheta 0.05, sigmaAb 0.3). The gyro-bias
    // prior follows its pad calibration: max(1e-4, sigma_gyro / sqrt(n)) over the n averaged samples.
    float sigma_p0 = 1.f;
    float sigma_v0 = 1.f;
    float sigma_theta0 = 0.05f;
    float sigma_ab0 = 0.3f;

    // Environment the firmware knows: gravity of the world SDF, the Earth field (NED, microtesla) at the world origin.
    float gravity = 9.8066f;
    Vec3 mag_ref_ned_ut{21.62762f, 0.96861f, 42.91632f};

    // Stationary alignment: IMU, magnetometer and barometer samples with align_skip_s <= t - t_first <
    // align_skip_s + align_window_s are averaged; the skip rejects the ground-contact transient at start.
    float align_skip_s = 0.1f;
    float align_window_s = 1.0f;
};

class Eskf {
public:
    static constexpr int kN = 15;

    explicit Eskf(const EskfParams& p = {});

    // Once per tick. Predicts on a fresh IMU sample, then fuses GNSS position and velocity, baro and mag as fresh.
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
    void fuse_gnss(const GnssSample& g);
    void fuse_baro(float pressure_pa);
    void fuse_mag(Vec3 m_frd);

    EskfParams prm_;
    float mag_decl_;  // declination of the reference field, rad

    bool started_ = false;
    bool aligned_ = false;
    std::uint64_t t_first_us_ = 0;
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
    bool have_origin_ = false;
    std::int32_t lat0_e7_ = 0;
    std::int32_t lon0_e7_ = 0;
    float alt0_ = 0.f;
    float m_per_e7_n_ = 0.f;
    float m_per_e7_e_ = 0.f;
    float baro_h0_ = 0.f;

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
