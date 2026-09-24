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

namespace marv {

struct EskfParams {
    // Process noise: white noise per IMU sample (eskf.ts: Q_v = sigma_accel^2 dt^2, Q_theta = sigma_gyro^2 dt^2),
    // the madflight FC3's LSM6DSV (docs/datasheets/LSM6DSV.pdf, DS13476 Rev 2, p.10 Table 2) at 1 kHz: noise
    // density * sqrt(BW), BW = ODR/2 = 500 Hz (LPF1 cutoff in high-performance mode, p.33 note 1). The same
    // numbers drive the simulated IMU (sitl/gazebo/marv_quad.sdf). Bias random walks per sqrt(s) (Q = sigma^2 dt):
    // the datasheet gives no bias instability; these are the fsw.ts preset.
    float sigma_accel = 0.01316f;     // m/s^2: An 60 ug/sqrt(Hz) (high-performance) * sqrt(500 Hz) = 1.342 mg
    float sigma_gyro = 1.093e-3f;     // rad/s: Rn 2.8 mdps/sqrt(Hz) * sqrt(500 Hz) = 0.0626 dps
    float sigma_accel_walk = 0.002f;  // m/s^2/sqrt(s)
    float sigma_gyro_walk = 0.0002f;  // rad/s/sqrt(s)

    // Measurement noise (1 sigma). GNSS: ASSUMED u-blox M10 class (the FC3's receiver is an unspecified external
    // module). Baro: BMP580 (docs/datasheets/BMP580.pdf p.9 Table 1) 0.25 Pa RMS * dh/dP = 1 / (1.225 kg/m^3 * g).
    // Heading: MMC5603NJ (docs/datasheets/MMC5603NJ.pdf p.2) 2.0 mG RMS across the 0.2165 G horizontal field is
    // 0.0092 rad. A tilt error moves the tilt-compensated heading by tan(inclination) = 1.98 times itself; the heading
    // Jacobian carries that coupling, so R adds no allowance for it.
    float sigma_gnss_pos = 1.0f;    // m, horizontal
    float sigma_gnss_alt = 1.5f;    // m, vertical
    float sigma_gnss_vel = 0.05f;   // m/s
    float sigma_baro = 0.0208f;     // m
    float sigma_heading = 0.0092f;  // rad

    // Priors at alignment: fsw.ts createEstimator (sigmaP 1, sigmaV 1). The accel-bias prior is the LSM6DSV typical
    // zero-g level, 12 mg (p.10 Table 2); aligned with it, tilt is off by up to 12 mg / g = 0.012 rad and the
    // tilt-compensated heading by 1.98 times that, so sigma_theta0 is 0.025 rad (fsw.ts's 0.05 lets GNSS velocity noise
    // swing the tilt by 1.5 deg in the first second). The gyro-bias prior follows the pad calibration:
    // max(1e-4, sigma_gyro / sqrt(n)) over the n averaged samples.
    float sigma_p0 = 1.f;
    float sigma_v0 = 1.f;
    float sigma_theta0 = 0.025f;
    float sigma_ab0 = 0.1177f;

    // Environment the firmware knows: gravity of the world SDF, the Earth field (NED, microtesla) at the world origin.
    float gravity = 9.8066f;
    Vec3 mag_ref_ned_ut{21.62762f, 0.96861f, 42.91632f};

    // Stationary alignment: IMU, magnetometer and barometer samples are averaged over align_window_s of unbroken
    // stillness. An IMU sample is still when |norm(f) - g| and norm(w) are within what the turn-on biases allow
    // (3 sigma per axis of 12 mg and 1 dps, plus noise) and f and w lie within 6 per-sample sigmas (norm of three
    // axes) of the window's mean so far. Any other sample discards the window (the start-up contact transient too).
    float align_window_s = 1.0f;
    float still_g_err = 0.4f;       // m/s^2
    float still_rate = 0.1f;        // rad/s
    float still_accel_dev = 0.08f;  // m/s^2
    float still_gyro_dev = 0.0066f;  // rad/s
};

class Eskf {
public:
    static constexpr int kN = 15;

    explicit Eskf(const EskfParams& p = {});

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

    EskfParams prm_;
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
