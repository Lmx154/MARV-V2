// Error-state unscented Kalman filter (USQUE-style, Crassidis & Markley 2003), ported from the avionics toolbox:
//   filter        src/lib/calc/ukf.ts                  (utWeights, ukfPredict, ukfCorrect)
//   bus wiring    src/lib/sim/lab/blocks/estimator.ts  (kalmanEstimator, the ukf block)
// Eskf's nominal state and 15-state error covariance [dp, dv, dtheta, da_b, dw_b]; the Jacobians replaced by 31 sigma
// points retracted onto the nominal state with the exponential map. Same process noise, priors and measurements as
// Eskf (EskfParams). float only, fixed storage, no heap.
#pragma once

#include <cstdint>

#include <marv/fsw/contracts.hpp>
#include <marv/fsw/ekf.hpp>
#include <marv/fsw/eskf.hpp>
#include <marv/fsw/geo.hpp>

namespace marv {

// Scaled unscented transform (Wan & van der Merwe 2000); ukf.ts UT_DEFAULT.
struct UtParams {
    float alpha = 1e-3f;
    float beta = 2.f;
    float kappa = 0.f;
};

class Ukf {
public:
    static constexpr int kN = 15;

    explicit Ukf(const EskfParams& p = {}, const UtParams& ut = {});

    // Once per tick. Predicts on a fresh IMU sample, then fuses GNSS position and velocity, baro and mag as fresh.
    void update(const SensorBus& bus);

    // valid only after alignment. p_ned is relative to the first GNSS fix.
    State state() const;

    bool aligned() const { return aligned_; }
    Vec3 accel_bias() const { return ab_; }
    Vec3 gyro_bias() const { return wb_; }
    // Sigma-point spreads that met a non-positive-definite covariance and were skipped.
    std::uint32_t cholesky_failures() const { return chol_fail_; }

    enum class Meas : std::uint8_t { kPosition, kVelocity, kAltitude, kHeading };

private:
    void align(const Alignment& a);
    bool cholesky();
    void predict(Vec3 am, Vec3 wm, float dt);
    void correct(Meas m, const float* y0, float sigma);

    EskfParams prm_;
    float mag_decl_;
    float gamma_, wm0_, wc0_, wi_;
    StationaryAlignment alignment_;
    LocalFrame frame_;
    bool aligned_ = false;
    std::uint64_t t_us_ = 0;
    std::uint64_t last_imu_us_ = 0;
    float baro_h0_ = 0.f;
    Vec3 w_meas_{0.f, 0.f, 0.f};
    std::uint32_t chol_fail_ = 0;

    // Nominal state.
    Vec3 p_{0.f, 0.f, 0.f};
    Vec3 v_{0.f, 0.f, 0.f};
    Quat q_{1.f, 0.f, 0.f, 0.f};
    Vec3 ab_{0.f, 0.f, 0.f};
    Vec3 wb_{0.f, 0.f, 0.f};

    float P_[kN][kN];  // error-state covariance
    float L_[kN][kN];  // scratch: chol(P), lower triangular
    float C_[kN][kN];  // scratch: accumulated sigma-point covariance
    float Pxz_[kN][3];  // scratch: cross covariance of a correction
    float K_[kN][3];    // scratch: its gain
};

}  // namespace marv
