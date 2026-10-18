#pragma once

// The attitude and gyro-bias error-state Kalman filter (quad spec section 4.0 M4, decision 0055): in SHADOW beside the Mahony
// filter that flies (mahony.hpp); nothing in the flight composition reads it. Pure, like Mahony: stepped once per IMU tick
// with plain inputs, it reads no clock, no HAL and no parameter; the caller passes the configuration.
//
// Reference: J. Sola, "Quaternion kinematics for the error-state Kalman filter", arXiv:1711.02508v1 (2017); equation numbers
// below are that version's. Structure and float robustness practices of PX4's EKF2 (BSD-3; quad spec section 6.3.2: error-state
// attitude, sequential scalar fusion with no matrix inverse, Joseph-form update with symmetric write-back, per-state variance
// floors and ceilings, innovation gating), PX4-Autopilot ddcaad6601be20453ec6acd0f08e567af3a7e1bb, read 2026-10-18 (INFERRED
// from a source read; no code copied).
//
// Frames and units (core section 3) as Mahony: body FRD, world NED, q the Hamilton unit quaternion [w, x, y, z] body -> NED,
// normalised every step, canonical (w >= 0). Gyro rad/s, specific force m/s^2, both body; dt from the caller's microseconds.
//
// States. Nominal: q and the gyro bias b (body, rad/s; the gyro reads omega + b). Error: dx = [dtheta, db] (6), dtheta the
// GLOBAL angular error in NED (Sola section 7: q_t = Exp(dtheta) (x) q, eq. (303)), so heading is dtheta_z alone and the
// gravity measurement never touches it (EKF2 also keeps heading as the third angle-error state). P is the 6 x 6 covariance of
// dx, stored full and written back symmetric after every operation.
//
// Prediction, every step with a finite gyro (Sola 7.2):
//   nominal   q <- q (x) exp(1/2 (w - b) dt)                       (260c; the exact exponential, as Mahony)
//             b <- b                                                (260e)
//   error     dtheta <- dtheta - R db dt + theta_i                  (310c; R = R{q} before the step)
//             db     <- db + omega_i                                (310e)
//   P <- F P F^T + Fi Qi Fi^T, F = [[I, -R dt], [0, I]]             (311, 312, 269)
//   Qi = diag(Theta_i I, Omega_i I). Theta_i = sigma_d^2 dt^2 (263) with sigma_d^2 = N_g^2 / (2 dt), the per-sample variance of
//   white noise of one-sided density N_g over the Nyquist band (the plant's rule, sim/plant/src/imu_model.hpp lines 16-26), so
//   Theta_i = N_g^2 dt / 2. Omega_i = K_g^2 dt (265), K_g the bias random walk (rad/s/sqrt(s)).
// With P = [[A, B], [B^T, C]] (3 x 3 blocks) and M = -R dt: A <- A + M B^T + B M^T + M C M^T + Theta_i I, B <- B + M C,
// C <- C + Omega_i I.
//
// Measurement, the accelerometer as a gravity direction (EKF2 gravity_fusion.cpp, "a la complementary filter"), once per window
// of accel_fusion_divisor predicted steps (EKF2 fuses at its filter update period EKF2_PREDICT_US, from the IMU delta velocity
// summed over it, gravity_fusion.cpp line 52; here every step is still predicted). Each step whose specific force f is finite and
// healthy (below) adds f dt to the window's sum S, after S is turned into the step's body axes: the body turns by
// theta = (w - b) dt over the step, so S <- S - theta x S (first order in theta; EKF2's down-sampler likewise rotates each delta
// velocity into one frame) then S <- S + f dt, and the window's span T <- T + dt. At the window's last step the measurement is the
// mean specific force S / T, y = S / |S|, and h(q) = -R^T e_z = -(R20, R21, R22) (body; level, (0, 0, -1)). With
// R_t = (I + [dtheta]x) R, h(x_t) = h - R^T [e_z]x dtheta, so H = H_x X_dx (278) has the rows H_k = (-R1k, R0k, 0, 0, 0, 0),
// k = x, y, z (no bias term). Residual r_k = y_k - h_k.
// Noise: white noise of one-sided density N has the variance N^2 / (2 dt) per sample over the Nyquist band (the plant's rule,
// above), so the time-weighted mean S / T of the window's samples has the variance sum(dt^2 N^2 / (2 dt)) / T^2 = N^2 / (2 T); per
// axis of the direction, (N_a / g)^2 / (2 T), N_a the accelerometer's density and g standard gravity. Averaging over the window
// divides the single sample's variance by the window's sample count. Plus the model error, which no averaging reduces: the
// specific force is gravity plus the vehicle's linear acceleration and vibration, not gravity alone. EKF2 sets the variance of
// every gravity-direction component to max(EKF2_GRAV_NOISE^2, 0.01^2), EKF2_GRAV_NOISE the "accelerometer measurement gaussian
// noise" in m/s^2, default 1.0 (gravity_fusion.cpp line 53, common.h line 483) - it does not divide by g, though the measurement
// is the unit vector y. Here the stated rule is applied: an error e (m/s^2) in the specific force turns the direction by e / |f|,
// |f| = g about hover, so the model-error variance per axis is (accel_model_noise / g)^2, accel_model_noise = EKF2_GRAV_NOISE (0
// switches the term off), added to the sensor term: var = (N_a / g)^2 / (2 T) + (accel_model_noise / g)^2. Without it the gate is
// set by the sensor noise alone (5 sigma = 0.12 deg for the bench's 75-sample window) and a manoeuvre locks the accelerometer
// out until the rejection timeout.
// Gating, in this order:
//   1. health: a step's f enters S only if |f| lies in the open interval (accel_healthy_min g, accel_healthy_max g) (Mahony's
//      window; EKF2 the same 0.9 g - 1.1 g on its filtered accelerometer, gravity_fusion.cpp lines 55-60) and is finite; the window
//      fuses only if T > 0 and |S / T| lies in the same interval;
//   2. innovation: with the prior P, s_k = H_k P H_k^T + var, and every k must pass r_k^2 <= gate^2 s_k, gate in standard
//      deviations; one failing axis rejects the window (EKF2 updateAidSourceStatus, ekf.h lines 1153-1229: test ratio
//      r^2 / (gate^2 s) > 1 rejects, any rejected axis rejects the observation);
//   3. reset on rejection timeout: if the spans T of the windows rejected by 2 since the last fusion sum to at least
//      accel_reset_us (an unhealthy window neither adds nor clears), the tilt is reset to the window's direction (yaw kept) and the
//      angle block of P to its initial value with its cross terms 0 (EKF2's pattern of resetting a state to its measurement after a
//      fusion timeout, common.h line 512 reset_timeout_max; without it a filter whose P has become overconfident would reject a
//      true accelerometer forever).
// S and T are cleared at the window's last step whatever its outcome.
// A passing window is fused as three sequential scalar updates, k = x, y, z, each with its residual and row recomputed from
// the nominal state after the previous one (EKF2 gravity_fusion.cpp lines 84-120).
//
// The scalar update, update_scalar(h, r, var) (the primitive a magnetometer, GNSS or barometer update will call):
//   u = P h^T, s = h u + var (no inverse), K = u / s                                     (274)
//   K's inhibited components are removed (below)
//   P <- (I - K h) P (I - K h)^T + K var K^T, the Joseph form (275-276, footnote 26), which holds for any gain K, the inhibited
//        one included (EKF2 the same, ekf_helper.cpp lines 1040-1085), evaluated as W = P - K u^T,
//        P <- W - (W h^T) K^T + var K K^T, then written back symmetric (P_ij = P_ji = their mean)
//   dx = K r                                                                             (275)
//   injection  q <- Exp(dtheta) (x) q, b <- b + db                                       (314c, 314e)
//   reset      dx <- 0, P <- G P G^T, G = blockdiag(I + [1/2 dtheta]x, I)                (315-317)
// then the variance limits. Variance floors and ceilings (EKF2 covariance.cpp constrainStateVariances, lines 249-288; one
// pair for the angle states, one for the bias states): a diagonal below its floor is raised to it (P stays positive
// semidefinite); one above its ceiling has its row and column scaled by sqrt(ceiling / P_ii), which sets it to the ceiling and
// keeps every correlation coefficient (EKF2 shrinks it by a zero-innovation fusion instead, for the same reason: clipping alone
// would raise the correlations). They apply after every prediction and every scalar update.
//
// Inhibited gains. Nothing here measures heading, so two directions of the error state are unobservable: the heading dtheta_z, and
// the bias component along gravity, b . d with d = R^T e_z the body's down direction (level, the F term -R db dt maps it onto
// dtheta_z alone, which no row of H reads). Every scalar update zeroes the heading's gain and removes the gain's bias component
// along d, K_b <- K_b - d (d . K_b), d from the nominal q at the update: EKF2 clearInhibitedStateKalmanGains (ekf_helper.cpp lines
// 1237-1252) zeroes the heading gain while heading is not observable and the gain of every inhibited bias axis; its per-axis rule
// (updateIMUBiasInhibit, lines 999-1027) treats a bias as unobservable or observable by its body axis's alignment with gravity
// (|R(2, i)| > cos 15 deg, the accelerometer bias); the projection is the same inhibition for the one direction that is unobservable,
// at any attitude. Why (the T2 bench replay, M4): the predictions and the reset Jacobian correlate the large heading variance with
// the tilt (G's [dtheta/2]x terms and -R dt's coupling through the board's own small tilt), so every tilt residual moved the heading,
// and the heading is correlated with b . d through -R db dt (correlation coefficient near -1 after a few seconds); and each update
// linearises at a slightly different d (the estimate's tilt noise), so the filter takes information on b . d that the data does not
// hold. Level and still, with the bias prior of EKF2's uncalibrated default, b_z wandered to 5 deg/s in 47 s; with the heading
// decorrelated instead (EKF2's uncorrelateAndLimitHeadingCovariance) the second path alone took it to 66 deg/s. A heading update
// (magnetometer, GNSS yaw) will have to lift both inhibitions.
//
// Initialisation and validity, as Mahony (decision 0042 item 5): the first healthy sample (|f| in the open health interval, gyro
// finite) sets q to its tilt with yaw 0, b to 0 and P to diag(angle_sigma_init^2 I, gyro_bias_sigma_init^2 I); until then
// step() changes nothing. b = 0 is the calibrated gyro's: the caller steps the filter on the boot-calibrated gyro (gyro_cal.hpp)
// from its first sample, and gyro_bias_sigma_init is the calibration's residual (the register's rule). valid() is true iff
// initialised and q is finite and unit. A non-finite gyro, or a step after which q, b or P is not finite, clears the state
// (q identity, b 0, P 0, not initialised, the window empty) until the next healthy sample; a non-finite or zero accelerometer
// vector only stays out of the window (the gyro-only step).

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

#include "marv/prim/constants.hpp"
#include "marv/prim/mat.hpp"
#include "marv/prim/quat.hpp"
#include "marv/prim/vec.hpp"

namespace marv::estimator {

// Error-state layout: dtheta (NED, rad) first, then db (body, rad/s).
inline constexpr std::size_t kEskfAngle = 0;
inline constexpr std::size_t kEskfBias = prim::kSpatialDim;
inline constexpr std::size_t kEskfStates = 2 * prim::kSpatialDim;
inline constexpr std::size_t kEskfHeading = kEskfAngle + 2;  // dtheta_z, about NED down

template <class T>
using EskfRow = prim::Vec<T, kEskfStates>;
template <class T>
using EskfCov = prim::Mat<T, kEskfStates, kEskfStates>;

template <class T>
struct EskfConfig {
  T gyro_noise_density{};      // N_g, rad/s/sqrt(Hz), one-sided
  T gyro_bias_random_walk{};   // K_g, rad/s/sqrt(s)
  T accel_noise_density{};     // N_a, m/s^2/sqrt(Hz), one-sided
  T angle_sigma_init{};        // rad, each angle-error state at initialisation and after a tilt reset
  T gyro_bias_sigma_init{};    // rad/s, each bias state at initialisation
  T accel_gate{};              // innovation gate, standard deviations
  std::uint64_t accel_reset_us = 0;  // span of rejected healthy samples after which the tilt is reset, microseconds
  T angle_var_min{};           // rad^2
  T angle_var_max{};           // rad^2
  T gyro_bias_var_min{};       // (rad/s)^2
  T gyro_bias_var_max{};       // (rad/s)^2
  T accel_healthy_min{};       // lower bound of the healthy specific-force norm, in g (open)
  T accel_healthy_max{};       // upper bound of the healthy specific-force norm, in g (open)
  std::uint32_t accel_fusion_divisor = 1;  // predicted steps per gravity fusion window (0 acts as 1)
  T accel_model_noise{};       // m/s^2, the specific force's model error (linear acceleration, vibration); 0 = none
};

// What the latest step did with the accelerometer window (NotUsed on every step that does not close a window).
enum class EskfAccel : std::uint8_t { NotUsed, Fused, Gated, Reset };

// Counts since init().
struct EskfCounts {
  std::uint32_t fused = 0;   // windows fused
  std::uint32_t gated = 0;   // healthy windows rejected by the innovation gate (a reset window included)
  std::uint32_t resets = 0;  // tilt resets after the rejection timeout
};

template <class T>
class Eskf {
 public:
  void init(const EskfConfig<T>& config) noexcept {
    cfg_ = config;
    counts_ = EskfCounts{};
    reset();
  }

  void reset() noexcept {
    q_ = prim::Quat<T>::identity();
    bias_ = prim::Vec3<T>();
    p_ = EskfCov<T>();
    initialised_ = false;
    gated_us_ = 0;
    clear_window();
    last_ = EskfAccel::NotUsed;
  }

  // gyro rad/s and specific force m/s^2, both body FRD; dt_us the time since the previous step, in microseconds.
  void step(const prim::Vec3<T>& gyro, const prim::Vec3<T>& accel, std::uint32_t dt_us) noexcept {
    if (!finite(gyro)) {
      reset();
      return;
    }
    last_ = EskfAccel::NotUsed;
    const T n2 = accel.dot(accel);
    if (!initialised_) {
      if (healthy(n2)) {
        initialise(accel);
      }
      return;
    }
    const T dt = static_cast<T>(dt_us) / static_cast<T>(prim::kMicrosecondsPerSecond);
    predict(gyro, dt);
    if (healthy(n2) && dt > T(0)) {
      window_ = window_ + accel * dt;
      window_us_ += dt_us;
    }
    if (++window_steps_ >= cfg_.accel_fusion_divisor) {
      if (window_us_ > 0 && state_finite()) {
        fuse_window();
      }
      clear_window();
    }
    if (!state_finite()) {
      reset();
    }
  }

  // H P H^T + variance for the row h over the error state.
  [[nodiscard]] T innovation_variance(const EskfRow<T>& h, T variance) const noexcept { return h.dot(p_ * h) + variance; }

  // One scalar measurement: residual = z - h(x) at the current nominal state, h its row over [dtheta, db], variance its noise.
  // Joseph-form update, injection, reset and variance limits (see above). False, and nothing changed, if the innovation
  // variance is not finite and positive or the residual is not finite. Not gated here: the caller gates.
  bool update_scalar(const EskfRow<T>& h, T residual, T variance) noexcept {
    const EskfRow<T> u = p_ * h;
    const T s = h.dot(u) + variance;
    if (!(std::isfinite(s) && s > T(0) && std::isfinite(residual))) {
      return false;
    }
    EskfRow<T> k = u * (T(1) / s);
    inhibit(k);
    EskfCov<T> w;
    for (std::size_t i = 0; i < kEskfStates; ++i) {
      for (std::size_t j = 0; j < kEskfStates; ++j) {
        w(i, j) = p_(i, j) - k[i] * u[j];
      }
    }
    const EskfRow<T> wh = w * h;
    for (std::size_t i = 0; i < kEskfStates; ++i) {
      for (std::size_t j = 0; j < kEskfStates; ++j) {
        p_(i, j) = w(i, j) - wh[i] * k[j] + variance * k[i] * k[j];
      }
    }
    symmetrise();
    inject(k * residual);
    limit_variances();
    return true;
  }

  [[nodiscard]] const prim::Quat<T>& quat() const noexcept { return q_; }
  [[nodiscard]] const prim::Vec3<T>& bias() const noexcept { return bias_; }  // rad/s, the gyro reads omega + bias
  [[nodiscard]] const EskfCov<T>& covariance() const noexcept { return p_; }
  [[nodiscard]] bool initialised() const noexcept { return initialised_; }
  // q was initialised from a healthy sample and is finite and unit.
  [[nodiscard]] bool valid() const noexcept {
    return initialised_ && std::isfinite(q_.w) && std::isfinite(q_.x) && std::isfinite(q_.y) && std::isfinite(q_.z);
  }
  [[nodiscard]] EskfAccel accel_status() const noexcept { return last_; }
  [[nodiscard]] const EskfCounts& counts() const noexcept { return counts_; }

 private:
  [[nodiscard]] static bool finite(const prim::Vec3<T>& v) noexcept {
    return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
  }

  // The squared specific-force norm n2 (m^2/s^4) lies in the open interval (min g, max g)^2; false for a non-finite n2.
  [[nodiscard]] bool healthy(T n2) const noexcept {
    const T g = static_cast<T>(prim::kStandardGravity);
    const T lo = cfg_.accel_healthy_min * g;
    const T hi = cfg_.accel_healthy_max * g;
    return std::isfinite(n2) && lo * lo < n2 && n2 < hi * hi;
  }

  [[nodiscard]] bool state_finite() const noexcept {
    if (!(std::isfinite(q_.w) && std::isfinite(q_.x) && std::isfinite(q_.y) && std::isfinite(q_.z) && finite(bias_))) {
      return false;
    }
    for (const T& e : p_.e) {
      if (!std::isfinite(e)) {
        return false;
      }
    }
    return true;
  }

  void initialise(const prim::Vec3<T>& f) noexcept {
    q_ = from_tilt(f, T(0));
    bias_ = prim::Vec3<T>();
    p_ = EskfCov<T>();
    const T a0 = cfg_.angle_sigma_init * cfg_.angle_sigma_init;
    const T b0 = cfg_.gyro_bias_sigma_init * cfg_.gyro_bias_sigma_init;
    for (std::size_t i = 0; i < prim::kSpatialDim; ++i) {
      p_(kEskfAngle + i, kEskfAngle + i) = a0;
      p_(kEskfBias + i, kEskfBias + i) = b0;
    }
    initialised_ = true;
    gated_us_ = 0;
    clear_window();
  }

  void predict(const prim::Vec3<T>& gyro, T dt) noexcept {
    const prim::Mat3<T> r = q_.to_rotation_matrix();
    const prim::Vec3<T> theta = (gyro - bias_) * dt;
    const prim::Quat<T> next = q_ * exp_half(theta);
    const T next_n2 = next.norm2();
    if (!(std::isfinite(next_n2) && next_n2 > T(0))) {
      q_.w = std::numeric_limits<T>::quiet_NaN();  // the step's finiteness check clears the state
      return;
    }
    q_ = next.normalized().canonical();
    window_ = window_ - theta.cross(window_);  // the window's sum into this step's body axes

    const prim::Mat3<T> a = block(kEskfAngle, kEskfAngle);
    const prim::Mat3<T> b = block(kEskfAngle, kEskfBias);
    const prim::Mat3<T> c = block(kEskfBias, kEskfBias);
    const prim::Mat3<T> m = r * (-dt);
    const prim::Mat3<T> mc = m * c;
    const prim::Mat3<T> bmt = b * m.transpose();
    const prim::Mat3<T> b_next = b + mc;
    prim::Mat3<T> a_next = a + bmt + bmt.transpose() + mc * m.transpose();
    prim::Mat3<T> c_next = c;
    const T theta_i = cfg_.gyro_noise_density * cfg_.gyro_noise_density * dt * T(0.5);
    const T omega_i = cfg_.gyro_bias_random_walk * cfg_.gyro_bias_random_walk * dt;
    for (std::size_t i = 0; i < prim::kSpatialDim; ++i) {
      a_next(i, i) += theta_i;
      c_next(i, i) += omega_i;
    }
    set_block(kEskfAngle, kEskfAngle, a_next);
    set_block(kEskfAngle, kEskfBias, b_next);
    set_block(kEskfBias, kEskfAngle, b_next.transpose());
    set_block(kEskfBias, kEskfBias, c_next);
    symmetrise();
    limit_variances();
  }

  // Row k of H and the residual y_k - h_k at the current nominal state.
  void gravity_row(const prim::Vec3<T>& y, std::size_t k, EskfRow<T>& h, T& residual) const noexcept {
    const prim::Mat3<T> r = q_.to_rotation_matrix();
    h = EskfRow<T>();
    h[kEskfAngle] = -r(1, k);
    h[kEskfAngle + 1] = r(0, k);
    residual = y[k] + r(2, k);
  }

  // The window's mean specific force as a gravity direction: health, gate, timeout, three scalar updates.
  void fuse_window() noexcept {
    const T span = static_cast<T>(window_us_) / static_cast<T>(prim::kMicrosecondsPerSecond);
    const prim::Vec3<T> mean = window_ * (T(1) / span);
    const T n2 = mean.dot(mean);
    if (!healthy(n2)) {
      return;
    }
    const prim::Vec3<T> y = mean * (T(1) / std::sqrt(n2));
    const T n = cfg_.accel_noise_density / static_cast<T>(prim::kStandardGravity);
    const T m = cfg_.accel_model_noise / static_cast<T>(prim::kStandardGravity);
    const T variance = n * n / (T(2) * span) + m * m;
    const T gate2 = cfg_.accel_gate * cfg_.accel_gate;
    bool pass = true;
    for (std::size_t k = 0; k < prim::kSpatialDim; ++k) {
      EskfRow<T> h;
      T residual{};
      gravity_row(y, k, h, residual);
      if (!(residual * residual <= gate2 * innovation_variance(h, variance))) {
        pass = false;
      }
    }
    if (!pass) {
      ++counts_.gated;
      last_ = EskfAccel::Gated;
      gated_us_ += window_us_;
      if (gated_us_ >= cfg_.accel_reset_us) {
        reset_tilt(y);
      }
      return;
    }
    for (std::size_t k = 0; k < prim::kSpatialDim; ++k) {
      EskfRow<T> h;
      T residual{};
      gravity_row(y, k, h, residual);
      static_cast<void>(update_scalar(h, residual, variance));
    }
    ++counts_.fused;
    last_ = EskfAccel::Fused;
    gated_us_ = 0;
  }

  void clear_window() noexcept {
    window_ = prim::Vec3<T>();
    window_us_ = 0;
    window_steps_ = 0;
  }

  // The inhibited gains (see above): no heading gain, and no bias gain along the body's down direction d = R^T e_z.
  void inhibit(EskfRow<T>& k) const noexcept {
    k[kEskfHeading] = T(0);
    const prim::Vec3<T> d(T(2) * (q_.x * q_.z - q_.w * q_.y), T(2) * (q_.y * q_.z + q_.w * q_.x),
                          T(1) - T(2) * (q_.x * q_.x + q_.y * q_.y));
    const T along = k[kEskfBias] * d[0] + k[kEskfBias + 1] * d[1] + k[kEskfBias + 2] * d[2];
    for (std::size_t i = 0; i < prim::kSpatialDim; ++i) {
      k[kEskfBias + i] -= along * d[i];
    }
  }

  // The tilt of the specific-force direction y with the current yaw; the angle block of P back to its initial value.
  void reset_tilt(const prim::Vec3<T>& y) noexcept {
    const T yaw = std::atan2(T(2) * (q_.w * q_.z + q_.x * q_.y), T(1) - T(2) * (q_.y * q_.y + q_.z * q_.z));
    q_ = from_tilt(y, yaw);
    const T a0 = cfg_.angle_sigma_init * cfg_.angle_sigma_init;
    for (std::size_t i = 0; i < prim::kSpatialDim; ++i) {
      for (std::size_t j = 0; j < kEskfStates; ++j) {
        p_(kEskfAngle + i, j) = T(0);
        p_(j, kEskfAngle + i) = T(0);
      }
      p_(kEskfAngle + i, kEskfAngle + i) = a0;
    }
    ++counts_.resets;
    last_ = EskfAccel::Reset;
    gated_us_ = 0;
  }

  // Injection of dx into the nominal state and the reset of P (Sola 314c, 314e, 316, 317).
  void inject(const EskfRow<T>& dx) noexcept {
    const prim::Vec3<T> dtheta(dx[kEskfAngle], dx[kEskfAngle + 1], dx[kEskfAngle + 2]);
    const prim::Quat<T> next = exp_half(dtheta) * q_;
    const T next_n2 = next.norm2();
    if (!(std::isfinite(next_n2) && next_n2 > T(0))) {
      q_.w = std::numeric_limits<T>::quiet_NaN();  // valid() is false; the step's finiteness check clears the state
      return;
    }
    q_ = next.normalized().canonical();
    bias_ = bias_ + prim::Vec3<T>(dx[kEskfBias], dx[kEskfBias + 1], dx[kEskfBias + 2]);
    const prim::Mat3<T> e = skew(dtheta * T(0.5));
    const prim::Mat3<T> a = block(kEskfAngle, kEskfAngle);
    const prim::Mat3<T> b = block(kEskfAngle, kEskfBias);
    const prim::Mat3<T> ea = e * a;
    const prim::Mat3<T> a_next = a + ea + ea.transpose() + ea * e.transpose();
    const prim::Mat3<T> b_next = b + e * b;
    set_block(kEskfAngle, kEskfAngle, a_next);
    set_block(kEskfAngle, kEskfBias, b_next);
    set_block(kEskfBias, kEskfAngle, b_next.transpose());
    symmetrise();
  }

  void limit_variances() noexcept {
    for (std::size_t i = 0; i < prim::kSpatialDim; ++i) {
      limit(kEskfAngle + i, cfg_.angle_var_min, cfg_.angle_var_max);
      limit(kEskfBias + i, cfg_.gyro_bias_var_min, cfg_.gyro_bias_var_max);
    }
  }

  void limit(std::size_t i, T lo, T hi) noexcept {
    if (p_(i, i) < lo) {
      p_(i, i) = lo;
    } else if (p_(i, i) > hi) {
      const T f = std::sqrt(hi / p_(i, i));
      for (std::size_t j = 0; j < kEskfStates; ++j) {
        if (j != i) {
          p_(i, j) *= f;
          p_(j, i) *= f;
        }
      }
      p_(i, i) = hi;
    }
  }

  void symmetrise() noexcept {
    for (std::size_t i = 0; i < kEskfStates; ++i) {
      for (std::size_t j = i + 1; j < kEskfStates; ++j) {
        const T m = (p_(i, j) + p_(j, i)) * T(0.5);
        p_(i, j) = m;
        p_(j, i) = m;
      }
    }
  }

  [[nodiscard]] prim::Mat3<T> block(std::size_t r0, std::size_t c0) const noexcept {
    prim::Mat3<T> m;
    for (std::size_t i = 0; i < prim::kSpatialDim; ++i) {
      for (std::size_t j = 0; j < prim::kSpatialDim; ++j) {
        m(i, j) = p_(r0 + i, c0 + j);
      }
    }
    return m;
  }

  void set_block(std::size_t r0, std::size_t c0, const prim::Mat3<T>& m) noexcept {
    for (std::size_t i = 0; i < prim::kSpatialDim; ++i) {
      for (std::size_t j = 0; j < prim::kSpatialDim; ++j) {
        p_(r0 + i, c0 + j) = m(i, j);
      }
    }
  }

  // [v]x, the matrix of the cross product v x.
  [[nodiscard]] static prim::Mat3<T> skew(const prim::Vec3<T>& v) noexcept {
    prim::Mat3<T> m;
    m(0, 1) = -v[2];
    m(0, 2) = v[1];
    m(1, 0) = v[2];
    m(1, 2) = -v[0];
    m(2, 0) = -v[1];
    m(2, 1) = v[0];
    return m;
  }

  // exp(1/2 phi) for the rotation vector phi: (cos(|phi|/2), phi sin(|phi|/2) / |phi|).
  [[nodiscard]] static prim::Quat<T> exp_half(const prim::Vec3<T>& phi) noexcept {
    const T angle = phi.norm();
    if (!(angle > T(0))) {
      return prim::Quat<T>::identity();
    }
    const T half = angle * T(0.5);
    const T k = std::sin(half) / angle;
    return prim::Quat<T>(std::cos(half), phi[0] * k, phi[1] * k, phi[2] * k);
  }

  // The tilt of the specific force f (roll phi = atan2(-fy, -fz), pitch theta = atan2(fx, sqrt(fy^2 + fz^2)), as Mahony) with
  // the given yaw: q = qz(yaw) (x) qy(theta) (x) qx(phi), for R = Rz Ry Rx.
  [[nodiscard]] static prim::Quat<T> from_tilt(const prim::Vec3<T>& f, T yaw) noexcept {
    const T roll = std::atan2(-f[1], -f[2]);
    const T pitch = std::atan2(f[0], std::sqrt(f[1] * f[1] + f[2] * f[2]));
    const prim::Quat<T> qz = prim::Quat<T>::from_axis_angle(prim::Vec3<T>(T(0), T(0), T(1)), yaw);
    const prim::Quat<T> qy = prim::Quat<T>::from_axis_angle(prim::Vec3<T>(T(0), T(1), T(0)), pitch);
    const prim::Quat<T> qx = prim::Quat<T>::from_axis_angle(prim::Vec3<T>(T(1), T(0), T(0)), roll);
    return (qz * qy * qx).normalized().canonical();
  }

  EskfConfig<T> cfg_{};
  prim::Quat<T> q_ = prim::Quat<T>::identity();
  prim::Vec3<T> bias_{};
  EskfCov<T> p_{};
  bool initialised_ = false;
  std::uint64_t gated_us_ = 0;
  prim::Vec3<T> window_{};          // sum of f dt over the window's healthy steps, in the latest step's body axes, m/s
  std::uint64_t window_us_ = 0;     // the window's span T, the sum of those steps' dt, microseconds
  std::uint32_t window_steps_ = 0;  // predicted steps in the window
  EskfAccel last_ = EskfAccel::NotUsed;
  EskfCounts counts_{};
};

}  // namespace marv::estimator
