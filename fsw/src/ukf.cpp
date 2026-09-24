// Port of avionics-toolbox src/lib/calc/ukf.ts wired as src/lib/sim/lab/blocks/estimator.ts kalmanEstimator (ukf block).
// Deviations from the TypeScript:
//  - float throughout. With the default alpha = 1e-3 the sigma points sit 0.004 sigma from the centre, far below the
//    float resolution of an absolute state (a gyro-bias point moves the quaternion by ~1e-10 per sample). ukf.ts already
//    takes every sum as a deviation from the centre point; here each sigma point's deviation is also *computed* as a
//    deviation, algebraically the same boxminus(moved[0], moved[i]) but without forming the absolute state:
//      dp  = dp_i + dt dv_i + dt^2/2 da,  dv = dv_i + dt da,  da = R0 ((R(dq_i) - I) f - R(dq_i) da_b_i),
//      dq  = q0'^-1 (x) q_i' = exp(R(e0)^T dtheta_i) (x) (e0^-1 (x) e_i),  e = exp((w_m - w_b) dt),
//    with R(dq) - I and e0^-1 (x) e_i expanded so that no two numbers of order one are subtracted. The heading
//    deviation yaw(q0 (x) dq_i) - yaw(q0) is taken the same way; the linear measurements' deviations are the sigma
//    point's own components (what measure() - Z[0] gives exactly).
//  - Sigma points are generated, pushed and accumulated one at a time (no 31-point storage); the covariance sum is the
//    moments() form, sum w_i d d^T + (w_c0 - w_m0 - 1) m m^T.
//  - Vector measurements (GNSS position, velocity) go through the transform as 3-vectors as in ukfCorrect; S (3x3) is
//    solved by Cholesky instead of inverted.
//  - A covariance that is not positive definite skips the step (predict: nominal kinematics and + Q; correct: the
//    measurement) and counts it, where ukf.ts throws.
//  - Heading: the magnetometer vector tilt-compensated with the estimated attitude, as eskf.cpp.
//  - Initialisation is Eskf's stationary alignment with kalmanEstimator's priors (eskfInit sigmas, theta 0.05 rad on
//    every axis, no injected heading error). Gravity aiding is not ported.
#include <marv/fsw/ukf.hpp>

#include <cmath>

#include <marv/fsw/math.hpp>

namespace marv {

namespace {

constexpr int IP = 0;
constexpr int IV = 3;
constexpr int ITH = 6;
constexpr int IAB = 9;
constexpr int IWB = 12;
constexpr int N = Ukf::kN;

float wrap(float a) { return std::atan2(std::sin(a), std::cos(a)); }

Vec3 v3(const float* c) { return {c[0], c[1], c[2]}; }

// (R(q) - I) x of the unit quaternion q = (w, u): -2 (u.u) x + 2 u (u.x) + 2 w (u x x). Exact for small u.
Vec3 rot_minus_i(Quat q, Vec3 x) {
    const Vec3 u{q.x, q.y, q.z};
    return (-2.f * dot(u, u)) * x + (2.f * dot(u, x)) * u + (2.f * q.w) * cross(u, x);
}

// exp(a)^-1 (x) exp(a + b) for a small rotation b, without forming exp(a + b) - exp(a). With s(x) = sin(x/2)/x and
// c(x) = cos(x/2) the product is (c0 ci + s0 si a.(a+b), (c0 si - ci s0) a + c0 si b - s0 si a x b).
Quat rel_exp(Vec3 a, Vec3 b) {
    const float x0 = dot(a, a);
    const float d = 2.f * dot(a, b) + dot(b, b);  // |a+b|^2 - |a|^2
    const float xi = x0 + d;
    float s0, si, c0, ci, ds, dc;
    if (x0 < 0.04f && xi < 0.04f) {
        // Series about 0 (truncation < 1e-8 below |x| = 0.2): s = 1/2 - x^2/48 + x^4/3840, c = 1 - x^2/8 + x^4/384.
        s0 = 0.5f - x0 / 48.f + x0 * x0 / 3840.f;
        si = 0.5f - xi / 48.f + xi * xi / 3840.f;
        c0 = 1.f - x0 / 8.f + x0 * x0 / 384.f;
        ci = 1.f - xi / 8.f + xi * xi / 384.f;
        ds = d * (-1.f / 48.f + (xi + x0) / 3840.f);
        dc = d * (-1.f / 8.f + (xi + x0) / 384.f);
    } else {
        const float n0 = std::sqrt(x0), ni = std::sqrt(xi);
        s0 = std::sin(0.5f * n0) / n0;
        si = std::sin(0.5f * ni) / ni;
        c0 = std::cos(0.5f * n0);
        ci = std::cos(0.5f * ni);
        ds = si - s0;
        dc = ci - c0;
    }
    const Vec3 v = (c0 * ds - s0 * dc) * a + (c0 * si) * b - (s0 * si) * cross(a, b);
    return {c0 * ci + s0 * si * (x0 + dot(a, b)), v.x, v.y, v.z};
}

// yaw(q0 (x) dq) - yaw(q0) with dq = exp(th): q0 (x) dq = q0 + D, D = q0 (x) (cos(|th|/2) - 1, u), and
// yaw = atan2(n, d), n = 2 (w z + x y), d = 1 - 2 (y^2 + z^2), so the difference is atan2(dn d0 - dd n0, d d0 + n n0).
float yaw_deviation(Quat q0, Vec3 th) {
    const float a = norm(th);
    const float h = std::sin(0.25f * a);
    const Quat e = quat_from_rotvec(th);
    const Quat D = q0 * Quat{-2.f * h * h, e.x, e.y, e.z};
    const float n0 = 2.f * (q0.w * q0.z + q0.x * q0.y);
    const float d0 = 1.f - 2.f * (q0.y * q0.y + q0.z * q0.z);
    const float dn = 2.f * (q0.w * D.z + D.w * q0.z + D.w * D.z + q0.x * D.y + D.x * q0.y + D.x * D.y);
    const float dd = -2.f * (2.f * q0.y * D.y + D.y * D.y + 2.f * q0.z * D.z + D.z * D.z);
    return std::atan2(dn * d0 - dd * n0, (d0 + dd) * d0 + (n0 + dn) * n0);
}

}  // namespace

Ukf::Ukf(const EskfParams& p, const UtParams& ut)
    : prm_(p), mag_decl_(std::atan2(p.mag_ref_ned_ut.y, p.mag_ref_ned_ut.x)), alignment_(p) {
    // utWeights: lambda = alpha^2 (n + kappa) - n.
    const float n = static_cast<float>(N);
    const float c = ut.alpha * ut.alpha * (n + ut.kappa);
    const float lambda = c - n;
    gamma_ = std::sqrt(c);
    wm0_ = lambda / c;
    wc0_ = wm0_ + 1.f - ut.alpha * ut.alpha + ut.beta;
    wi_ = 1.f / (2.f * c);
    for (int i = 0; i < N; ++i)
        for (int j = 0; j < N; ++j) {
            P_[i][j] = 0.f;
            L_[i][j] = 0.f;
            C_[i][j] = 0.f;
        }
}

void Ukf::update(const SensorBus& bus) {
    t_us_ = bus.t_us;
    if ((bus.fresh & kGnss) && bus.gnss.fix && !frame_.valid()) frame_.set({bus.gnss.lat_e7, bus.gnss.lon_e7, bus.gnss.alt_m});

    if (!aligned_) {
        Alignment a;
        if (alignment_.feed(bus, a)) {
            align(a);
            if (bus.fresh & kImu) {
                last_imu_us_ = bus.t_us;
                w_meas_ = bus.imu.gyro_frd;
            }
        }
        return;
    }

    if (bus.fresh & kImu) {
        w_meas_ = bus.imu.gyro_frd;
        const float dt = static_cast<float>(bus.t_us - last_imu_us_) * 1e-6f;
        last_imu_us_ = bus.t_us;
        if (dt > 0.f) predict(bus.imu.accel_frd, bus.imu.gyro_frd, dt);
    }
    if ((bus.fresh & kGnss) && bus.gnss.fix && frame_.valid()) {
        const Vec3 z = frame_.to_ned({bus.gnss.lat_e7, bus.gnss.lon_e7, bus.gnss.alt_m});
        const float yp[3] = {z.x - p_.x, z.y - p_.y, z.z - p_.z};
        correct(Meas::kPosition, yp, prm_.sigma_gnss_pos);
        const float yv[3] = {bus.gnss.vel_ned.x - v_.x, bus.gnss.vel_ned.y - v_.y, bus.gnss.vel_ned.z - v_.z};
        correct(Meas::kVelocity, yv, prm_.sigma_gnss_vel);
    }
    if (bus.fresh & kBaro) {
        const float y = (isa_height(bus.baro.pressure_pa) - baro_h0_) - (-p_.z);
        correct(Meas::kAltitude, &y, prm_.sigma_baro);
    }
    if (bus.fresh & kMag) {
        const float y = heading_innovation(q_, bus.mag.field_frd_ut, mag_decl_);
        correct(Meas::kHeading, &y, prm_.sigma_heading);
    }
}

void Ukf::align(const Alignment& a) {
    p_ = {0.f, 0.f, 0.f};
    v_ = {0.f, 0.f, 0.f};
    q_ = a.q;
    ab_ = {0.f, 0.f, 0.f};
    wb_ = a.gyro_mean;
    baro_h0_ = a.baro_h0;
    float sigma_wb = prm_.sigma_gyro / std::sqrt(static_cast<float>(a.n_imu));
    if (sigma_wb < 1e-4f) sigma_wb = 1e-4f;
    const float d[5] = {prm_.sigma_p0, prm_.sigma_v0, prm_.sigma_theta0, prm_.sigma_ab0, sigma_wb};
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) P_[i][j] = 0.f;
        P_[i][i] = d[i / 3] * d[i / 3];
    }
    aligned_ = true;
}

// Lower-triangular L_ with L_ L_^T = P_. False when P_ is not positive definite.
bool Ukf::cholesky() {
    for (int j = 0; j < N; ++j) {
        float d = P_[j][j];
        for (int k = 0; k < j; ++k) d -= L_[j][k] * L_[j][k];
        if (!(d > 0.f)) return false;
        const float l = std::sqrt(d);
        L_[j][j] = l;
        for (int i = 0; i < j; ++i) L_[i][j] = 0.f;
        for (int i = j + 1; i < N; ++i) {
            float s = P_[i][j];
            for (int k = 0; k < j; ++k) s -= L_[i][k] * L_[j][k];
            L_[i][j] = s / l;
        }
    }
    return true;
}

// ukfPredict: sigma points of P through the IMU kinematics, recombined about the propagated centre point, + Q.
void Ukf::predict(Vec3 am, Vec3 wm, float dt) {
    const Vec3 f = am - ab_;
    const Vec3 w0 = wm - wb_;
    const Vec3 a0 = rotate(q_, f) + Vec3{0.f, 0.f, prm_.gravity};
    const Quat e0 = quat_from_rotvec(dt * w0);
    const bool spread = cholesky();
    if (!spread) ++chol_fail_;

    float mean[N] = {};
    if (spread) {
        for (int i = 0; i < N; ++i)
            for (int j = i; j < N; ++j) C_[i][j] = 0.f;
        for (int col = 0; col < 2 * N; ++col) {
            const float sg = col < N ? gamma_ : -gamma_;
            const int j = col % N;
            float c[N];
            for (int k = 0; k < N; ++k) c[k] = sg * L_[k][j];
            const Vec3 dp = v3(c + IP), dv = v3(c + IV), dth = v3(c + ITH), dab = v3(c + IAB), dwb = v3(c + IWB);
            const Quat dq = quat_from_rotvec(dth);
            const Vec3 da = rotate(q_, rot_minus_i(dq, f) - (dab + rot_minus_i(dq, dab)));
            float d[N];
            const Vec3 ddp = dp + dt * dv + (0.5f * dt * dt) * da;
            const Vec3 ddv = dv + dt * da;
            const Quat r = quat_from_rotvec(rotate_inv(e0, dth)) * rel_exp(dt * w0, -dt * dwb);
            const Vec3 ddth = rotvec_from_quat(r);
            const Vec3 blocks[5] = {ddp, ddv, ddth, dab, dwb};
            for (int b = 0; b < 5; ++b) {
                d[3 * b] = blocks[b].x;
                d[3 * b + 1] = blocks[b].y;
                d[3 * b + 2] = blocks[b].z;
            }
            for (int i = 0; i < N; ++i) {
                mean[i] += wi_ * d[i];
                const float wd = wi_ * d[i];
                for (int k = i; k < N; ++k) C_[i][k] += wd * d[k];
            }
        }
        const float c0 = wc0_ - wm0_ - 1.f;
        for (int i = 0; i < N; ++i)
            for (int k = i; k < N; ++k) {
                const float v = C_[i][k] + c0 * mean[i] * mean[k];
                P_[i][k] = v;
                P_[k][i] = v;
            }
    }

    const float qv = prm_.sigma_accel * prm_.sigma_accel * dt * dt;
    const float qt = prm_.sigma_gyro * prm_.sigma_gyro * dt * dt;
    const float qa = prm_.sigma_accel_walk * prm_.sigma_accel_walk * dt;
    const float qw = prm_.sigma_gyro_walk * prm_.sigma_gyro_walk * dt;
    for (int i = 0; i < 3; ++i) {
        P_[IV + i][IV + i] += qv;
        P_[ITH + i][ITH + i] += qt;
        P_[IAB + i][IAB + i] += qa;
        P_[IWB + i][IWB + i] += qw;
    }

    // Centre point through the kinematics, then retract(moved[0], mean).
    p_ = p_ + dt * v_ + (0.5f * dt * dt) * a0 + v3(mean + IP);
    v_ = v_ + dt * a0 + v3(mean + IV);
    q_ = normalized(normalized(q_ * e0) * quat_from_rotvec(v3(mean + ITH)));
    ab_ += v3(mean + IAB);
    wb_ += v3(mean + IWB);
}

// ukfCorrect: sigma points of the prior through h, z_hat and P_zz from their spread, P_xz from their correlation with
// the error-state deviations, K = P_xz S^-1, dx = K (z - z_hat), P <- P - K S K^T (= P - K P_xz^T), dx injected.
// y0 is the innovation about the nominal state, z - h(x).
void Ukf::correct(Meas m, const float* y0, float sigma) {
    if (!cholesky()) {
        ++chol_fail_;
        return;
    }
    const int k = (m == Meas::kPosition || m == Meas::kVelocity) ? 3 : 1;
    float mz[3] = {}, pzz[3][3] = {};
    float (&pxz)[N][3] = Pxz_;
    for (auto& row : pxz)
        for (float& v : row) v = 0.f;
    for (int col = 0; col < 2 * N; ++col) {
        const float sg = col < N ? gamma_ : -gamma_;
        const int j = col % N;
        float c[N];
        for (int r = 0; r < N; ++r) c[r] = sg * L_[r][j];
        float dz[3] = {};
        switch (m) {
            case Meas::kPosition:
                for (int r = 0; r < 3; ++r) dz[r] = c[IP + r];
                break;
            case Meas::kVelocity:
                for (int r = 0; r < 3; ++r) dz[r] = c[IV + r];
                break;
            case Meas::kAltitude:
                dz[0] = -c[IP + 2];
                break;
            case Meas::kHeading:
                dz[0] = yaw_deviation(q_, v3(c + ITH));
                break;
        }
        for (int a = 0; a < k; ++a) {
            mz[a] += wi_ * dz[a];
            for (int b = 0; b < k; ++b) pzz[a][b] += wi_ * dz[a] * dz[b];
        }
        for (int r = j; r < N; ++r)  // c[r] = 0 above the diagonal of L
            for (int b = 0; b < k; ++b) pxz[r][b] += wi_ * c[r] * dz[b];
    }
    const float c0 = wc0_ - wm0_ - 1.f;
    float S[3][3];
    for (int a = 0; a < k; ++a)
        for (int b = 0; b < k; ++b) S[a][b] = pzz[a][b] + c0 * mz[a] * mz[b] + (a == b ? sigma * sigma : 0.f);
    float y[3];
    for (int a = 0; a < k; ++a) y[a] = y0[a] - mz[a];
    if (m == Meas::kHeading) y[0] = wrap(y[0]);

    // Cholesky of S (k <= 3), then each row of K solves S K_r^T = P_xz,r^T.
    float Ls[3][3] = {};
    for (int j = 0; j < k; ++j) {
        float d = S[j][j];
        for (int t = 0; t < j; ++t) d -= Ls[j][t] * Ls[j][t];
        if (!(d > 0.f)) {
            ++chol_fail_;
            return;
        }
        Ls[j][j] = std::sqrt(d);
        for (int i = j + 1; i < k; ++i) {
            float s = S[i][j];
            for (int t = 0; t < j; ++t) s -= Ls[i][t] * Ls[j][t];
            Ls[i][j] = s / Ls[j][j];
        }
    }
    float (&K)[N][3] = K_;
    float dx[N];
    for (int r = 0; r < N; ++r) {
        float t[3];
        for (int i = 0; i < k; ++i) {
            float s = pxz[r][i];
            for (int q = 0; q < i; ++q) s -= Ls[i][q] * t[q];
            t[i] = s / Ls[i][i];
        }
        for (int i = k - 1; i >= 0; --i) {
            float s = t[i];
            for (int q = i + 1; q < k; ++q) s -= Ls[q][i] * K[r][q];
            K[r][i] = s / Ls[i][i];
        }
        float a = 0.f;
        for (int i = 0; i < k; ++i) a += K[r][i] * y[i];
        dx[r] = a;
    }
    for (int r = 0; r < N; ++r)
        for (int c = r; c < N; ++c) {
            float s = 0.f;
            for (int i = 0; i < k; ++i) s += K[r][i] * pxz[c][i];
            float t = 0.f;
            for (int i = 0; i < k; ++i) t += K[c][i] * pxz[r][i];
            const float v = P_[r][c] - 0.5f * (s + t);
            P_[r][c] = v;
            P_[c][r] = v;
        }

    p_ += v3(dx + IP);
    v_ += v3(dx + IV);
    q_ = normalized(q_ * quat_from_rotvec(v3(dx + ITH)));
    ab_ += v3(dx + IAB);
    wb_ += v3(dx + IWB);
}

State Ukf::state() const {
    State s{};
    s.t_us = t_us_;
    s.q = kQuatIdentity;
    if (!aligned_) return s;
    s.p_ned = p_;
    s.v_ned = v_;
    s.q = q_;
    s.w_frd = w_meas_ - wb_;
    s.valid = frame_.valid();
    return s;
}

}  // namespace marv
