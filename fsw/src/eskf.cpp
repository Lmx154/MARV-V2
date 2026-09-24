// Port of avionics-toolbox src/lib/calc/eskf.ts (15-state ESKF) and the sensor wiring of src/lib/sim/sixdof/fsw.ts.
// Deviations from the TypeScript, each for the M33 (float, fixed storage, no inverse):
//  - float throughout; P is symmetrised after every predict and update.
//  - F_x P F_x^T is applied block-sparse (only the non-identity blocks of F_x) instead of dense 15x15 products.
//  - Vector measurements (GNSS position, velocity) are fused as sequential scalar updates with diagonal R, which is
//    exactly the vector update (no 3x3 inverse); each scalar uses the Joseph form, as kfUpdate does.
//  - The heading Jacobian is analytic (d heading / d dtheta of the tilt-compensated field) instead of central
//    differences at eps 1e-6, which float cannot resolve.
//  - The toolbox's sensor bus hands the filter a heading; here the magnetometer vector is tilt-compensated with the
//    estimated attitude and compared with the declination of the reference field.
//  - Initialisation is stationary alignment (accel tilt, mag heading, gyro average as the pad calibration of
//    createEstimator) instead of a truth-plus-error initial state; gravity aiding (a mission flag in the lab) is not
//    ported: alignment levels the filter before it runs. It averages only over a window the IMU calls still.
//  - A GNSS fix is fused over two ticks (position, then velocity), so no tick carries all six updates.
#include <marv/fsw/eskf.hpp>

#include <cmath>

#include <marv/fsw/math.hpp>

namespace marv {

namespace {

constexpr int IP = 0;
constexpr int IV = 3;
constexpr int ITH = 6;
constexpr int IAB = 9;
constexpr int IWB = 12;
constexpr int N = Eskf::kN;

// ISA troposphere (the model of gz-sim's air-pressure sensor): h = T0/L (1 - (P/P0)^(R L / g M)).
constexpr float kIsaP0 = 101325.f;
constexpr float kIsaT0OverL = 44330.77f;
constexpr float kIsaExp = 0.190263f;

struct M3 {
    float m[3][3];
};

// R with v_ned = R v_body (eskf.ts quatToRotMat).
M3 rotmat(Quat q) {
    const float w = q.w, x = q.x, y = q.y, z = q.z;
    return {{{1.f - 2.f * (y * y + z * z), 2.f * (x * y - w * z), 2.f * (x * z + w * y)},
             {2.f * (x * y + w * z), 1.f - 2.f * (x * x + z * z), 2.f * (y * z - w * x)},
             {2.f * (x * z - w * y), 2.f * (y * z + w * x), 1.f - 2.f * (x * x + y * y)}}};
}

float wrap(float a) { return std::atan2(std::sin(a), std::cos(a)); }

float baro_height(float pressure_pa) { return kIsaT0OverL * (1.f - std::pow(pressure_pa / kIsaP0, kIsaExp)); }

void symmetrise(float (&P)[N][N]) {
    for (int i = 0; i < N; ++i)
        for (int j = i + 1; j < N; ++j) {
            const float s = 0.5f * (P[i][j] + P[j][i]);
            P[i][j] = s;
            P[j][i] = s;
        }
}

// out = F_x in, with F_x of eskfPredict: I plus the blocks (p,v) = I dt, (v,theta) = A, (v,a_b) = B,
// (theta,theta) = C, (theta,w_b) = -I dt.
void apply_fx(const float (&in)[N][N], float (&out)[N][N], const M3& A, const M3& B, const M3& C, float dt) {
    for (int c = 0; c < N; ++c) {
        for (int i = 0; i < 3; ++i) {
            out[IP + i][c] = in[IP + i][c] + dt * in[IV + i][c];
            float v = in[IV + i][c];
            float th = -dt * in[IWB + i][c];
            for (int j = 0; j < 3; ++j) {
                v += A.m[i][j] * in[ITH + j][c] + B.m[i][j] * in[IAB + j][c];
                th += C.m[i][j] * in[ITH + j][c];
            }
            out[IV + i][c] = v;
            out[ITH + i][c] = th;
            out[IAB + i][c] = in[IAB + i][c];
            out[IWB + i][c] = in[IWB + i][c];
        }
    }
}

}  // namespace

Eskf::Eskf(const EskfParams& p) : prm_(p), mag_decl_(std::atan2(p.mag_ref_ned_ut.y, p.mag_ref_ned_ut.x)) {
    for (int i = 0; i < N; ++i) {
        dx_[i] = 0.f;
        for (int j = 0; j < N; ++j) {
            P_[i][j] = 0.f;
            M_[i][j] = 0.f;
        }
    }
}

void Eskf::update(const SensorBus& bus) {
    t_us_ = bus.t_us;

    // The GNSS origin is the first fix, taken whenever it arrives. If the vehicle has moved since alignment, horizontal
    // position restarts there (0, 0) with the GNSS prior; height stays the barometer's, so the origin's altitude is
    // the alignment height (the fix's less the height climbed since).
    if ((bus.fresh & kGnss) && bus.gnss.fix && !frame_.valid()) {
        frame_.set(GeoPoint{bus.gnss.lat_e7, bus.gnss.lon_e7, bus.gnss.alt_m + (aligned_ ? p_.z : 0.f)});
        if (aligned_) {
            p_.x = p_.y = 0.f;
            for (int i = IP; i < IP + 2; ++i) {
                for (int j = 0; j < N; ++j) P_[i][j] = P_[j][i] = 0.f;
                P_[i][i] = prm_.sigma_gnss_pos * prm_.sigma_gnss_pos;
            }
        }
    }

    if (!aligned_) {
        accumulate_alignment(bus);
        return;
    }

    if (bus.fresh & kImu) {
        w_meas_ = bus.imu.gyro_frd;
        const float dt = static_cast<float>(bus.t_us - last_imu_us_) * 1e-6f;
        last_imu_us_ = bus.t_us;
        if (dt > 0.f) predict(bus.imu.accel_frd, bus.imu.gyro_frd, dt);
    }
    // A fix's six scalar updates are split over two ticks: position on the tick the fix arrives, its velocity on
    // the next, residual against the state then. That velocity is one tick (1 ms) stale when fused: at the 3 m/s^2
    // of the square's corners that is 3 mm/s, 6 % of sigma_gnss_vel, so it is not compensated. A new fix while its
    // predecessor's velocity is still pending supersedes it, so a tick never carries more than three GNSS updates.
    const bool fix = (bus.fresh & kGnss) && bus.gnss.fix && frame_.valid();
    if (fix) {
        fuse_gnss_pos(bus.gnss);
        vel_meas_ = bus.gnss.vel_ned;
        vel_pending_ = true;
    } else if (vel_pending_) {
        fuse_gnss_vel();
        vel_pending_ = false;
    }
    if (bus.fresh & kBaro) fuse_baro(bus.baro.pressure_pa);
    if (bus.fresh & kMag) fuse_mag(bus.mag.field_frd_ut);
}

void Eskf::accumulate_alignment(const SensorBus& bus) {
    if (bus.fresh & kImu) {
        const Vec3 f = bus.imu.accel_frd;
        const Vec3 w = bus.imu.gyro_frd;
        bool still = std::fabs(norm(f) - prm_.gravity) < prm_.still_g_err && norm(w) < prm_.still_rate;
        if (still && n_imu_ > 0) {
            const float inv = 1.f / static_cast<float>(n_imu_);
            still = norm(f - inv * sum_f_) < prm_.still_accel_dev && norm(w - inv * sum_w_) < prm_.still_gyro_dev;
        }
        if (!still) {
            sum_f_ = sum_w_ = sum_m_ = Vec3{0.f, 0.f, 0.f};
            sum_h_ = 0.f;
            n_imu_ = n_mag_ = n_baro_ = 0;
            return;
        }
        if (n_imu_ == 0) t_win_us_ = bus.t_us;
        sum_f_ += f;
        sum_w_ += w;
        ++n_imu_;
    }
    if (n_imu_ == 0) return;
    if (bus.fresh & kMag) {
        sum_m_ += bus.mag.field_frd_ut;
        ++n_mag_;
    }
    if (bus.fresh & kBaro) {
        sum_h_ += baro_height(bus.baro.pressure_pa);
        ++n_baro_;
    }
    const float t = static_cast<float>(bus.t_us - t_win_us_) * 1e-6f;
    if (t >= prm_.align_window_s && n_mag_ > 0 && n_baro_ > 0) {
        align();
        if (bus.fresh & kImu) {
            last_imu_us_ = bus.t_us;
            w_meas_ = bus.imu.gyro_frd;
        }
    }
}

void Eskf::align() {
    const Vec3 f = (1.f / static_cast<float>(n_imu_)) * sum_f_;
    const Vec3 m = (1.f / static_cast<float>(n_mag_)) * sum_m_;
    // At rest f = R^T (0, 0, -g): roll and pitch from its direction.
    const float roll = std::atan2(-f.y, -f.z);
    const float pitch = std::atan2(f.x, std::sqrt(f.y * f.y + f.z * f.z));
    // Tilt-compensated heading: the levelled field makes angle (declination - yaw) with north.
    const Vec3 mh = rotate(quat_from_euler(roll, pitch, 0.f), m);
    const float yaw = wrap(mag_decl_ - std::atan2(mh.y, mh.x));

    p_ = {0.f, 0.f, 0.f};
    v_ = {0.f, 0.f, 0.f};
    q_ = normalized(quat_from_euler(roll, pitch, yaw));
    ab_ = {0.f, 0.f, 0.f};
    wb_ = (1.f / static_cast<float>(n_imu_)) * sum_w_;  // fsw.ts pad calibration
    baro_h0_ = sum_h_ / static_cast<float>(n_baro_);

    float sigma_wb = prm_.sigma_gyro / std::sqrt(static_cast<float>(n_imu_));
    if (sigma_wb < 1e-4f) sigma_wb = 1e-4f;
    const float d[5] = {prm_.sigma_p0, prm_.sigma_v0, prm_.sigma_theta0, prm_.sigma_ab0, sigma_wb};
    for (int i = 0; i < N; ++i) {
        dx_[i] = 0.f;
        for (int j = 0; j < N; ++j) P_[i][j] = 0.f;
        P_[i][i] = d[i / 3] * d[i / 3];
    }
    aligned_ = true;
}

// eskfPredict: nominal kinematics with q before the step, then P <- F_x P F_x^T + F_i Q_i F_i^T.
void Eskf::predict(Vec3 am, Vec3 wm, float dt) {
    const M3 R = rotmat(q_);
    const Vec3 a_body = am - ab_;
    const Vec3 w_body = wm - wb_;
    const Vec3 a_world = rotate(q_, a_body) + Vec3{0.f, 0.f, prm_.gravity};
    const Quat dq = quat_from_rotvec(dt * w_body);

    p_ = p_ + dt * v_ + (0.5f * dt * dt) * a_world;
    v_ = v_ + dt * a_world;
    q_ = normalized(q_ * dq);

    // A = -R [a_body]x dt, B = -R dt, C = R(dq)^T.
    const float sk[3][3] = {{0.f, -a_body.z, a_body.y}, {a_body.z, 0.f, -a_body.x}, {-a_body.y, a_body.x, 0.f}};
    const M3 Rdq = rotmat(dq);
    M3 A{}, B{}, C{};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) {
            float s = 0.f;
            for (int k = 0; k < 3; ++k) s += R.m[i][k] * sk[k][j];
            A.m[i][j] = -dt * s;
            B.m[i][j] = -dt * R.m[i][j];
            C.m[i][j] = Rdq.m[j][i];
        }

    // M = F P; transpose; P = F M^T = (F P F^T)^T.
    apply_fx(P_, M_, A, B, C, dt);
    for (int i = 0; i < N; ++i)
        for (int j = i + 1; j < N; ++j) {
            const float t = M_[i][j];
            M_[i][j] = M_[j][i];
            M_[j][i] = t;
        }
    apply_fx(M_, P_, A, B, C, dt);

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
    symmetrise(P_);
}

// One scalar row of kfUpdate: innovation y (about the nominal state) less what dx_ already explains,
// K = P h / s, dx += K y, P <- (I - K h^T) P (I - K h^T)^T + K r K^T.
// With u = P h and s = h^T u + r the Joseph form expands exactly to P - k u^T - u k^T + s k k^T: one pass over the
// upper triangle, symmetric by construction, about half the work of forming (I - k h^T) P and its product.
void Eskf::scalar_update(const float* h, float y, float r) {
    float u[N] = {};
    for (int j = 0; j < N; ++j) {
        if (h[j] == 0.f) continue;
        for (int i = 0; i < N; ++i) u[i] += P_[i][j] * h[j];
    }
    float s = r;
    for (int i = 0; i < N; ++i) {
        s += h[i] * u[i];
        y -= h[i] * dx_[i];
    }
    float k[N];
    for (int i = 0; i < N; ++i) {
        k[i] = u[i] / s;
        dx_[i] += k[i] * y;
    }
    for (int i = 0; i < N; ++i)
        for (int j = i; j < N; ++j) {
            const float v = P_[i][j] - k[i] * u[j] - u[i] * k[j] + s * k[i] * k[j];
            P_[i][j] = v;
            P_[j][i] = v;
        }
}

// eskfCorrect injection and reset: q <- q (x) q{dtheta}, P <- G P G^T with G_theta = I - [dtheta / 2]x.
void Eskf::inject_and_reset() {
    p_ += Vec3{dx_[IP], dx_[IP + 1], dx_[IP + 2]};
    v_ += Vec3{dx_[IV], dx_[IV + 1], dx_[IV + 2]};
    const Vec3 dth{dx_[ITH], dx_[ITH + 1], dx_[ITH + 2]};
    q_ = normalized(q_ * quat_from_rotvec(dth));
    ab_ += Vec3{dx_[IAB], dx_[IAB + 1], dx_[IAB + 2]};
    wb_ += Vec3{dx_[IWB], dx_[IWB + 1], dx_[IWB + 2]};

    const Vec3 e = 0.5f * dth;
    const float G[3][3] = {{1.f, e.z, -e.y}, {-e.z, 1.f, e.x}, {e.y, -e.x, 1.f}};
    // Rows of the theta block, then columns.
    for (int c = 0; c < N; ++c) {
        float r[3];
        for (int i = 0; i < 3; ++i) r[i] = G[i][0] * P_[ITH][c] + G[i][1] * P_[ITH + 1][c] + G[i][2] * P_[ITH + 2][c];
        for (int i = 0; i < 3; ++i) P_[ITH + i][c] = r[i];
    }
    for (int rr = 0; rr < N; ++rr) {
        float r[3];
        for (int i = 0; i < 3; ++i) r[i] = G[i][0] * P_[rr][ITH] + G[i][1] * P_[rr][ITH + 1] + G[i][2] * P_[rr][ITH + 2];
        for (int i = 0; i < 3; ++i) P_[rr][ITH + i] = r[i];
    }
    symmetrise(P_);
    for (int i = 0; i < N; ++i) dx_[i] = 0.f;
}

// fsw.ts order: position, then velocity, each corrected and injected.
void Eskf::fuse_gnss_pos(const GnssSample& g) {
    const Vec3 z = frame_.to_ned(GeoPoint{g.lat_e7, g.lon_e7, g.alt_m});
    const float zp[3] = {z.x, z.y, z.z};
    const float pp[3] = {p_.x, p_.y, p_.z};
    const float rh = prm_.sigma_gnss_pos * prm_.sigma_gnss_pos;
    const float rp[3] = {rh, rh, prm_.sigma_gnss_alt * prm_.sigma_gnss_alt};
    for (int i = 0; i < 3; ++i) {
        float h[N] = {};
        h[IP + i] = 1.f;
        scalar_update(h, zp[i] - pp[i], rp[i]);
    }
    inject_and_reset();
}

void Eskf::fuse_gnss_vel() {
    const float zv[3] = {vel_meas_.x, vel_meas_.y, vel_meas_.z};
    const float vv[3] = {v_.x, v_.y, v_.z};
    const float rv = prm_.sigma_gnss_vel * prm_.sigma_gnss_vel;
    for (int i = 0; i < 3; ++i) {
        float h[N] = {};
        h[IV + i] = 1.f;
        scalar_update(h, zv[i] - vv[i], rv);
    }
    inject_and_reset();
}

// 'altitude': z = height above the aligned baro reference, h(x) = -p_z.
void Eskf::fuse_baro(float pressure_pa) {
    float h[N] = {};
    h[IP + 2] = -1.f;
    scalar_update(h, (baro_height(pressure_pa) - baro_h0_) - (-p_.z), prm_.sigma_baro * prm_.sigma_baro);
    inject_and_reset();
}

// 'heading': the field rotated to NED with the estimate lies at (declination + yaw_est - yaw_mag), so the
// innovation yaw_mag - yaw_est is declination - its angle. With mw = R m, d mw / d dtheta = -[mw]x R, so
// d atan2(mw_y, mw_x) / d dtheta = (-mw_x mw_z / |mw_h|^2, -mw_y mw_z / |mw_h|^2, 1) R: yaw plus the tilt coupling,
// taken at the predicted field (the reference): at the measured one the Jacobian carries the sample's noise, which
// correlates with the innovation's and biases every update (in SITL it walked the east tilt and a_b y 5 sigma off).
void Eskf::fuse_mag(Vec3 m_frd) {
    const Vec3 mw = rotate(q_, m_frd);
    const float mh2 = mw.x * mw.x + mw.y * mw.y;
    if (mh2 < 1e-6f) return;  // no horizontal field: heading undefined
    const float y = wrap(mag_decl_ - std::atan2(mw.y, mw.x));
    const M3 R = rotmat(q_);
    const Vec3 m0 = prm_.mag_ref_ned_ut;
    const float m0h2 = m0.x * m0.x + m0.y * m0.y;
    const float e[3] = {-m0.x * m0.z / m0h2, -m0.y * m0.z / m0h2, 1.f};
    float h[N] = {};
    for (int j = 0; j < 3; ++j) h[ITH + j] = e[0] * R.m[0][j] + e[1] * R.m[1][j] + e[2] * R.m[2][j];
    scalar_update(h, y, prm_.sigma_heading * prm_.sigma_heading);
    inject_and_reset();
}

State Eskf::state() const {
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
