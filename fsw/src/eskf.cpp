// Port of avionics-toolbox src/lib/calc/eskf.ts (15-state ESKF) and the sensor wiring of src/lib/sim/sixdof/fsw.ts.
// Deviations from the TypeScript, each for the M33 (float, fixed storage, no inverse):
//  - float throughout; P is symmetrised after every predict and update.
//  - F_x P F_x^T is applied block-sparse (only the non-identity blocks of F_x) instead of dense 15x15 products.
//  - Vector measurements (GNSS position, velocity) are fused as sequential scalar updates with diagonal R, which is
//    exactly the vector update (no 3x3 inverse); each scalar uses the Joseph form, as kfUpdate does.
//  - The heading Jacobian is analytic (d yaw / d dtheta for ZYX Euler) instead of central differences at eps 1e-6,
//    which float cannot resolve.
//  - The toolbox's sensor bus hands the filter a heading; here the magnetometer vector is tilt-compensated with the
//    estimated attitude and compared with the declination of the reference field.
//  - Initialisation is stationary alignment (accel tilt, mag heading, gyro average as the pad calibration of
//    createEstimator) instead of a truth-plus-error initial state; gravity aiding (a mission flag in the lab) is not
//    ported: alignment levels the filter before it runs.
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

constexpr float kPi = 3.14159265358979f;
constexpr float kRadPerE7 = 1e-7f * kPi / 180.f;

// ISA troposphere (the model of gz-sim's air-pressure sensor): h = T0/L (1 - (P/P0)^(R L / g M)).
constexpr float kIsaP0 = 101325.f;
constexpr float kIsaT0OverL = 44330.77f;
constexpr float kIsaExp = 0.190263f;

// WGS84.
constexpr float kWgsA = 6378137.f;
constexpr float kWgsE2 = 6.69437999014e-3f;

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
    if (!started_) {
        started_ = true;
        t_first_us_ = bus.t_us;
    }
    t_us_ = bus.t_us;

    // The GNSS origin is the first fix, taken whenever it arrives (at start the vehicle is at rest there).
    if ((bus.fresh & kGnss) && bus.gnss.fix && !have_origin_) {
        have_origin_ = true;
        lat0_e7_ = bus.gnss.lat_e7;
        lon0_e7_ = bus.gnss.lon_e7;
        alt0_ = bus.gnss.alt_m;
        // Flat earth about the origin: meridian and prime-vertical radii of WGS84 at its latitude.
        const float lat = static_cast<float>(lat0_e7_) * kRadPerE7;
        const float s = std::sin(lat);
        const float d = 1.f - kWgsE2 * s * s;
        const float rn = kWgsA * (1.f - kWgsE2) / (d * std::sqrt(d));
        const float re = kWgsA / std::sqrt(d);
        m_per_e7_n_ = (rn + alt0_) * kRadPerE7;
        m_per_e7_e_ = (re + alt0_) * std::cos(lat) * kRadPerE7;
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
    if ((bus.fresh & kGnss) && bus.gnss.fix && have_origin_) fuse_gnss(bus.gnss);
    if (bus.fresh & kBaro) fuse_baro(bus.baro.pressure_pa);
    if (bus.fresh & kMag) fuse_mag(bus.mag.field_frd_ut);
}

void Eskf::accumulate_alignment(const SensorBus& bus) {
    const float t = static_cast<float>(bus.t_us - t_first_us_) * 1e-6f;
    if (t >= prm_.align_skip_s && t < prm_.align_skip_s + prm_.align_window_s) {
        if (bus.fresh & kImu) {
            sum_f_ += bus.imu.accel_frd;
            sum_w_ += bus.imu.gyro_frd;
            ++n_imu_;
        }
        if (bus.fresh & kMag) {
            sum_m_ += bus.mag.field_frd_ut;
            ++n_mag_;
        }
        if (bus.fresh & kBaro) {
            sum_h_ += baro_height(bus.baro.pressure_pa);
            ++n_baro_;
        }
    }
    if (t >= prm_.align_skip_s + prm_.align_window_s && n_imu_ > 0 && n_mag_ > 0 && n_baro_ > 0) {
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
void Eskf::scalar_update(const float* h, float y, float r) {
    float u[N];
    float s = r;
    for (int i = 0; i < N; ++i) {
        float a = 0.f;
        for (int j = 0; j < N; ++j) a += P_[i][j] * h[j];
        u[i] = a;
        s += h[i] * a;
        y -= h[i] * dx_[i];
    }
    float k[N];
    for (int i = 0; i < N; ++i) {
        k[i] = u[i] / s;
        dx_[i] += k[i] * y;
    }
    // B = (I - k h^T) P = P - k u^T; w = B h; P = B (I - k h^T)^T + r k k^T = B - w k^T + r k k^T.
    for (int i = 0; i < N; ++i)
        for (int j = 0; j < N; ++j) P_[i][j] -= k[i] * u[j];
    float w[N];
    for (int i = 0; i < N; ++i) {
        float a = 0.f;
        for (int j = 0; j < N; ++j) a += P_[i][j] * h[j];
        w[i] = a;
    }
    for (int i = 0; i < N; ++i)
        for (int j = 0; j < N; ++j) P_[i][j] += (r * k[i] - w[i]) * k[j];
    symmetrise(P_);
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
void Eskf::fuse_gnss(const GnssSample& g) {
    const Vec3 z{static_cast<float>(g.lat_e7 - lat0_e7_) * m_per_e7_n_,
                 static_cast<float>(g.lon_e7 - lon0_e7_) * m_per_e7_e_, -(g.alt_m - alt0_)};
    const float zp[3] = {z.x, z.y, z.z};
    const float pp[3] = {p_.x, p_.y, p_.z};
    const float rp = prm_.sigma_gnss_pos * prm_.sigma_gnss_pos;
    for (int i = 0; i < 3; ++i) {
        float h[N] = {};
        h[IP + i] = 1.f;
        scalar_update(h, zp[i] - pp[i], rp);
    }
    inject_and_reset();

    const float zv[3] = {g.vel_ned.x, g.vel_ned.y, g.vel_ned.z};
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
// innovation yaw_mag - yaw_est is declination - its angle. d yaw / d dtheta = (0, sin(roll), cos(roll)) / cos(pitch)
// = (0, R32, R33) / (R32^2 + R33^2).
void Eskf::fuse_mag(Vec3 m_frd) {
    const Vec3 mw = rotate(q_, m_frd);
    const float y = wrap(mag_decl_ - std::atan2(mw.y, mw.x));
    const M3 R = rotmat(q_);
    const float c2 = R.m[2][1] * R.m[2][1] + R.m[2][2] * R.m[2][2];
    if (c2 < 1e-6f) return;  // pitch at +-90 deg: heading undefined
    float h[N] = {};
    h[ITH + 1] = R.m[2][1] / c2;
    h[ITH + 2] = R.m[2][2] / c2;
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
    s.valid = true;
    return s;
}

}  // namespace marv
