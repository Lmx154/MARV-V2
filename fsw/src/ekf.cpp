// Port of avionics-toolbox src/lib/calc/ekf-full.ts wired as src/lib/sim/lab/blocks/estimator.ts kalmanEstimator.
// Deviations from the TypeScript, each for the M33 (float, fixed storage, no inverse) or shared with eskf.cpp:
//  - float throughout; P is symmetrised after every predict and update.
//  - F P F^T is applied block-sparse (only the non-identity blocks of F) instead of dense 16x16 products.
//  - Vector measurements (GNSS position, velocity) are fused as sequential scalar updates with diagonal R, each
//    innovation taken about the state the previous scalar left: exactly the vector update (no 3x3 inverse), Joseph
//    form as kfUpdate. The quaternion is re-normalised after the whole measurement, as ekfCorrect does.
//  - Heading: the magnetometer vector tilt-compensated with the estimated attitude, as eskf.cpp.
//  - Initialisation is Eskf's stationary alignment (tilt, heading, gyro average as the pad calibration) instead of a
//    truth-plus-heading-error initial state; the attitude prior 0.05 rad on every axis is kalmanEstimator's thetaCov
//    with no injected heading error. Gravity aiding (a mission flag in the lab) is not ported.
#include <marv/fsw/ekf.hpp>

#include <cmath>

#include <marv/fsw/math.hpp>

namespace marv {

namespace {

constexpr int IP = 0;
constexpr int IV = 3;
constexpr int IQ = 6;
constexpr int IAB = 10;
constexpr int IWB = 13;
constexpr int N = Ekf::kN;

constexpr float kIsaP0 = 101325.f;
constexpr float kIsaT0OverL = 44330.77f;
constexpr float kIsaExp = 0.190263f;

float wrap(float a) { return std::atan2(std::sin(a), std::cos(a)); }

struct M3 {
    float m[3][3];
};

M3 rotmat(Quat q) {
    const float w = q.w, x = q.x, y = q.y, z = q.z;
    return {{{1.f - 2.f * (y * y + z * z), 2.f * (x * y - w * z), 2.f * (x * z + w * y)},
             {2.f * (x * y + w * z), 1.f - 2.f * (x * x + z * z), 2.f * (y * z - w * x)},
             {2.f * (x * z - w * y), 2.f * (y * z + w * x), 1.f - 2.f * (x * x + y * y)}}};
}

// quatXi: q (x) [0, w] = Xi(q) w.
void xi(Quat q, float (&X)[4][3]) {
    const float w = q.w, x = q.x, y = q.y, z = q.z;
    const float r[4][3] = {{-x, -y, -z}, {w, -z, y}, {z, w, -x}, {-y, x, w}};
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 3; ++j) X[i][j] = r[i][j];
}

// quatRight: q (x) r = [r]_R q.
void quat_right(Quat r, float (&Q)[4][4]) {
    const float w = r.w, x = r.x, y = r.y, z = r.z;
    const float m[4][4] = {{w, -x, -y, -z}, {x, w, z, -y}, {y, -z, w, x}, {z, y, -x, w}};
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) Q[i][j] = m[i][j];
}

// dRotDq: d(R(q) a)/dq of the homogeneous form (w^2 - u.u) a + 2 (u.a) u + 2 w (u x a).
void drot_dq(Quat q, Vec3 a, float (&J)[3][4]) {
    const float w = q.w;
    const float u[3] = {q.x, q.y, q.z};
    const float av[3] = {a.x, a.y, a.z};
    const float ua = u[0] * av[0] + u[1] * av[1] + u[2] * av[2];
    const float uxa[3] = {u[1] * av[2] - u[2] * av[1], u[2] * av[0] - u[0] * av[2], u[0] * av[1] - u[1] * av[0]};
    const float ax[3][3] = {{0.f, -av[2], av[1]}, {av[2], 0.f, -av[0]}, {-av[1], av[0], 0.f}};
    for (int i = 0; i < 3; ++i) {
        J[i][0] = 2.f * (w * av[i] + uxa[i]);
        for (int j = 0; j < 3; ++j) J[i][1 + j] = 2.f * ((i == j ? ua : 0.f) + u[i] * av[j] - av[i] * u[j] - w * ax[i][j]);
    }
}

void symmetrise(float (&P)[N][N]) {
    for (int i = 0; i < N; ++i)
        for (int j = i + 1; j < N; ++j) {
            const float s = 0.5f * (P[i][j] + P[j][i]);
            P[i][j] = s;
            P[j][i] = s;
        }
}

// out = F in, F of ekfPredict: I plus (p,v) = I dt, (p,q) = Ja dt^2/2, (p,a_b) = -R dt^2/2, (v,q) = Ja dt,
// (v,a_b) = -R dt, (q,q) = [dq]_R, (q,w_b) = -Xi dt/2.
void apply_f(const float (&in)[N][N], float (&out)[N][N], const float (&Ja)[3][4], const M3& R, const float (&Qr)[4][4],
             const float (&X)[4][3], float dt) {
    const float h = 0.5f * dt * dt;
    for (int c = 0; c < N; ++c) {
        for (int i = 0; i < 3; ++i) {
            float jq = 0.f, ra = 0.f;
            for (int j = 0; j < 4; ++j) jq += Ja[i][j] * in[IQ + j][c];
            for (int j = 0; j < 3; ++j) ra += R.m[i][j] * in[IAB + j][c];
            out[IP + i][c] = in[IP + i][c] + dt * in[IV + i][c] + h * (jq - ra);
            out[IV + i][c] = in[IV + i][c] + dt * (jq - ra);
            out[IAB + i][c] = in[IAB + i][c];
            out[IWB + i][c] = in[IWB + i][c];
        }
        for (int i = 0; i < 4; ++i) {
            float a = 0.f;
            for (int j = 0; j < 4; ++j) a += Qr[i][j] * in[IQ + j][c];
            for (int j = 0; j < 3; ++j) a -= 0.5f * dt * X[i][j] * in[IWB + j][c];
            out[IQ + i][c] = a;
        }
    }
}

}  // namespace

float isa_height(float pressure_pa) { return kIsaT0OverL * (1.f - std::pow(pressure_pa / kIsaP0, kIsaExp)); }

float heading_innovation(Quat q, Vec3 m_frd, float declination) {
    const Vec3 mw = rotate(q, m_frd);
    return wrap(declination - std::atan2(mw.y, mw.x));
}

StationaryAlignment::StationaryAlignment(const param::SensorParams& s, float gravity)
    : p_(s), gravity_(gravity), declination_(std::atan2(s.mag_ref_ned_ut_y, s.mag_ref_ned_ut_x)) {}

bool StationaryAlignment::feed(const SensorBus& bus, Alignment& out) {
    if (bus.fresh & kImu) {
        const Vec3 f = bus.imu.accel_frd;
        const Vec3 w = bus.imu.gyro_frd;
        bool still = std::fabs(norm(f) - gravity_) < p_.still_g_err && norm(w) < p_.still_rate;
        if (still && n_imu_ > 0) {
            const float inv = 1.f / static_cast<float>(n_imu_);
            still = norm(f - inv * sum_f_) < p_.still_accel_dev && norm(w - inv * sum_w_) < p_.still_gyro_dev;
        }
        if (!still) {
            sum_f_ = sum_w_ = sum_m_ = Vec3{0.f, 0.f, 0.f};
            sum_h_ = 0.f;
            n_imu_ = n_mag_ = n_baro_ = 0;
            return false;
        }
        if (n_imu_ == 0) t_win_us_ = bus.t_us;
        sum_f_ += f;
        sum_w_ += w;
        ++n_imu_;
    }
    if (n_imu_ == 0) return false;
    if (bus.fresh & kMag) {
        sum_m_ += bus.mag.field_frd_ut;
        ++n_mag_;
    }
    if (bus.fresh & kBaro) {
        sum_h_ += isa_height(bus.baro.pressure_pa);
        ++n_baro_;
    }
    const float t = static_cast<float>(bus.t_us - t_win_us_) * 1e-6f;
    if (!(t >= p_.align_window_s && n_mag_ > 0 && n_baro_ > 0)) return false;
    const Vec3 f = (1.f / static_cast<float>(n_imu_)) * sum_f_;
    const Vec3 m = (1.f / static_cast<float>(n_mag_)) * sum_m_;
    const float roll = std::atan2(-f.y, -f.z);
    const float pitch = std::atan2(f.x, std::sqrt(f.y * f.y + f.z * f.z));
    const Vec3 mh = rotate(quat_from_euler(roll, pitch, 0.f), m);
    const float yaw = wrap(declination_ - std::atan2(mh.y, mh.x));
    out.q = normalized(quat_from_euler(roll, pitch, yaw));
    out.gyro_mean = (1.f / static_cast<float>(n_imu_)) * sum_w_;
    out.baro_h0 = sum_h_ / static_cast<float>(n_baro_);
    out.n_imu = n_imu_;
    out.t_us = bus.t_us;
    return true;
}

Ekf::Ekf(const param::EskfPriors& p, const param::SensorParams& s, const param::VehicleParams& v)
    : pri_(p),
      sns_(s),
      gravity_(v.gravity),
      mag_ref_{s.mag_ref_ned_ut_x, s.mag_ref_ned_ut_y, s.mag_ref_ned_ut_z},
      mag_decl_(std::atan2(s.mag_ref_ned_ut_y, s.mag_ref_ned_ut_x)),
      alignment_(s, v.gravity) {
    for (int i = 0; i < N; ++i) {
        x_[i] = 0.f;
        for (int j = 0; j < N; ++j) {
            P_[i][j] = 0.f;
            M_[i][j] = 0.f;
        }
    }
    x_[IQ] = 1.f;
}

void Ekf::update(const SensorBus& bus) {
    t_us_ = bus.t_us;
    // First fix after alignment: horizontal position restarts there with the GNSS prior, the origin's altitude is the
    // alignment height (as Eskf::update).
    if ((bus.fresh & kGnss) && bus.gnss.fix && !frame_.valid()) {
        frame_.set({bus.gnss.lat_e7, bus.gnss.lon_e7, bus.gnss.alt_m + (aligned_ ? x_[IP + 2] : 0.f)});
        if (aligned_) {
            for (int i = IP; i < IP + 2; ++i) {
                x_[i] = 0.f;
                for (int j = 0; j < N; ++j) P_[i][j] = P_[j][i] = 0.f;
                P_[i][i] = sns_.sigma_gnss_pos * sns_.sigma_gnss_pos;
            }
        }
    }

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
    // estimator.ts order: position, velocity, altitude, heading.
    if ((bus.fresh & kGnss) && bus.gnss.fix && frame_.valid()) {
        const Vec3 z = frame_.to_ned({bus.gnss.lat_e7, bus.gnss.lon_e7, bus.gnss.alt_m});
        const float zp[3] = {z.x, z.y, z.z};
        const float rp = sns_.sigma_gnss_pos * sns_.sigma_gnss_pos;
        for (int i = 0; i < 3; ++i) {
            float h[N] = {};
            h[IP + i] = 1.f;
            scalar_update(h, zp[i] - x_[IP + i], rp);
        }
        normalise_q();
        const float zv[3] = {bus.gnss.vel_ned.x, bus.gnss.vel_ned.y, bus.gnss.vel_ned.z};
        const float rv = sns_.sigma_gnss_vel * sns_.sigma_gnss_vel;
        for (int i = 0; i < 3; ++i) {
            float h[N] = {};
            h[IV + i] = 1.f;
            scalar_update(h, zv[i] - x_[IV + i], rv);
        }
        normalise_q();
    }
    if (bus.fresh & kBaro) {
        float h[N] = {};
        h[IP + 2] = -1.f;
        scalar_update(h, (isa_height(bus.baro.pressure_pa) - baro_h0_) - (-x_[IP + 2]), sns_.sigma_baro * sns_.sigma_baro);
        normalise_q();
    }
    if (bus.fresh & kMag) {
        // d heading / dq of the tilt-compensated field, heading = atan2(mw_y, mw_x) with mw = R(q) m: yaw and tilt,
        // taken at the predicted field (the reference), as Eskf::fuse_mag.
        const Quat q{x_[IQ], x_[IQ + 1], x_[IQ + 2], x_[IQ + 3]};
        const Vec3 mw = rotate(q, bus.mag.field_frd_ut);
        if (mw.x * mw.x + mw.y * mw.y > 1e-6f) {
            const Vec3 m0 = mag_ref_;
            const float r = m0.x * m0.x + m0.y * m0.y;
            float J[3][4];
            drot_dq(q, rotate_inv(q, m0), J);
            float h[N] = {};
            for (int i = 0; i < 4; ++i) h[IQ + i] = (m0.x * J[1][i] - m0.y * J[0][i]) / r;
            scalar_update(h, heading_innovation(q, bus.mag.field_frd_ut, mag_decl_), sns_.sigma_heading * sns_.sigma_heading);
            normalise_q();
        }
    }
}

// ekfInit with kalmanEstimator's priors; P_qq = 1/4 Xi(q) (sigma_theta^2 I) Xi(q)^T, of rank 3.
void Ekf::align(const Alignment& a) {
    for (int i = 0; i < N; ++i) {
        x_[i] = 0.f;
        for (int j = 0; j < N; ++j) P_[i][j] = 0.f;
    }
    x_[IQ] = a.q.w;
    x_[IQ + 1] = a.q.x;
    x_[IQ + 2] = a.q.y;
    x_[IQ + 3] = a.q.z;
    x_[IWB] = a.gyro_mean.x;
    x_[IWB + 1] = a.gyro_mean.y;
    x_[IWB + 2] = a.gyro_mean.z;
    baro_h0_ = a.baro_h0;

    float sigma_wb = sns_.sigma_gyro / std::sqrt(static_cast<float>(a.n_imu));
    if (sigma_wb < 1e-4f) sigma_wb = 1e-4f;
    for (int i = 0; i < 3; ++i) {
        P_[IP + i][IP + i] = pri_.sigma_p0 * pri_.sigma_p0;
        P_[IV + i][IV + i] = pri_.sigma_v0 * pri_.sigma_v0;
        P_[IAB + i][IAB + i] = pri_.sigma_ab0 * pri_.sigma_ab0;
        P_[IWB + i][IWB + i] = sigma_wb * sigma_wb;
    }
    float X[4][3];
    xi(a.q, X);
    const float s = 0.25f * pri_.sigma_theta0 * pri_.sigma_theta0;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) P_[IQ + i][IQ + j] = s * (X[i][0] * X[j][0] + X[i][1] * X[j][1] + X[i][2] * X[j][2]);
    aligned_ = true;
}

// ekfPredict: the eskfPredict kinematics, P <- F P F^T + Q.
void Ekf::predict(Vec3 am, Vec3 wm, float dt) {
    const Quat q{x_[IQ], x_[IQ + 1], x_[IQ + 2], x_[IQ + 3]};
    const Vec3 ab{x_[IAB], x_[IAB + 1], x_[IAB + 2]};
    const Vec3 wb{x_[IWB], x_[IWB + 1], x_[IWB + 2]};
    const M3 R = rotmat(q);
    const Vec3 a_body = am - ab;
    const Vec3 a_world = rotate(q, a_body) + Vec3{0.f, 0.f, gravity_};
    const Quat dq = quat_from_rotvec(dt * (wm - wb));

    float Ja[3][4], Qr[4][4], X[4][3];
    drot_dq(q, a_body, Ja);
    quat_right(dq, Qr);
    xi(q, X);

    const float pv[3] = {x_[IV], x_[IV + 1], x_[IV + 2]};
    const float aw[3] = {a_world.x, a_world.y, a_world.z};
    for (int i = 0; i < 3; ++i) {
        x_[IP + i] += dt * pv[i] + 0.5f * dt * dt * aw[i];
        x_[IV + i] += dt * aw[i];
    }
    const Quat qn = normalized(q * dq);
    x_[IQ] = qn.w;
    x_[IQ + 1] = qn.x;
    x_[IQ + 2] = qn.y;
    x_[IQ + 3] = qn.z;

    // M = F P; transpose; P = F M^T = (F P F^T)^T.
    apply_f(P_, M_, Ja, R, Qr, X, dt);
    for (int i = 0; i < N; ++i)
        for (int j = i + 1; j < N; ++j) {
            const float t = M_[i][j];
            M_[i][j] = M_[j][i];
            M_[j][i] = t;
        }
    apply_f(M_, P_, Ja, R, Qr, X, dt);

    const float qv = sns_.sigma_accel * sns_.sigma_accel * dt * dt;
    const float qq = (0.5f * sns_.sigma_gyro * dt) * (0.5f * sns_.sigma_gyro * dt);
    const float qa = sns_.sigma_accel_walk * sns_.sigma_accel_walk * dt;
    const float qw = sns_.sigma_gyro_walk * sns_.sigma_gyro_walk * dt;
    for (int i = 0; i < 3; ++i) {
        P_[IV + i][IV + i] += qv;
        P_[IAB + i][IAB + i] += qa;
        P_[IWB + i][IWB + i] += qw;
    }
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) P_[IQ + i][IQ + j] += qq * (X[i][0] * X[j][0] + X[i][1] * X[j][1] + X[i][2] * X[j][2]);
    symmetrise(P_);
}

// One scalar row of kfUpdate: K = P h / s, x += K y, P <- (I - K h^T) P (I - K h^T)^T + K r K^T.
void Ekf::scalar_update(const float* h, float y, float r) {
    float u[N];
    float s = r;
    for (int i = 0; i < N; ++i) {
        float a = 0.f;
        for (int j = 0; j < N; ++j) a += P_[i][j] * h[j];
        u[i] = a;
        s += h[i] * a;
    }
    float k[N];
    for (int i = 0; i < N; ++i) {
        k[i] = u[i] / s;
        x_[i] += k[i] * y;
    }
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

void Ekf::normalise_q() {
    const Quat q = normalized(Quat{x_[IQ], x_[IQ + 1], x_[IQ + 2], x_[IQ + 3]});
    x_[IQ] = q.w;
    x_[IQ + 1] = q.x;
    x_[IQ + 2] = q.y;
    x_[IQ + 3] = q.z;
}

State Ekf::state() const {
    State s{};
    s.t_us = t_us_;
    s.q = kQuatIdentity;
    if (!aligned_) return s;
    s.p_ned = {x_[IP], x_[IP + 1], x_[IP + 2]};
    s.v_ned = {x_[IV], x_[IV + 1], x_[IV + 2]};
    s.q = {x_[IQ], x_[IQ + 1], x_[IQ + 2], x_[IQ + 3]};
    s.w_frd = w_meas_ - gyro_bias();
    s.valid = frame_.valid();
    return s;
}

}  // namespace marv
