// A synthetic flight with known truth, and the ideal sensors it implies, at the Gazebo rates (IMU 1 kHz, mag 100 Hz,
// baro 50 Hz, GNSS 10 Hz from t = 1 ms): rest (2 s by default) at rest yawed +90 deg (with the ground-contact
// transient on the first IMU sample), then translation on all three axes and rotation about all three, each
// A (1 - cos(w t)) so velocity and body rate start from zero. The same trajectory as tests/test_eskf.cpp. Conventions
// of contracts.hpp: NED / FRD, specific force R^T (a - g). The Earth field is the WMM's at the origin (what the
// estimators look up at the first fix) unless Options::mag_ned sets one. The truth and the synthesis are double; the
// estimators see the float SensorBus only.
#pragma once

#include <cmath>
#include <cstdint>

#include <marv/fsw/contracts.hpp>
#include <marv/fsw/geo_mag.hpp>
#include <marv/fsw/math.hpp>

namespace synth {

using namespace marv;

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0;
constexpr double kG = 9.8066;
constexpr double kRest = 2.0;
constexpr double kLat0 = 47.3977419 * kDeg, kLon0 = 8.5455938 * kDeg, kAlt0 = 488.0;
constexpr std::int32_t kLat0E7 = 473977419, kLon0E7 = 85455938;

struct Options {
    double seconds = kRest + 60.0;
    double rest = kRest;  // s at rest before the motion
    double accel_bias[3] = {0, 0, 0};
    double gyro_bias[3] = {0, 0, 0};
    double mag_ned[3] = {0, 0, 0};  // microtesla; all 0: earth_field_ned_ut at the origin
    bool flip_mag = false;  // negative control: the field synthesised as R m instead of R^T m
};

struct Truth {
    double t;
    double p[3], v[3], a[3];  // NED, relative to the start
    double roll, pitch, yaw;  // ZYX
    double w[3];              // body rate, FRD
    Quat q() const { return quat_from_euler(static_cast<float>(roll), static_cast<float>(pitch), static_cast<float>(yaw)); }
};

inline Truth truth_at(double t, double rest = kRest) {
    Truth s{};
    s.t = t;
    const double tm = t > rest ? t - rest : 0.0;
    const double A[3] = {4.0, 3.0, -2.0}, W[3] = {2 * kPi / 16, 2 * kPi / 12, 2 * kPi / 10};
    for (int i = 0; i < 3; ++i) {
        s.p[i] = A[i] * (1 - std::cos(W[i] * tm));
        s.v[i] = A[i] * W[i] * std::sin(W[i] * tm);
        s.a[i] = t > rest ? A[i] * W[i] * W[i] * std::cos(W[i] * tm) : 0.0;
    }
    const double B[3] = {0.15, -0.12, 0.8}, E[3] = {2 * kPi / 7, 2 * kPi / 9, 2 * kPi / 15};
    double ed[3];
    double e[3];
    for (int i = 0; i < 3; ++i) {
        e[i] = B[i] * (1 - std::cos(E[i] * tm));
        ed[i] = B[i] * E[i] * std::sin(E[i] * tm);
    }
    s.roll = e[0];
    s.pitch = e[1];
    s.yaw = 0.5 * kPi + e[2];
    const double cr = std::cos(s.roll), sr = std::sin(s.roll), cp = std::cos(s.pitch), sp = std::sin(s.pitch);
    s.w[0] = ed[0] - ed[2] * sp;
    s.w[1] = ed[1] * cr + ed[2] * sr * cp;
    s.w[2] = -ed[1] * sr + ed[2] * cr * cp;
    return s;
}

// Steps k = 1, 2, ... at 1 ms, each producing the SensorBus of that tick and its truth.
class Flight {
public:
    explicit Flight(const Options& o) : o_(o) {
        const double s = std::sin(kLat0);
        const double d = 1 - 6.69437999014e-3 * s * s;
        rn_ = 6378137.0 * (1 - 6.69437999014e-3) / std::pow(d, 1.5);
        re_ = 6378137.0 / std::sqrt(d);
        const Vec3 m = earth_field_ned_ut(kLat0E7, kLon0E7);
        const bool set = o.mag_ned[0] != 0 || o.mag_ned[1] != 0 || o.mag_ned[2] != 0;
        mag_[0] = set ? o.mag_ned[0] : m.x;
        mag_[1] = set ? o.mag_ned[1] : m.y;
        mag_[2] = set ? o.mag_ned[2] : m.z;
    }

    bool next(SensorBus& bus, Truth& s) {
        ++k_;
        if (static_cast<double>(k_) * 1e-3 > o_.seconds + 1e-9) return false;
        const double t = static_cast<double>(k_) * 1e-3;
        s = truth_at(t, o_.rest);
        double R[3][3];
        rotmat(s, R);
        bus.t_us = static_cast<std::uint64_t>(k_) * 1000u;
        bus.fresh = kImu;
        const double sf[3] = {s.a[0], s.a[1], s.a[2] - kG};
        Vec3 acc = body_of(R, sf, true);
        if (k_ == 1) acc.z = -19.7f;  // ground-contact transient
        bus.imu.accel_frd = {acc.x + static_cast<float>(o_.accel_bias[0]), acc.y + static_cast<float>(o_.accel_bias[1]),
                             acc.z + static_cast<float>(o_.accel_bias[2])};
        bus.imu.gyro_frd = {static_cast<float>(s.w[0] + o_.gyro_bias[0]), static_cast<float>(s.w[1] + o_.gyro_bias[1]),
                            static_cast<float>(s.w[2] + o_.gyro_bias[2])};
        if (k_ % 10 == 0) {
            bus.fresh |= kMag;
            bus.mag.field_frd_ut = body_of(R, mag_, !o_.flip_mag);
        }
        if (k_ % 20 == 0) {
            bus.fresh |= kBaro;
            const double h = -s.p[2];  // sea level at the world origin
            bus.baro.pressure_pa = static_cast<float>(101325.0 * std::pow(1 - h / 44330.77, 1 / 0.190263));
            bus.baro.temperature_c = 15.f;
        }
        if ((k_ - 1) % 100 == 0) {
            bus.fresh |= kGnss;
            bus.gnss.lat_e7 = static_cast<std::int32_t>(std::lround((kLat0 + s.p[0] / (rn_ + kAlt0)) / kDeg * 1e7));
            bus.gnss.lon_e7 =
                static_cast<std::int32_t>(std::lround((kLon0 + s.p[1] / ((re_ + kAlt0) * std::cos(kLat0))) / kDeg * 1e7));
            bus.gnss.alt_m = static_cast<float>(kAlt0 - s.p[2]);
            bus.gnss.vel_ned = {static_cast<float>(s.v[0]), static_cast<float>(s.v[1]), static_cast<float>(s.v[2])};
            bus.gnss.fix = true;
        }
        return true;
    }

private:
    static void rotmat(const Truth& s, double R[3][3]) {
        const double cr = std::cos(s.roll), sr = std::sin(s.roll), cp = std::cos(s.pitch), sp = std::sin(s.pitch);
        const double cy = std::cos(s.yaw), sy = std::sin(s.yaw);
        const double r[3][3] = {{cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr},
                                {sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr},
                                {-sp, cp * sr, cp * cr}};
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) R[i][j] = r[i][j];
    }

    static Vec3 body_of(const double R[3][3], const double v[3], bool transpose) {
        double o[3];
        for (int i = 0; i < 3; ++i) o[i] = transpose ? R[0][i] * v[0] + R[1][i] * v[1] + R[2][i] * v[2]
                                                      : R[i][0] * v[0] + R[i][1] * v[1] + R[i][2] * v[2];
        return {static_cast<float>(o[0]), static_cast<float>(o[1]), static_cast<float>(o[2])};
    }

    Options o_;
    long k_ = 0;
    double rn_ = 0, re_ = 0;
    double mag_[3] = {0, 0, 0};  // the Earth field synthesised, NED, microtesla
};

}  // namespace synth
