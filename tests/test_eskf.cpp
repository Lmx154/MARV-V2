// The ESKF against an analytic truth: ideal sensors synthesised from a known p(t), q(t) at the Gazebo rates
// (IMU 1 kHz, mag 100 Hz, baro 50 Hz, GNSS 10 Hz from t = 1 ms), with the ground-contact transient at start.
// The truth and the synthesis are double; the filter sees the float SensorBus only.
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include <marv/fsw/eskf.hpp>
#include <marv/fsw/math.hpp>

using namespace marv;

static int failures = 0;
#define CHECK(c)                                                     \
    do {                                                             \
        if (!(c)) {                                                  \
            std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); \
            ++failures;                                              \
        }                                                            \
    } while (0)

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0;
constexpr double kG = 9.8066;
constexpr double kMagNed[3] = {21.62762, 0.96861, 42.91632};
constexpr double kRest = 2.0;  // s at rest before the motion
constexpr double kLat0 = 47.3977419 * kDeg, kLon0 = 8.5455938 * kDeg, kAlt0 = 488.0;

struct D3 {
    double x, y, z;
};

struct Truth {
    D3 p, v, a;              // NED, relative to the start
    double roll, pitch, yaw;  // ZYX
    D3 w;                     // body rate, FRD
};

// Motion after kRest: every axis moves as A (1 - cos(w t)), so velocity and body rate start from zero.
Truth truth_at(double t) {
    Truth s{};
    const double tm = t > kRest ? t - kRest : 0.0;
    const double A[3] = {4.0, 3.0, -2.0}, W[3] = {2 * kPi / 16, 2 * kPi / 12, 2 * kPi / 10};
    double p[3], v[3], a[3];
    for (int i = 0; i < 3; ++i) {
        p[i] = A[i] * (1 - std::cos(W[i] * tm));
        v[i] = A[i] * W[i] * std::sin(W[i] * tm);
        a[i] = t > kRest ? A[i] * W[i] * W[i] * std::cos(W[i] * tm) : 0.0;
    }
    s.p = {p[0], p[1], p[2]};
    s.v = {v[0], v[1], v[2]};
    s.a = {a[0], a[1], a[2]};
    const double B[3] = {0.15, -0.12, 0.8}, E[3] = {2 * kPi / 7, 2 * kPi / 9, 2 * kPi / 15};
    double e[3], ed[3];
    for (int i = 0; i < 3; ++i) {
        e[i] = B[i] * (1 - std::cos(E[i] * tm));
        ed[i] = B[i] * E[i] * std::sin(E[i] * tm);
    }
    s.roll = e[0];
    s.pitch = e[1];
    s.yaw = 0.5 * kPi + e[2];
    const double cr = std::cos(s.roll), sr = std::sin(s.roll), cp = std::cos(s.pitch), sp = std::sin(s.pitch);
    s.w = {ed[0] - ed[2] * sp, ed[1] * cr + ed[2] * sr * cp, -ed[1] * sr + ed[2] * cr * cp};
    return s;
}

// R (body -> NED) of ZYX Euler angles.
void rotmat(const Truth& s, double R[3][3]) {
    const double cr = std::cos(s.roll), sr = std::sin(s.roll), cp = std::cos(s.pitch), sp = std::sin(s.pitch);
    const double cy = std::cos(s.yaw), sy = std::sin(s.yaw);
    const double r[3][3] = {{cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr},
                            {sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr},
                            {-sp, cp * sr, cp * cr}};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) R[i][j] = r[i][j];
}

Vec3 body_of(const double R[3][3], const double v[3], bool transpose) {
    double o[3];
    for (int i = 0; i < 3; ++i) o[i] = transpose ? R[0][i] * v[0] + R[1][i] * v[1] + R[2][i] * v[2]
                                                  : R[i][0] * v[0] + R[i][1] * v[1] + R[i][2] * v[2];
    return {static_cast<float>(o[0]), static_cast<float>(o[1]), static_cast<float>(o[2])};
}

struct Options {
    double seconds = kRest + 60.0;
    D3 accel_bias{0, 0, 0};
    D3 gyro_bias{0, 0, 0};
    bool flip_mag = false;  // negative control: R m instead of R^T m
};

struct Result {
    bool aligned = false;
    double align_t = 0, align_att_deg = 0;
    double att_max_deg = 0, vel_max = 0, pos_max = 0;
    double att_max_after30_deg = 0;
    Vec3 ab{}, wb{};
    double ns_per_update = 0;
};

double err3(Vec3 a, D3 b) {
    const double dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

Result run(const Options& o) {
    Eskf f;
    SensorBus bus{};
    Result r;
    const double Rn = 6378137.0 * (1 - 6.69437999014e-3) / std::pow(1 - 6.69437999014e-3 * std::sin(kLat0) * std::sin(kLat0), 1.5);
    const double Re = 6378137.0 / std::sqrt(1 - 6.69437999014e-3 * std::sin(kLat0) * std::sin(kLat0));
    const long steps = std::lround(o.seconds * 1000.0);
    std::chrono::steady_clock::duration spent{};
    for (long k = 1; k <= steps; ++k) {
        const double t = k * 1e-3;
        const Truth s = truth_at(t);
        double R[3][3];
        rotmat(s, R);
        bus.t_us = static_cast<std::uint64_t>(k) * 1000u;
        bus.fresh = kImu;
        const double sf[3] = {s.a.x, s.a.y, s.a.z - kG};
        Vec3 acc = body_of(R, sf, true);
        if (k == 1) acc.z = -19.7f;  // ground-contact transient
        bus.imu.accel_frd = {acc.x + static_cast<float>(o.accel_bias.x), acc.y + static_cast<float>(o.accel_bias.y),
                             acc.z + static_cast<float>(o.accel_bias.z)};
        bus.imu.gyro_frd = {static_cast<float>(s.w.x + o.gyro_bias.x), static_cast<float>(s.w.y + o.gyro_bias.y),
                            static_cast<float>(s.w.z + o.gyro_bias.z)};
        if (k % 10 == 0) {
            bus.fresh |= kMag;
            bus.mag.field_frd_ut = body_of(R, kMagNed, !o.flip_mag);
        }
        if (k % 20 == 0) {
            bus.fresh |= kBaro;
            const double h = -s.p.z;  // sea level at the world origin
            bus.baro.pressure_pa = static_cast<float>(101325.0 * std::pow(1 - h / 44330.77, 1 / 0.190263));
            bus.baro.temperature_c = 15.f;
        }
        if ((k - 1) % 100 == 0) {
            bus.fresh |= kGnss;
            const double h = kAlt0 - s.p.z;
            bus.gnss.lat_e7 = static_cast<std::int32_t>(std::lround((kLat0 + s.p.x / (Rn + kAlt0)) / kDeg * 1e7));
            bus.gnss.lon_e7 =
                static_cast<std::int32_t>(std::lround((kLon0 + s.p.y / ((Re + kAlt0) * std::cos(kLat0))) / kDeg * 1e7));
            bus.gnss.alt_m = static_cast<float>(h);
            bus.gnss.vel_ned = {static_cast<float>(s.v.x), static_cast<float>(s.v.y), static_cast<float>(s.v.z)};
            bus.gnss.fix = true;
        }
        const auto t0 = std::chrono::steady_clock::now();
        f.update(bus);
        spent += std::chrono::steady_clock::now() - t0;

        const State e = f.state();
        if (!e.valid) continue;
        const Quat qt = quat_from_euler(static_cast<float>(s.roll), static_cast<float>(s.pitch), static_cast<float>(s.yaw));
        const double att = quat_angle(e.q, qt) / kDeg;
        if (!r.aligned) {
            r.aligned = true;
            r.align_t = t;
            r.align_att_deg = att;
        }
        r.att_max_deg = std::fmax(r.att_max_deg, att);
        if (t >= kRest + 30.0) r.att_max_after30_deg = std::fmax(r.att_max_after30_deg, att);
        r.vel_max = std::fmax(r.vel_max, err3(e.v_ned, s.v));
        r.pos_max = std::fmax(r.pos_max, err3(e.p_ned, s.p));
    }
    r.ab = f.accel_bias();
    r.wb = f.gyro_bias();
    r.ns_per_update = std::chrono::duration<double, std::nano>(spent).count() / static_cast<double>(steps);
    return r;
}

bool within(float est, double truth, double frac) { return std::fabs(est - truth) <= frac * std::fabs(truth); }

}  // namespace

int main() {
    // 1. Ideal sensors, 60 s of translation and rotation on all axes.
    {
        const Result r = run(Options{});
        std::printf("test1 trajectory: aligned at %.3f s, att max %.4f deg, vel max %.4f m/s, pos max %.4f m, %.0f ns/update\n",
                    r.align_t, r.att_max_deg, r.vel_max, r.pos_max, r.ns_per_update);
        CHECK(r.aligned);
        CHECK(r.att_max_deg < 0.5);
        CHECK(r.vel_max < 0.05);
        CHECK(r.pos_max < 0.15);
    }
    // 2. Constant gyro and accelerometer biases.
    {
        Options o;
        o.gyro_bias = {0.01, -0.02, 0.005};
        o.accel_bias = {0.05, -0.05, 0.1};
        const Result r = run(o);
        std::printf("test2 biases: gyro (%.5f %.5f %.5f) rad/s, accel (%.4f %.4f %.4f) m/s^2, att max %.3f deg, after 30 s %.3f deg\n",
                    static_cast<double>(r.wb.x), static_cast<double>(r.wb.y), static_cast<double>(r.wb.z),
                    static_cast<double>(r.ab.x), static_cast<double>(r.ab.y), static_cast<double>(r.ab.z), r.att_max_deg,
                    r.att_max_after30_deg);
        CHECK(within(r.wb.x, o.gyro_bias.x, 0.2) && within(r.wb.y, o.gyro_bias.y, 0.2) && within(r.wb.z, o.gyro_bias.z, 0.2));
        CHECK(within(r.ab.x, o.accel_bias.x, 0.2) && within(r.ab.y, o.accel_bias.y, 0.2) && within(r.ab.z, o.accel_bias.z, 0.2));
        CHECK(r.att_max_after30_deg < 2.0);
    }
    // 3. At rest through the contact transient: roll = pitch = 0, yaw = +90 deg.
    {
        Options o;
        o.seconds = 1.5;
        const Result r = run(o);
        std::printf("test3 rest alignment: aligned at %.3f s, attitude error %.5f deg\n", r.align_t, r.align_att_deg);
        CHECK(r.aligned);
        CHECK(r.align_att_deg < 0.2);
    }
    // 4. Negative control: the magnetometer synthesised with the wrong rotation must fail test 1.
    {
        Options o;
        o.flip_mag = true;
        const Result r = run(o);
        std::printf("test4 negative control (mag R m): att max %.2f deg, vel max %.3f m/s, pos max %.3f m\n", r.att_max_deg,
                    r.vel_max, r.pos_max);
        CHECK(r.att_max_deg >= 0.5);
    }
    std::printf(failures ? "FAIL (%d)\n" : "PASS\n", failures);
    return failures ? EXIT_FAILURE : EXIT_SUCCESS;
}
