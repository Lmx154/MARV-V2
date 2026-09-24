// The ESKF against an analytic truth: sensors synthesised from a known p(t), q(t) at the Gazebo rates
// (IMU 1 kHz, mag 100 Hz, baro 50 Hz, GNSS 10 Hz from t = 1 ms), with the ground-contact transient at start,
// ideal or with the noise model of sitl/gazebo/marv_quad.sdf (seeded). The truth and the synthesis are double; the
// filter sees the float SensorBus only.
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>

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

void set_rates(Truth& s, const double ed[3]) {
    const double cr = std::cos(s.roll), sr = std::sin(s.roll), cp = std::cos(s.pitch), sp = std::sin(s.pitch);
    s.w = {ed[0] - ed[2] * sp, ed[1] * cr + ed[2] * sr * cp, -ed[1] * sr + ed[2] * cr * cp};
}

// Carried before it is set down: for t < move_s every axis moves as A sin^2(pi t / move_s), then rests there.
Truth carried_at(double t, double move_s) {
    Truth s{};
    const double tm = t < move_s ? t : move_s, k = kPi / move_s;
    const double sn = std::sin(k * tm), s2 = sn * sn, d1 = k * std::sin(2 * k * tm);
    const double d2 = t < move_s ? 2 * k * k * std::cos(2 * k * tm) : 0.0;
    const double A[3] = {0.5, 0.3, -0.4}, B[3] = {0.3, -0.2, 0.5};
    s.p = {A[0] * s2, A[1] * s2, A[2] * s2};
    s.v = {A[0] * d1, A[1] * d1, A[2] * d1};
    s.a = {A[0] * d2, A[1] * d2, A[2] * d2};
    s.roll = B[0] * s2;
    s.pitch = B[1] * s2;
    s.yaw = 0.5 * kPi + B[2] * s2;
    const double ed[3] = {B[0] * d1, B[1] * d1, B[2] * d1};
    set_rates(s, ed);
    return s;
}

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
    set_rates(s, ed);
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
    double move_s = 0;      // > 0: carried_at(t, move_s) instead of truth_at(t)
    bool noisy = false;     // the noise model of sitl/gazebo/marv_quad.sdf, drawn from seed
    unsigned seed = 1;
};

// sitl/gazebo/marv_quad.sdf, per sample: LSM6DSV white noise and turn-on bias sigmas, BMP580, MMC5603NJ, and the
// assumed M10-class GNSS with its horizontal sigma in degrees as gz-sensors applies it.
constexpr double kSdAccel = 0.01316, kSdAccelBias = 0.1177, kSdGyro = 1.093e-3, kSdGyroBias = 0.01745;
constexpr double kSdBaroPa = 0.25, kSdMagUt = 0.2, kSdGnssDeg = 9.0e-6, kSdGnssAlt = 1.5, kSdGnssVel = 0.05;

struct Result {
    bool aligned = false;
    double align_t = 0, align_att_deg = 0;
    double att_max_deg = 0, vel_max = 0, pos_max = 0;
    double att_max_after30_deg = 0;
    // Root mean squares after alignment. Horizontal position against the truth about the first fix (the filter's
    // origin); vertical against the truth itself, since the baro (zeroed at alignment) outweighs GNSS altitude.
    double att_rms_deg = 0, vel_rms = 0, pos_h_rms = 0, pos_v_rms = 0;
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
    std::mt19937 rng(o.seed);
    std::normal_distribution<double> gauss(0.0, 1.0);
    auto nz = [&](double sd) { return o.noisy ? sd * gauss(rng) : 0.0; };
    const double ab[3] = {o.accel_bias.x + nz(kSdAccelBias), o.accel_bias.y + nz(kSdAccelBias),
                          o.accel_bias.z + nz(kSdAccelBias)};
    const double wb[3] = {o.gyro_bias.x + nz(kSdGyroBias), o.gyro_bias.y + nz(kSdGyroBias),
                          o.gyro_bias.z + nz(kSdGyroBias)};
    bool have_origin = false;
    D3 origin_err{0, 0, 0};  // z unused: see Result
    double sum_att = 0, sum_vel = 0, sum_h = 0, sum_v = 0;
    long n = 0;
    for (long k = 1; k <= steps; ++k) {
        const double t = k * 1e-3;
        const Truth s = o.move_s > 0 ? carried_at(t, o.move_s) : truth_at(t);
        double R[3][3];
        rotmat(s, R);
        bus.t_us = static_cast<std::uint64_t>(k) * 1000u;
        bus.fresh = kImu;
        const double sf[3] = {s.a.x, s.a.y, s.a.z - kG};
        Vec3 acc = body_of(R, sf, true);
        if (k == 1) acc.z = -19.7f;  // ground-contact transient
        bus.imu.accel_frd = {acc.x + static_cast<float>(ab[0] + nz(kSdAccel)),
                             acc.y + static_cast<float>(ab[1] + nz(kSdAccel)),
                             acc.z + static_cast<float>(ab[2] + nz(kSdAccel))};
        bus.imu.gyro_frd = {static_cast<float>(s.w.x + wb[0] + nz(kSdGyro)),
                            static_cast<float>(s.w.y + wb[1] + nz(kSdGyro)),
                            static_cast<float>(s.w.z + wb[2] + nz(kSdGyro))};
        if (k % 10 == 0) {
            bus.fresh |= kMag;
            const Vec3 m = body_of(R, kMagNed, !o.flip_mag);
            bus.mag.field_frd_ut = {m.x + static_cast<float>(nz(kSdMagUt)), m.y + static_cast<float>(nz(kSdMagUt)),
                                    m.z + static_cast<float>(nz(kSdMagUt))};
        }
        if (k % 20 == 0) {
            bus.fresh |= kBaro;
            const double h = -s.p.z;  // sea level at the world origin
            bus.baro.pressure_pa =
                static_cast<float>(101325.0 * std::pow(1 - h / 44330.77, 1 / 0.190263) + nz(kSdBaroPa));
            bus.baro.temperature_c = 15.f;
        }
        if ((k - 1) % 100 == 0) {
            bus.fresh |= kGnss;
            const double lat = kLat0 + s.p.x / (Rn + kAlt0) + nz(kSdGnssDeg) * kDeg;
            const double lon = kLon0 + s.p.y / ((Re + kAlt0) * std::cos(kLat0)) + nz(kSdGnssDeg) * kDeg;
            const double h = kAlt0 - s.p.z + nz(kSdGnssAlt);
            bus.gnss.lat_e7 = static_cast<std::int32_t>(std::lround(lat / kDeg * 1e7));
            bus.gnss.lon_e7 = static_cast<std::int32_t>(std::lround(lon / kDeg * 1e7));
            bus.gnss.alt_m = static_cast<float>(h);
            bus.gnss.vel_ned = {static_cast<float>(s.v.x + nz(kSdGnssVel)), static_cast<float>(s.v.y + nz(kSdGnssVel)),
                                static_cast<float>(s.v.z + nz(kSdGnssVel))};
            bus.gnss.fix = true;
            if (!have_origin) {
                have_origin = true;
                origin_err = {(lat - kLat0) * (Rn + kAlt0) - s.p.x,
                              (lon - kLon0) * (Re + kAlt0) * std::cos(kLat0) - s.p.y, -(h - kAlt0) - s.p.z};
            }
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
        const double dn = e.p_ned.x - (s.p.x - origin_err.x), de = e.p_ned.y - (s.p.y - origin_err.y);
        const double dd = e.p_ned.z - s.p.z, ev = err3(e.v_ned, s.v);
        sum_att += att * att;
        sum_vel += ev * ev;
        sum_h += dn * dn + de * de;
        sum_v += dd * dd;
        ++n;
    }
    if (n > 0) {
        r.att_rms_deg = std::sqrt(sum_att / static_cast<double>(n));
        r.vel_rms = std::sqrt(sum_vel / static_cast<double>(n));
        r.pos_h_rms = std::sqrt(sum_h / static_cast<double>(n));
        r.pos_v_rms = std::sqrt(sum_v / static_cast<double>(n));
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
    // 5. The sdf noise model, ten seeds (each draws new turn-on biases), graded on the SITL shadow gate: attitude RMS
    //    < 1 deg and max < 3 deg, velocity RMS < 0.2 m/s, horizontal RMS < 1 m and vertical RMS < 0.5 m, all after
    //    alignment, position in the filter's own frame (about the first fix, whose own error no filter can see).
    {
        double worst[5] = {0, 0, 0, 0, 0};
        for (unsigned seed = 1; seed <= 10; ++seed) {
            Options o;
            o.noisy = true;
            o.seed = seed;
            const Result r = run(o);
            const double v[5] = {r.att_rms_deg, r.att_max_deg, r.vel_rms, r.pos_h_rms, r.pos_v_rms};
            for (int i = 0; i < 5; ++i) worst[i] = std::fmax(worst[i], v[i]);
            std::printf("test5 seed %2u: aligned %.4f deg, att rms %.3f max %.3f (after 30 s %.3f) deg, vel rms %.4f m/s, "
                        "pos rms h %.3f v %.3f m\n", seed, r.align_att_deg, v[0], v[1], r.att_max_after30_deg, v[2], v[3], v[4]);
            CHECK(r.aligned);
        }
        std::printf("test5 noisy, worst of 10 seeds: att rms %.3f max %.3f deg, vel rms %.4f m/s, pos rms h %.3f v %.3f m\n",
                    worst[0], worst[1], worst[2], worst[3], worst[4]);
        CHECK(worst[0] < 1.0);
        CHECK(worst[1] < 3.0);
        CHECK(worst[2] < 0.2);
        CHECK(worst[3] < 1.0);
        CHECK(worst[4] < 0.5);
    }
    // 6. Carried for 3 s, then set down: no state until align_window_s (1 s) of stillness after the motion ends,
    //    then the rest attitude. Ideal sensors, and the noise model (which must not restart the window: 0.1 s slack).
    for (int noisy = 0; noisy <= 1; ++noisy) {
        Options o;
        o.move_s = 3.0;
        o.seconds = 5.0;
        o.noisy = noisy != 0;
        const Result r = run(o);
        std::printf("test6 carried until 3 s (%s): aligned at %.3f s, attitude error %.4f deg\n", noisy ? "noisy" : "ideal",
                    r.align_t, r.align_att_deg);
        CHECK(r.aligned);
        CHECK(r.align_t >= o.move_s + 1.0 && r.align_t < o.move_s + 1.1);
        CHECK(r.align_att_deg < (noisy ? 1.5 : 0.2));
    }
    std::printf(failures ? "FAIL (%d)\n" : "PASS\n", failures);
    return failures ? EXIT_FAILURE : EXIT_SUCCESS;
}
