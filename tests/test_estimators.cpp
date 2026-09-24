// The preset estimators (EKF, UKF, Mahony, complementary) against the analytic truth of synth.hpp, and preset
// selection through the shared packet dispatch (SetPreset, Reset, Reboot).
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#include <marv/fsw/complementary.hpp>
#include <marv/fsw/ekf.hpp>
#include <marv/fsw/eskf.hpp>
#include <marv/fsw/fsw.hpp>
#include <marv/fsw/mahony.hpp>
#include <marv/fsw/math.hpp>
#include <marv/fsw/presets.hpp>
#include <marv/fsw/ukf.hpp>
#include <marv/link/protocol.hpp>

#include "../firmware/src/dispatch.hpp"
#include "synth.hpp"

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

struct Result {
    bool aligned = false;
    double align_t = 0;
    double att_max_deg = 0, vel_max = 0, pos_max = 0;
    double att_max_after30_deg = 0;
    double us_per_update = 0;
    double fix_t = -1;     // the first GNSS fix delivered
    bool dropped = false;  // valid went true -> false
};

double err3(Vec3 a, const double b[3]) {
    const double dx = a.x - b[0], dy = a.y - b[1], dz = a.z - b[2];
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// GNSS is withheld before gnss_from (s); position errors are about the truth at the first fix delivered, the
// estimator's origin.
template <class E> Result run(E& f, const synth::Options& o, double gnss_from = 0.0) {
    synth::Flight flight(o);
    SensorBus bus{};
    synth::Truth s{};
    Result r;
    long n = 0;
    bool was_valid = false;
    double origin[3] = {0, 0, 0};
    std::chrono::steady_clock::duration spent{};
    while (flight.next(bus, s)) {
        if (s.t < gnss_from) bus.fresh = static_cast<std::uint8_t>(bus.fresh & ~kGnss);
        if ((bus.fresh & kGnss) && r.fix_t < 0) {
            r.fix_t = s.t;
            for (int i = 0; i < 3; ++i) origin[i] = s.p[i];
        }
        const auto t0 = std::chrono::steady_clock::now();
        f.update(bus);
        spent += std::chrono::steady_clock::now() - t0;
        ++n;
        const State e = f.state();
        if (was_valid && !e.valid) r.dropped = true;
        was_valid = e.valid;
        if (!e.valid) continue;
        if (!r.aligned) {
            r.aligned = true;
            r.align_t = s.t;
        }
        const double att = quat_angle(e.q, s.q()) / synth::kDeg;
        r.att_max_deg = std::fmax(r.att_max_deg, att);
        if (s.t >= synth::kRest + 30.0) r.att_max_after30_deg = std::fmax(r.att_max_after30_deg, att);
        r.vel_max = std::fmax(r.vel_max, err3(e.v_ned, s.v));
        const double p[3] = {s.p[0] - origin[0], s.p[1] - origin[1], s.p[2] - origin[2]};
        r.pos_max = std::fmax(r.pos_max, err3(e.p_ned, p));
    }
    CHECK(!r.dropped);
    r.us_per_update = std::chrono::duration<double, std::micro>(spent).count() / static_cast<double>(n);
    return r;
}

void print(const char* name, const Result& r) {
    std::printf("%-28s aligned at %.3f s, att max %.4f deg (after 30 s %.4f), vel max %.4f m/s, pos max %.4f m, %.2f us/update\n",
                name, r.align_t, r.att_max_deg, r.att_max_after30_deg, r.vel_max, r.pos_max, r.us_per_update);
}

struct Limits {
    double att_deg, vel, pos;
};

void check(const Result& r, const Limits& l) {
    CHECK(r.aligned);
    CHECK(r.align_t < 1.2);
    CHECK(r.att_max_deg < l.att_deg);
    CHECK(r.vel_max < l.vel);
    CHECK(r.pos_max < l.pos);
}

// ---- preset selection through fw::dispatch ------------------------------------------------------

struct TestPlatform {
    std::vector<std::uint8_t> out;
    std::uint8_t stored = 0;
    int reboots = 0;
    void send(const std::uint8_t* p, std::size_t n) { out.insert(out.end(), p, p + n); }
    std::uint8_t load_preset() { return stored; }
    void store_preset(std::uint8_t id) { stored = id; }
    void reboot() { ++reboots; }
};

template <class T> void deliver(const T& msg, Fsw& fsw, TestPlatform& pf) {
    std::uint8_t frame[link::kMaxFrame];
    const std::size_t n = link::encode(msg, frame);
    link::Decoder dec;
    for (std::size_t i = 0; i < n; ++i)
        if (dec.push(frame[i])) fw::dispatch(dec.packet(), fsw, pf);
}

// Runs 1.5 s of the synthetic flight through dispatch and a reference estimator side by side. Returns true when every
// Telemetry carried `preset` and an estimate bit-identical to the reference's.
template <class E> bool flies(Fsw& fsw, TestPlatform& pf, std::uint8_t preset) {
    E ref;
    synth::Options o;
    o.seconds = 1.5;
    synth::Flight flight(o);
    SensorBus bus{};
    synth::Truth s{};
    link::Decoder dec;
    bool ok = true, valid = false;
    while (flight.next(bus, s)) {
        pf.out.clear();
        deliver(bus, fsw, pf);
        ref.update(bus);
        const State e = ref.state();
        Telemetry t{};
        bool got = false;
        for (std::uint8_t b : pf.out)
            if (dec.push(b) && dec.packet().as(t)) got = true;
        ok = ok && got && t.preset == preset && t.est.valid == e.valid &&
             std::memcmp(&t.est.q, &e.q, sizeof(Quat)) == 0 && std::memcmp(&t.est.p_ned, &e.p_ned, sizeof(Vec3)) == 0;
        valid = valid || t.est.valid;
    }
    return ok && valid;
}

}  // namespace

int main() {
    const synth::Options ideal;

    // 1. Ideal sensors, 60 s of translation and rotation on all axes, per estimator.
    // The Kalman filters get test_eskf.cpp's ESKF limits (0.5 deg, 0.05 m/s, 0.15 m): same measurements, same model.
    {
        Ekf f;
        const Result r = run(f, ideal);
        print("ekf", r);
        check(r, {0.5, 0.05, 0.15});
    }
    {
        Ukf f;
        const Result r = run(f, ideal);
        print("ukf", r);
        check(r, {0.5, 0.05, 0.15});
        CHECK(f.cholesky_failures() == 0);
    }
    // The complementary filters take the specific force for gravity whenever | |a_m| - g | is within the 0.25 m/s^2
    // gate, which the trajectory's accelerations (up to ~1.2 m/s^2, apparent tilt atan(a / g) up to ~7 deg) pass: their
    // error is that model error, not noise. Mahony's PI loop (natural frequency sqrt(kI) = 0.32 rad/s, the trajectory's
    // 0.39-0.9 rad/s) winds it into the bias estimate and roughly doubles it: measured 9.0 deg against 3.9 deg, and
    // 0.04 / 0.03 deg for both with the gate closed (gyro and magnetometer only). Limits: the measured maxima plus
    // about a third.
    {
        Mahony f;
        const Result r = run(f, ideal);
        print("mahony", r);
        check(r, {12.0, 0.5, 0.4});
    }
    {
        Complementary f;
        const Result r = run(f, ideal);
        print("complementary", r);
        check(r, {5.5, 0.25, 0.2});
    }
    // 2. Constant gyro and accelerometer biases: the Kalman filters estimate both (test_eskf.cpp test 2 limits);
    // Mahony's integral term absorbs the gyro bias within its test 1 limit.
    {
        synth::Options o;
        o.gyro_bias[0] = 0.01, o.gyro_bias[1] = -0.02, o.gyro_bias[2] = 0.005;
        o.accel_bias[0] = 0.05, o.accel_bias[1] = -0.05, o.accel_bias[2] = 0.1;
        const auto within = [](Vec3 est, const double (&truth)[3], double frac) {
            return std::fabs(est.x - truth[0]) <= frac * std::fabs(truth[0]) &&
                   std::fabs(est.y - truth[1]) <= frac * std::fabs(truth[1]) &&
                   std::fabs(est.z - truth[2]) <= frac * std::fabs(truth[2]);
        };
        Ekf e;
        const Result re = run(e, o);
        print("ekf biased", re);
        CHECK(within(e.gyro_bias(), o.gyro_bias, 0.2) && within(e.accel_bias(), o.accel_bias, 0.2));
        CHECK(re.att_max_after30_deg < 2.0);
        Ukf u;
        const Result ru = run(u, o);
        print("ukf biased", ru);
        CHECK(within(u.gyro_bias(), o.gyro_bias, 0.2) && within(u.accel_bias(), o.accel_bias, 0.2));
        CHECK(ru.att_max_after30_deg < 2.0);
        CHECK(u.cholesky_failures() == 0);
        Mahony m;
        const Result rm = run(m, o);
        print("mahony biased", rm);
        CHECK(rm.att_max_after30_deg < 12.0);
        std::printf("gyro bias ekf (%.5f %.5f %.5f) ukf (%.5f %.5f %.5f) mahony (%.5f %.5f %.5f)\n", e.gyro_bias().x,
                    e.gyro_bias().y, e.gyro_bias().z, u.gyro_bias().x, u.gyro_bias().y, u.gyro_bias().z, m.gyro_bias().x,
                    m.gyro_bias().y, m.gyro_bias().z);
    }
    // 3. Negative control: the magnetometer synthesised with the wrong rotation must fail test 1's UKF limits.
    {
        synth::Options o;
        o.flip_mag = true;
        Ukf f;
        const Result r = run(f, o);
        print("ukf negative control", r);
        CHECK(r.att_max_deg >= 0.5);
    }
    // 4. Preset selection: SetPreset stores, Reset applies, an unknown id runs preset 0, Reboot restarts.
    {
        TestPlatform pf;
        Fsw fsw{pf.load_preset()};
        CHECK(fsw.preset() == 0);
        deliver(link::SetPreset{2}, fsw, pf);
        CHECK(pf.stored == 2);
        CHECK(fsw.preset() == 0);  // stored, not yet applied
        deliver(link::Reset{}, fsw, pf);
        CHECK(fsw.preset() == 2);
        CHECK(flies<Ukf>(fsw, pf, 2));
        deliver(link::Reset{}, fsw, pf);
        CHECK(!flies<Eskf>(fsw, pf, 2));  // the check can fail: the same flight on another estimator differs

        const std::uint8_t ids[] = {0, 1, 3, 4};
        for (std::uint8_t id : ids) {
            deliver(link::SetPreset{id}, fsw, pf);
            deliver(link::Reset{}, fsw, pf);
            CHECK(fsw.preset() == id);
        }
        deliver(link::SetPreset{0}, fsw, pf);
        deliver(link::Reset{}, fsw, pf);
        CHECK(flies<Eskf>(fsw, pf, 0));
        deliver(link::SetPreset{1}, fsw, pf);
        deliver(link::Reset{}, fsw, pf);
        CHECK(flies<Ekf>(fsw, pf, 1));
        deliver(link::SetPreset{3}, fsw, pf);
        deliver(link::Reset{}, fsw, pf);
        CHECK(flies<Mahony>(fsw, pf, 3));
        deliver(link::SetPreset{4}, fsw, pf);
        deliver(link::Reset{}, fsw, pf);
        CHECK(flies<Complementary>(fsw, pf, 4));

        deliver(link::SetPreset{200}, fsw, pf);
        deliver(link::Reset{}, fsw, pf);
        CHECK(fsw.preset() == 0);
        CHECK(flies<Eskf>(fsw, pf, 0));

        deliver(link::SetPreset{1}, fsw, pf);
        deliver(link::Reboot{}, fsw, pf);
        CHECK(pf.reboots == 1);
        CHECK(fsw.preset() == 1);
        std::printf("test4 preset selection: done\n");
    }
    // 5. GNSS withheld for the whole run: no estimator is ever valid, however well it aligned.
    {
        synth::Options o;
        o.seconds = 10.0;
        Eskf a;
        Ekf b;
        Ukf c;
        Mahony d;
        Complementary e;
        const Result r[5] = {run(a, o, 1e9), run(b, o, 1e9), run(c, o, 1e9), run(d, o, 1e9), run(e, o, 1e9)};
        for (const Result& x : r) CHECK(!x.aligned);
        std::printf("test5 no GNSS: valid ever eskf %d ekf %d ukf %d mahony %d complementary %d\n", r[0].aligned,
                    r[1].aligned, r[2].aligned, r[3].aligned, r[4].aligned);
    }
    // 6. GNSS withheld until 5 s (3 s into the motion): invalid until the first fix, valid first on it. Error bounds
    // are graded with the first fix at rest (1.5 s): estimators do not re-reference p when the GNSS origin arrives
    // after the vehicle has moved (first-fix-in-motion); open, see lead's list.
    {
        const auto late = [&](auto& f, const char* name, double from, const Limits* l) {
            const Result r = run(f, ideal, from);
            print(name, r);
            CHECK(r.aligned);
            CHECK(r.fix_t >= from && r.align_t == r.fix_t);
            if (!l) return;
            CHECK(r.att_max_deg < l->att_deg);
            CHECK(r.vel_max < l->vel);
            CHECK(r.pos_max < l->pos);
        };
        const Limits kf{0.5, 0.05, 0.15}, mahony{12.0, 0.5, 0.4}, comp{5.5, 0.25, 0.2};
        const double froms[2] = {5.0, 1.5};
        for (int k = 0; k < 2; ++k) {
            const double from = froms[k];
            const bool graded = k == 1;
            std::printf("gnss from %.1f s\n", from);
            Eskf a;
            late(a, "eskf", from, graded ? &kf : nullptr);
            Ekf b;
            late(b, "ekf", from, graded ? &kf : nullptr);
            Ukf c;
            late(c, "ukf", from, graded ? &kf : nullptr);
            Mahony d;
            late(d, "mahony", from, graded ? &mahony : nullptr);
            Complementary e;
            late(e, "complementary", from, graded ? &comp : nullptr);
        }
    }
    // 7. Fsw on the estimate: with GNSS withheld the mission to fly never arms a motor; with GNSS it arms once the
    // estimator is valid.
    for (int gnss = 0; gnss <= 1; ++gnss) {
        Fsw fsw{0};
        fsw.on_mission({Mode::kFly, NavSource::kEstimate, {}});
        synth::Options o;
        o.seconds = 3.0;
        synth::Flight flight(o);
        SensorBus bus{};
        synth::Truth s{};
        bool armed_off = false, motors_off = true, armed_ever = false, armed_before_valid = false;
        while (flight.next(bus, s)) {
            if (!gnss) bus.fresh = static_cast<std::uint8_t>(bus.fresh & ~kGnss);
            const Tick t = fsw.step(bus);
            armed_ever = armed_ever || t.act.armed;
            armed_before_valid = armed_before_valid || (t.act.armed && !t.tlm.est.valid);
            if (!gnss) {
                armed_off = armed_off || t.act.armed;
                for (float m : t.act.motor) motors_off = motors_off && m == 0.f;
            }
        }
        std::printf("test7 fsw %s GNSS: armed ever %d\n", gnss ? "with" : "without", armed_ever);
        if (gnss) CHECK(armed_ever && !armed_before_valid);
        else CHECK(!armed_off && motors_off);
    }
    std::printf(failures ? "FAIL (%d)\n" : "PASS\n", failures);
    return failures ? EXIT_FAILURE : EXIT_SUCCESS;
}
