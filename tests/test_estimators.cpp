// The preset estimators (EKF, Mahony, complementary) against the analytic truth of synth.hpp, and preset
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
    double d_max = 0;      // vertical position error
    double fix_t = -1;     // the first GNSS fix delivered
    double core_align_t = -1;  // the estimator's own alignment (aligned()), which valid may follow
    bool dropped = false;  // valid went true -> false
};

double err3(Vec3 a, const double b[3]) {
    const double dx = a.x - b[0], dy = a.y - b[1], dz = a.z - b[2];
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// GNSS is withheld before gnss_from (s). Errors are graded from grade_after_fix (s) after the first fix delivered;
// horizontal position about the truth at that fix (the estimator's origin), vertical about the start (the barometer's
// reference, taken at alignment). Every fix after the first reads gnss_alt_bias (m) high.
template <class E>
Result run(E& f, const synth::Options& o, double gnss_from = 0.0, double grade_after_fix = 0.0, float gnss_alt_bias = 0.f) {
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
            for (int i = 0; i < 2; ++i) origin[i] = s.p[i];
        } else if (bus.fresh & kGnss) {
            bus.gnss.alt_m += gnss_alt_bias;
        }
        const auto t0 = std::chrono::steady_clock::now();
        f.update(bus);
        spent += std::chrono::steady_clock::now() - t0;
        ++n;
        if (f.aligned() && r.core_align_t < 0) r.core_align_t = s.t;
        const State e = f.state();
        if (was_valid && !e.valid) r.dropped = true;
        was_valid = e.valid;
        if (!e.valid) continue;
        if (!r.aligned) {
            r.aligned = true;
            r.align_t = s.t;
        }
        if (s.t < r.fix_t + grade_after_fix) continue;
        const double att = quat_angle(e.q, s.q()) / synth::kDeg;
        r.att_max_deg = std::fmax(r.att_max_deg, att);
        if (s.t >= synth::kRest + 30.0) r.att_max_after30_deg = std::fmax(r.att_max_after30_deg, att);
        r.vel_max = std::fmax(r.vel_max, err3(e.v_ned, s.v));
        const double p[3] = {s.p[0] - origin[0], s.p[1] - origin[1], s.p[2] - origin[2]};
        r.pos_max = std::fmax(r.pos_max, err3(e.p_ned, p));
        r.d_max = std::fmax(r.d_max, std::fabs(e.p_ned.z - s.p[2]));
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
    std::vector<std::uint8_t> record;
    int reboots = 0;
    void send(const std::uint8_t* p, std::size_t n) { out.insert(out.end(), p, p + n); }
    std::size_t read_record(std::uint8_t* p, std::size_t cap) {
        const std::size_t n = record.size() < cap ? record.size() : cap;
        std::memcpy(p, record.data(), n);
        return n;
    }
    void write_record(const std::uint8_t* p, std::size_t n) { record.assign(p, p + n); }
    void reboot() { ++reboots; }
};
using Node = fw::Node<TestPlatform>;

template <class T> void deliver(const T& msg, Node& node) {
    std::uint8_t frame[link::kMaxFrame];
    const std::size_t n = link::encode(msg, frame);
    link::Decoder dec;
    for (std::size_t i = 0; i < n; ++i)
        if (dec.push(frame[i])) node.dispatch(dec.packet());
}

// The estimator kind of the stored record, 0xFF when it is not valid.
std::uint8_t stored_estimator(const TestPlatform& pf) {
    param::Setup s;
    return fw::decode_record(pf.record.data(), pf.record.size(), s) ? s.kind[param::k_estimator] : 0xFF;
}

// Runs 2.5 s of the synthetic flight (0.5 s of it moving: at rest, estimators can agree to the bit) through dispatch
// and a reference estimator side by side. Returns true when every Telemetry carried `preset` and an estimate
// bit-identical to the reference's.
template <class E> bool flies(Node& node, TestPlatform& pf, std::uint8_t preset) {
    E ref;
    synth::Options o;
    o.seconds = 2.5;
    synth::Flight flight(o);
    SensorBus bus{};
    synth::Truth s{};
    link::Decoder dec;
    bool ok = true, valid = false;
    while (flight.next(bus, s)) {
        pf.out.clear();
        deliver(bus, node);
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
        Mahony m;
        const Result rm = run(m, o);
        print("mahony biased", rm);
        CHECK(rm.att_max_after30_deg < 12.0);
        std::printf("gyro bias ekf (%.5f %.5f %.5f) mahony (%.5f %.5f %.5f)\n", e.gyro_bias().x, e.gyro_bias().y,
                    e.gyro_bias().z, m.gyro_bias().x, m.gyro_bias().y, m.gyro_bias().z);
    }
    // 3. Negative control: the magnetometer synthesised with the wrong rotation must fail test 1's EKF limits.
    {
        synth::Options o;
        o.flip_mag = true;
        Ekf f;
        const Result r = run(f, o);
        print("ekf negative control", r);
        CHECK(r.att_max_deg >= 0.5);
    }
    // 4. Preset selection: SetPreset stages and stores the factory setup, Reset applies it, an unknown id runs factory 0,
    // Reboot restarts.
    {
        TestPlatform pf;
        Node node{pf};
        CHECK(node.fsw().preset() == 0);
        deliver(link::SetPreset{2}, node);
        CHECK(stored_estimator(pf) == 2);
        CHECK(node.fsw().preset() == 0);  // stored, not yet applied
        deliver(link::Reset{}, node);
        CHECK(node.fsw().preset() == 2);
        CHECK(flies<Mahony>(node, pf, 2));
        deliver(link::Reset{}, node);
        CHECK(!flies<Eskf>(node, pf, 2));  // the check can fail: the same flight on another estimator differs

        const std::uint8_t ids[] = {0, 1, 3};
        for (std::uint8_t id : ids) {
            deliver(link::SetPreset{id}, node);
            deliver(link::Reset{}, node);
            CHECK(node.fsw().preset() == id);
        }
        deliver(link::SetPreset{0}, node);
        deliver(link::Reset{}, node);
        CHECK(flies<Eskf>(node, pf, 0));
        deliver(link::SetPreset{1}, node);
        deliver(link::Reset{}, node);
        CHECK(flies<Ekf>(node, pf, 1));
        deliver(link::SetPreset{2}, node);
        deliver(link::Reset{}, node);
        CHECK(flies<Mahony>(node, pf, 2));
        deliver(link::SetPreset{3}, node);
        deliver(link::Reset{}, node);
        CHECK(flies<Complementary>(node, pf, 3));

        deliver(link::SetPreset{200}, node);
        deliver(link::Reset{}, node);
        CHECK(node.fsw().preset() == 0);
        CHECK(flies<Eskf>(node, pf, 0));

        deliver(link::SetPreset{1}, node);
        deliver(link::Reboot{}, node);
        CHECK(pf.reboots == 1);
        CHECK(node.fsw().preset() == 1);
        std::printf("test4 preset selection: done\n");
    }
    // 5. GNSS withheld for the whole run: no estimator is ever valid, however well it aligned.
    {
        synth::Options o;
        o.seconds = 10.0;
        Eskf a;
        Ekf b;
        Mahony d;
        Complementary e;
        const Result r[4] = {run(a, o, 1e9), run(b, o, 1e9), run(d, o, 1e9), run(e, o, 1e9)};
        for (const Result& x : r) CHECK(!x.aligned);
        std::printf("test5 no GNSS: valid ever eskf %d ekf %d mahony %d complementary %d\n", r[0].aligned, r[1].aligned,
                    r[2].aligned, r[3].aligned);
    }
    // 6. GNSS withheld until 5 s: invalid until the first fix, valid first on it.
    // (a) At rest until 6 s, the field from the WMM lookup (the default): the still window is done long before, but
    // the field is unknown until the fix, so every estimator aligns on the fix tick; test 1's bounds from there.
    // (b) The setup's field overrides the lookup (non-zero mag_ref_ned_ut, and the synthesis the same field: -21.8 deg
    // of declination against the WMM's 3.4 at the origin): aligned within the first rest second, valid first on the
    // fix 3 s into the motion, test 1's bounds from 1 s after it (horizontal position re-referenced to that fix).
    // (c) Negative control: that field synthesised, the estimator left on the lookup, must fail test 1's EKF bound.
    {
        const auto late = [&](auto& f, const char* name, const synth::Options& o, double grade_after_fix, bool on_fix,
                              const Limits& l) {
            const Result r = run(f, o, 5.0, grade_after_fix);
            print(name, r);
            CHECK(r.aligned);
            CHECK(r.fix_t >= 5.0 && r.align_t == r.fix_t);
            CHECK(on_fix ? r.core_align_t == r.fix_t : r.core_align_t < 1.2);
            CHECK(r.att_max_deg < l.att_deg);
            CHECK(r.vel_max < l.vel);
            CHECK(r.pos_max < l.pos);
        };
        const Limits kf{0.5, 0.05, 0.15}, mahony{12.0, 0.5, 0.4}, comp{5.5, 0.25, 0.2};
        synth::Options still = ideal;
        still.rest = 6.0;
        std::printf("gnss from 5.0 s, at rest until 6 s, WMM lookup\n");
        Eskf a;
        late(a, "eskf", still, 0.0, true, kf);
        Ekf b;
        late(b, "ekf", still, 0.0, true, kf);
        Mahony d;
        late(d, "mahony", still, 0.0, true, mahony);
        Complementary e;
        late(e, "complementary", still, 0.0, true, comp);

        synth::Options over = ideal;
        over.mag_ned[0] = 20.0, over.mag_ned[1] = -8.0, over.mag_ned[2] = 43.0;
        param::SensorParams sp;
        sp.mag_ref_ned_ut_x = 20.f, sp.mag_ref_ned_ut_y = -8.f, sp.mag_ref_ned_ut_z = 43.f;
        std::printf("gnss from 5.0 s, moving from 2 s, field override\n");
        Eskf a2{{}, sp};
        late(a2, "eskf", over, 1.0, false, kf);
        Ekf b2{{}, sp};
        late(b2, "ekf", over, 1.0, false, kf);
        Mahony d2{{}, sp};
        late(d2, "mahony", over, 1.0, false, mahony);
        Complementary e2{{}, sp};
        late(e2, "complementary", over, 1.0, false, comp);

        Ekf neg;
        const Result rn = run(neg, over);
        print("ekf override negative", rn);
        CHECK(rn.att_max_deg >= 0.5);
    }
    // 7. Fsw on the estimate: with GNSS withheld the mission to fly never arms a motor; with GNSS it arms once the
    // estimator is valid.
    for (int gnss = 0; gnss <= 1; ++gnss) {
        Fsw fsw{kFactory[0]};
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
    // 8. Every fix after the first reads 1.5 m high (the origin fix's own altitude error): the complementary filters'
    // height is the barometer's, so their vertical error stays within 0.1 m.
    {
        Mahony d;
        const Result rd = run(d, ideal, 0.0, 0.0, 1.5f);
        Complementary e;
        const Result re = run(e, ideal, 0.0, 0.0, 1.5f);
        std::printf("test8 gnss alt +1.5 m: d max mahony %.4f m complementary %.4f m\n", rd.d_max, re.d_max);
        CHECK(rd.d_max < 0.1);
        CHECK(re.d_max < 0.1);
    }
    std::printf(failures ? "FAIL (%d)\n" : "PASS\n", failures);
    return failures ? EXIT_FAILURE : EXIT_SUCCESS;
}
