// The rocket chain (apogee predictor -> apogee PID -> rocket brake) against the avionics toolbox's rocket-airbrakes run
// in tests/fixtures/rocket-airbrakes.json (scripts/rocket-fixture.mjs: the preset with the apogee-pid controller), at
// the toolbox's 20 ms frames:
//  1. on the toolbox ESKF's navigation state: our apogee prediction within 1e-3 relative of guid/apogee-pred, our brake
//     within 1e-3 of fcs/brake-cmd; negative control: a 2 % drag-coefficient error in the model fails that bound.
//  2. the chain at 1 ms (nav interpolated between frames) against the 20 ms chain held: under 1 % of full deployment.
//  3. the ESKF in shadow cannot run on the fixture's sensors (the preset has no magnetometer, which our alignment needs):
//     1 and 2 again on the truth, against the toolbox's predictApogee and apogee-pid run on the truth.
//  4. invariants: a rocket setup commands every motor zero on every tick; a uav setup commands the brake zero on every
//     tick; a rollout takes at most 400 steps. And factory 4 brakes only while the mission flags the coast. A flight
//     profile, the manual flag and sticks change nothing on a rocket, which reports the hold profile (ADR-0012).
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <marv/fsw/fsw.hpp>
#include <marv/fsw/guidance.hpp>
#include <marv/fsw/presets.hpp>

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

// The fixture: its text (for the scalar fields) and the frame table.
struct Fixture {
    std::string text;
    std::vector<std::string> cols;
    std::vector<std::vector<double>> rows;

    int col(const char* name) const {
        for (std::size_t i = 0; i < cols.size(); ++i)
            if (cols[i] == name) return static_cast<int>(i);
        std::printf("FAIL no column %s\n", name);
        std::exit(EXIT_FAILURE);
    }
    double at(std::size_t k, const char* name) const { return rows[k][static_cast<std::size_t>(col(name))]; }
    // The first number after "key":
    double num(const char* key) const {
        const std::string k = std::string("\"") + key + "\":";
        const std::size_t p = text.find(k);
        if (p == std::string::npos) {
            std::printf("FAIL no key %s\n", key);
            std::exit(EXIT_FAILURE);
        }
        return std::strtod(text.c_str() + p + k.size(), nullptr);
    }
};

Fixture load() {
    std::string dir = __FILE__;
    dir.resize(dir.find_last_of('/') + 1);
    std::ifstream f(dir + "fixtures/rocket-airbrakes.json");
    if (!f) {
        std::printf("FAIL cannot open %sfixtures/rocket-airbrakes.json\n", dir.c_str());
        std::exit(EXIT_FAILURE);
    }
    std::stringstream ss;
    ss << f.rdbuf();
    Fixture fx;
    fx.text = ss.str();
    const char* s = fx.text.c_str();
    const char* c = std::strchr(std::strstr(s, "\"columns\":"), '[') + 1;
    for (const char* e = std::strchr(c, ']'); (c = std::strchr(c, '"')) && c < e;) {
        const char* q = std::strchr(c + 1, '"');
        fx.cols.emplace_back(c + 1, q);
        c = q + 1;
    }
    const char* r = std::strchr(std::strstr(s, "\"frames\":"), '[') + 1;
    while ((r = std::strchr(r, '[')) != nullptr) {
        std::vector<double> row;
        ++r;
        while (*r != ']') {
            while (*r == ' ' || *r == ',' || *r == '\n') ++r;
            if (std::strncmp(r, "null", 4) == 0) {
                row.push_back(NAN);
                r += 4;
            } else {
                char* end;
                row.push_back(std::strtod(r, &end));
                r = end;
            }
        }
        if (row.size() != fx.cols.size()) {
            std::printf("FAIL row %zu has %zu fields\n", fx.rows.size(), row.size());
            std::exit(EXIT_FAILURE);
        }
        fx.rows.push_back(std::move(row));
    }
    return fx;
}

// Factory 4 with the toolbox's model, gravity and gains, as the fixture records them.
param::Setup toolbox_setup(const Fixture& fx) {
    param::Setup s = kFactory[4];
    s.values[param::k_sensors_suite_gravity] = static_cast<float>(fx.num("g"));
    s.values[param::k_guidance_apogee_predictor_predict_dt] = static_cast<float>(fx.num("predictDt"));
    s.values[param::k_guidance_apogee_predictor_cd_base] = static_cast<float>(fx.num("cdBase"));
    s.values[param::k_guidance_apogee_predictor_ref_area] = static_cast<float>(fx.num("refArea"));
    s.values[param::k_guidance_apogee_predictor_mass] = static_cast<float>(fx.num("mass"));
    s.values[param::k_guidance_apogee_predictor_target_apogee_m] = static_cast<float>(fx.num("targetApogee"));
    s.values[param::k_controller_apogee_pid_kp] = static_cast<float>(fx.num("kp"));
    s.values[param::k_controller_apogee_pid_ki] = static_cast<float>(fx.num("ki"));
    s.values[param::k_controller_apogee_pid_i_limit] = static_cast<float>(fx.num("iLimit"));
    return s;
}

// Navigation state of frame k from the columns <prefix>p_n.. (the toolbox's nav: "nav_", the truth: "").
State nav_of(const Fixture& fx, std::size_t k, const std::string& pre) {
    auto v = [&](const char* n) { return static_cast<float>(fx.at(k, (pre + n).c_str())); };
    State s{};
    s.p_ned = {v("p_n"), v("p_e"), v("p_d")};
    s.v_ned = {v("v_n"), v("v_e"), v("v_d")};
    s.q = {v("q_w"), v("q_x"), v("q_y"), v("q_z")};
    s.w_frd = {v("w_x"), v("w_y"), v("w_z")};
    s.valid = true;
    return s;
}

std::uint64_t us_of(double t) { return static_cast<std::uint64_t>(std::llround(t * 1e6)); }

MissionCommand mission_of(const Fixture& fx, std::size_t k, bool coast_bit = true) {
    MissionCommand m{fx.at(k, "fly") > 0.5 ? Mode::kFly : Mode::kIdle, NavSource::kTruth, {}};
    if (coast_bit && fx.at(k, "coast") > 0.5) m.ref.has = kRefCoast;
    return m;
}

bool motors_zero(const ActuatorCommand& a) {
    for (float m : a.motor)
        if (m != 0.f) return false;
    return true;
}

// The Fsw on the fixture's frames at 20 ms, on the navigation columns given: the brake of every frame; every motor zero
// on every tick counted in *motor_ticks.
std::vector<float> run_20ms(const Fixture& fx, const param::Setup& s, const std::string& pre, int* motor_ticks,
                            bool coast_bit = true) {
    Fsw fsw{s};
    std::vector<float> b;
    for (std::size_t k = 0; k < fx.rows.size(); ++k) {
        const std::uint64_t t = us_of(fx.at(k, "t"));
        State nav = nav_of(fx, k, pre);
        nav.t_us = t;
        fsw.on_mission(mission_of(fx, k, coast_bit));
        fsw.on_truth(nav);
        SensorBus bus{};
        bus.t_us = t;
        const Tick tk = fsw.step(bus);
        if (!motors_zero(tk.act)) ++*motor_ticks;
        CHECK(tk.act.brake == tk.tlm.req.brake);
        b.push_back(tk.act.brake);
    }
    return b;
}

// The same at 1 ms: position and velocity interpolated between frames, attitude, rate and the mission of the frame
// before. The largest |brake - the 20 ms brake of the frame before|.
float run_1ms_gap(const Fixture& fx, const param::Setup& s, const std::string& pre, const std::vector<float>& b20,
                  int* motor_ticks) {
    Fsw fsw{s};
    float gap = 0.f;
    const std::uint64_t t_end = us_of(fx.at(fx.rows.size() - 1, "t"));
    for (std::uint64_t t = 0; t <= t_end; t += 1000) {
        const std::size_t k = static_cast<std::size_t>(t / 20000);
        const std::size_t k1 = k + 1 < fx.rows.size() ? k + 1 : k;
        const float a = static_cast<float>(t - k * 20000) / 20000.f;
        const State s0 = nav_of(fx, k, pre), s1 = nav_of(fx, k1, pre);
        State nav = s0;
        nav.p_ned = {s0.p_ned.x + a * (s1.p_ned.x - s0.p_ned.x), s0.p_ned.y + a * (s1.p_ned.y - s0.p_ned.y),
                     s0.p_ned.z + a * (s1.p_ned.z - s0.p_ned.z)};
        nav.v_ned = {s0.v_ned.x + a * (s1.v_ned.x - s0.v_ned.x), s0.v_ned.y + a * (s1.v_ned.y - s0.v_ned.y),
                     s0.v_ned.z + a * (s1.v_ned.z - s0.v_ned.z)};
        nav.t_us = t;
        fsw.on_mission(mission_of(fx, k));
        fsw.on_truth(nav);
        SensorBus bus{};
        bus.t_us = t;
        const Tick tk = fsw.step(bus);
        if (!motors_zero(tk.act)) ++*motor_ticks;
        gap = std::fmax(gap, std::fabs(tk.act.brake - b20[k]));
    }
    return gap;
}

struct Match {
    float apogee_rel = 0.f;  // largest |ours - toolbox| / toolbox, apogee prediction
    float brake = 0.f;       // largest |ours - toolbox|, brake
    int coast = 0;           // frames compared
    int steps = 0;           // most rollout steps
};

// Test 1 on the navigation columns given, against the expected columns given.
Match match(const Fixture& fx, const param::Setup& s, const std::string& pre, const char* apogee_col,
            const char* brake_col, int* motor_ticks) {
    Match m;
    ApogeePredictor pred{param::guidance_apogee_predictor(s), param::sensors_suite(s)};
    for (std::size_t k = 0; k < fx.rows.size(); ++k) {
        if (fx.at(k, "coast") < 0.5 || fx.at(k, "fly") < 0.5) continue;
        const State n = nav_of(fx, k, pre);
        const float a = pred.apogee(-n.p_ned.z, -n.v_ned.z, std::sqrt(n.v_ned.x * n.v_ned.x + n.v_ned.y * n.v_ned.y));
        const double ref = fx.at(k, apogee_col);
        m.apogee_rel = std::fmax(m.apogee_rel, static_cast<float>(std::fabs(a - ref) / std::fabs(ref)));
        m.steps = pred.steps() > m.steps ? pred.steps() : m.steps;
        ++m.coast;
    }
    const std::vector<float> b = run_20ms(fx, s, pre, motor_ticks);
    for (std::size_t k = 0; k < fx.rows.size(); ++k)
        m.brake = std::fmax(m.brake, static_cast<float>(std::fabs(b[k] - fx.at(k, brake_col))));
    return m;
}

}  // namespace

int main() {
    const Fixture fx = load();
    std::printf("fixture: %zu frames, toolbox %s\n", fx.rows.size(),
                fx.text.substr(fx.text.find("\"toolbox_commit\": \"") + 19, 7).c_str());
    CHECK(fx.rows.size() > 300);
    const param::Setup setup = toolbox_setup(fx);
    CHECK(param::consistent(setup));
    int motor_ticks = 0;  // rocket ticks with a motor command other than zero, every run

    // 1. On the toolbox's navigation state.
    {
        const Match m = match(fx, setup, "nav_", "apogee_pred", "brake_cmd", &motor_ticks);
        std::printf("1. toolbox nav: %d coast frames, apogee rel err max %.3g, brake err max %.3g, steps max %d\n",
                    m.coast, m.apogee_rel, m.brake, m.steps);
        CHECK(m.coast > 100);
        CHECK(m.apogee_rel <= 1e-3f);
        CHECK(m.brake <= 1e-3f);

        // The brake did work in the fixture: the comparison is not of two zeros.
        float peak = 0.f;
        for (std::size_t k = 0; k < fx.rows.size(); ++k) peak = std::fmax(peak, static_cast<float>(fx.at(k, "brake_cmd")));
        CHECK(peak > 0.5f);

        // Negative control: the drag coefficient of the model 2 % high fails the bound.
        param::Setup bad = setup;
        bad.values[param::k_guidance_apogee_predictor_cd_base] *= 1.02f;
        int ignored = 0;
        const Match n = match(fx, bad, "nav_", "apogee_pred", "brake_cmd", &ignored);
        std::printf("1. negative control (cd_base +2 %%): apogee rel err max %.3g, brake err max %.3g\n", n.apogee_rel,
                    n.brake);
        CHECK(n.brake > 1e-3f);
    }

    // 2. 1 ms against 20 ms, on the toolbox's navigation state.
    {
        const std::vector<float> b20 = run_20ms(fx, setup, "nav_", &motor_ticks);
        const float gap = run_1ms_gap(fx, setup, "nav_", b20, &motor_ticks);
        std::printf("2. 1 ms vs 20 ms, toolbox nav: brake gap max %.3g\n", gap);
        CHECK(gap < 0.01f);
    }

    // 3. Fallback: 1 and 2 on the truth.
    {
        const Match m = match(fx, setup, "", "apogee_pred_truth", "brake_cmd_truth", &motor_ticks);
        std::printf("3. truth nav: %d coast frames, apogee rel err max %.3g, brake err max %.3g, steps max %d\n",
                    m.coast, m.apogee_rel, m.brake, m.steps);
        CHECK(m.coast > 100);
        CHECK(m.apogee_rel <= 1e-3f);
        CHECK(m.brake <= 1e-3f);
        const std::vector<float> b20 = run_20ms(fx, setup, "", &motor_ticks);
        const float gap = run_1ms_gap(fx, setup, "", b20, &motor_ticks);
        std::printf("3. 1 ms vs 20 ms, truth nav: brake gap max %.3g\n", gap);
        CHECK(gap < 0.01f);
    }

    // 4. Invariants.
    {
        CHECK(motor_ticks == 0);

        // A uav setup never moves the brake, even with the coast flagged; it did fly (the control).
        Fsw uav{kFactory[0]};
        bool armed = false, brake_zero = true;
        for (std::size_t k = 0; k < fx.rows.size(); ++k) {
            const std::uint64_t t = us_of(fx.at(k, "t"));
            State nav = nav_of(fx, k, "");
            nav.t_us = t;
            MissionCommand m{Mode::kFly, NavSource::kTruth, {}};
            m.ref.has = kRefCoast | kRefApogee;
            m.ref.apogee_m = 100.f;
            m.ref.apogee_pred_m = 300.f;
            uav.on_mission(m);
            uav.on_truth(nav);
            SensorBus bus{};
            bus.t_us = t;
            const Tick tk = uav.step(bus);
            armed = armed || tk.act.armed;
            brake_zero = brake_zero && tk.act.brake == 0.f && tk.tlm.req.brake == 0.f;
        }
        CHECK(armed && brake_zero);

        // kArmed: armed (the save lockout holds), the brake closed and every motor zero, even with a demand.
        {
            ControlRequest req{};
            req.brake = 0.8f;
            const ActuatorCommand c = RocketBrake{}.run(req, Mode::kArmed);
            CHECK(c.armed && c.brake == 0.f);
            for (float m : c.motor) CHECK(m == 0.f);
            CHECK(RocketBrake{}.run(req, Mode::kFly).brake == 0.8f && !RocketBrake{}.run(req, Mode::kIdle).armed);
        }

        // The rollout bound: the finest step from the ground at 1 km/s stops at 400 steps.
        param::Setup fine = setup;
        fine.values[param::k_guidance_apogee_predictor_predict_dt] = 0.02f;
        ApogeePredictor pred{param::guidance_apogee_predictor(fine), param::sensors_suite(fine)};
        const float a = pred.apogee(0.f, 1000.f, 0.f);
        std::printf("4. worst case: %d steps (apogee %.1f m at the cap)\n", pred.steps(), a);
        CHECK(pred.steps() == ApogeePredictor::kMaxSteps);

        // Host cost of that worst-case rollout (reported, not checked).
        const int n = 20000;
        float sink = 0.f;
        const auto t0 = std::chrono::steady_clock::now();
        for (int i = 0; i < n; ++i) sink += pred.apogee(0.f, 1000.f + 1e-3f * static_cast<float>(i & 7), 0.f);
        const auto t1 = std::chrono::steady_clock::now();
        std::printf("4. host: %.2f us per 400-step rollout (%g)\n",
                    std::chrono::duration<double, std::micro>(t1 - t0).count() / n, static_cast<double>(sink) * 0.0);
    }

    // Factory 4 brakes only while the mission flags the coast; without the flag, never.
    {
        int ignored = 0;
        const std::vector<float> b = run_20ms(fx, kFactory[4], "nav_", &ignored);
        int braking = 0, outside = 0;
        for (std::size_t k = 0; k < fx.rows.size(); ++k) {
            if (b[k] == 0.f) continue;
            if (fx.at(k, "coast") > 0.5) ++braking;
            else ++outside;
        }
        std::printf("factory 4: %d coast frames braking, %d outside the coast\n", braking, outside);
        CHECK(braking > 50 && outside == 0);
        const std::vector<float> nb = run_20ms(fx, kFactory[4], "nav_", &ignored, false);
        bool none = true;
        for (float x : nb) none = none && x == 0.f;
        CHECK(none);
        CHECK(Fsw{kFactory[4]}.preset() == 4);

        // Profile agile, manual 1 and full sticks: the brake bit for bit the plain run's, profile 0 reported.
        Fsw fsw{kFactory[4]};
        bool same = true, hold = true;
        for (std::size_t k = 0; k < fx.rows.size(); ++k) {
            const std::uint64_t t = us_of(fx.at(k, "t"));
            State nav = nav_of(fx, k, "nav_");
            nav.t_us = t;
            MissionCommand m = mission_of(fx, k, true);
            m.profile = param::k_profile_agile;
            m.manual = 1;
            m.sticks = {1.f, 1.f, 1.f, 1.f};
            fsw.on_mission(m);
            fsw.on_truth(nav);
            SensorBus bus{};
            bus.t_us = t;
            const Tick tk = fsw.step(bus);
            same = same && tk.act.brake == b[k] && motors_zero(tk.act);
            hold = hold && tk.tlm.profile == param::k_profile_hold;
        }
        std::printf("factory 4 with profile agile, manual 1 and full sticks: brake as without %d, profile 0 reported %d\n",
                    same, hold);
        CHECK(same && hold);
    }

    std::printf(failures ? "FAIL (%d)\n" : "PASS\n", failures);
    return failures ? EXIT_FAILURE : EXIT_SUCCESS;
}
