// The pilot's configurable mapping (ground/src/radio_config.hpp, radio_json.hpp, pilot.hpp): stick calibration
// (min/center/max, reverse, deadband), profile bands (edges, 2..6 bands) and buttons, the arming interlock, the built-in
// configs flying exactly as marv_ground's phase-E mapping, the JSON round trip and its refusals, the device slug, and
// the stored file.
#define MARV_PARAMS_TEXT
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <functional>
#include <random>
#include <string>

#include <boost/json/src.hpp>

#include "../ground/src/pilot.hpp"
#include "../ground/src/radio_json.hpp"

using namespace marv;
using namespace marv::ground;
namespace json = boost::json;

static int failures = 0;
#define CHECK(c)                                                                    \
    do {                                                                            \
        if (!(c)) {                                                                 \
            std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c);                \
            ++failures;                                                             \
        }                                                                           \
    } while (0)
#define NEAR(a, b, tol) CHECK(std::fabs((a) - (b)) <= (tol))

constexpr std::uint8_t kButton = 1, kAxis = 2;

// The phase-E Pilot (ground/src/pilot.hpp @ 959d3e6) verbatim but for the names: the reference the built-in configs
// must reproduce.
class PhaseE {
public:
    explicit PhaseE(bool radio) : radio_(radio) {}
    static float stick(std::int16_t raw) {
        constexpr float kBand = 0.1f;
        const float v = static_cast<float>(raw) / 32767.f;
        const float a = std::fabs(v);
        if (a <= kBand) return 0.f;
        return std::copysign(std::fmin((a - kBand) / (1.f - kBand), 1.f), v);
    }
    static std::uint8_t ch6_profile(std::int16_t raw) {
        if (raw < -16384) return param::k_profile_hold;
        if (raw < 0) return param::k_profile_freestyle;
        if (raw < 16384) return param::k_profile_stabilized;
        return param::k_profile_agile;
    }
    void event(std::uint8_t type, std::uint8_t number, std::int16_t value) {
        if (type == kAxis && number < 8) {
            axis_[number] = value;
            seen_[number] = true;
        } else if (!radio_ && type == kButton && value) {
            switch (number) {
                case 0: fly_ = true; break;
                case 1: fly_ = false; break;
                case 2: profile_ = param::k_profile_hold; break;
                case 3: profile_ = param::k_profile_stabilized; break;
                case 4: profile_ = param::k_profile_freestyle; break;
                case 5: profile_ = param::k_profile_agile; break;
                default: break;
            }
        }
    }
    void update() {
        sticks_.up = radio_ ? stick(axis_[2]) : 0.f - stick(axis_[1]);
        sticks_.yaw = radio_ ? stick(axis_[3]) : stick(axis_[0]);
        sticks_.fwd = radio_ ? stick(axis_[1]) : 0.f - stick(axis_[4]);
        sticks_.right = radio_ ? stick(axis_[0]) : stick(axis_[3]);
        if (!radio_) return;
        if (seen_[4]) {
            const int sw = axis_[4] > 0 ? 1 : 0;
            if (!sw) fly_ = refused_ = false;
            else if (prev_switch_ == 0 && sticks_.up <= -0.95f) fly_ = true;
            else if (prev_switch_ == 0) refused_ = true;
            prev_switch_ = sw;
        }
        if (seen_[5]) profile_ = ch6_profile(axis_[5]);
    }
    Sticks sticks_{0.f, 0.f, 0.f, 0.f};
    bool fly_ = false, refused_ = false;
    std::uint8_t profile_ = param::k_profile_hold;

private:
    bool radio_;
    std::int16_t axis_[8] = {};
    bool seen_[8] = {};
    int prev_switch_ = -1;
};

// The phase-E mapping against the same device through its built-in config, event for event.
static void check_default_matches(bool radio) {
    PhaseE old(radio);
    Pilot cfg(radio ? default_config("EdgeTX Radiomaster Pocket Joystick") : default_config("Microsoft X-Box One S pad"));
    std::mt19937 rng(radio ? 1u : 2u);
    const std::int16_t levels[] = {-32768, -32767, -16385, -16384, -3277, -3276, 0, 3276, 3277, 16383, 16384, 32767};
    for (int period = 0; period < 5000; ++period) {
        for (int k = static_cast<int>(rng() % 4); k > 0; --k) {
            const std::uint8_t type = rng() % 3 ? kAxis : kButton;
            const std::uint8_t n = static_cast<std::uint8_t>(rng() % 8);
            const std::int16_t v = type == kButton ? static_cast<std::int16_t>(rng() % 2)
                                   : rng() % 2 ? levels[rng() % 12]
                                               : static_cast<std::int16_t>(static_cast<int>(rng() % 65536) - 32768);
            old.event(type, n, v);
            cfg.event(type, n, v);
            CHECK(old.profile_ == cfg.profile() && old.fly_ == cfg.fly());
        }
        old.update();
        cfg.update();
        const Sticks& s = cfg.sticks();
        CHECK(old.sticks_.up == s.up && old.sticks_.yaw == s.yaw && old.sticks_.fwd == s.fwd && old.sticks_.right == s.right);
        CHECK(old.profile_ == cfg.profile() && old.fly_ == cfg.fly());
        CHECK(old.refused_ == cfg.refused() || !radio);
        if (failures) return;
    }
}

static std::string refusal(const std::function<void(json::object&)>& edit, bool radio = true) {
    json::value v = to_json(radio ? radiomaster_config("R") : xbox_config("X"));
    edit(v.as_object());
    RadioConfig c;
    return from_json(v, c);
}

int main() {
    // Normalisation: the default axis is phase E's stick() at every raw value.
    for (int raw = -32768; raw <= 32767; ++raw) {
        const auto r = static_cast<std::int16_t>(raw);
        const float v = static_cast<float>(r) / 32767.f, a = std::fabs(v);
        const float old = a <= 0.1f ? 0.f : std::copysign(std::fmin((a - 0.1f) / 0.9f, 1.f), v);
        if (normalize(StickAxis{}, r) != old) {
            CHECK(normalize(StickAxis{}, r) == old);
            break;
        }
    }
    {
        StickAxis s{0, -30000, 1000, 31000, false, 0.f};
        NEAR(normalize(s, 1000), 0.f, 0.f);
        NEAR(normalize(s, 31000), 1.f, 0.f);
        NEAR(normalize(s, 32767), 1.f, 0.f);  // beyond max: clamped
        NEAR(normalize(s, -30000), -1.f, 0.f);
        NEAR(normalize(s, -32768), -1.f, 0.f);
        NEAR(normalize(s, 16000), 0.5f, 1e-6f);   // halfway center..max
        NEAR(normalize(s, -14500), -0.5f, 1e-6f);  // halfway min..center
        s.reverse = true;
        NEAR(normalize(s, 16000), -0.5f, 1e-6f);
        NEAR(normalize(s, -30000), 1.f, 0.f);
        s.reverse = false;
        s.deadband = 0.2f;
        NEAR(normalize(s, 7000), 0.f, 0.f);  // 0.2: on the band's edge
        NEAR(normalize(s, 19000), 0.5f, 1e-6f);  // 0.6 -> (0.6 - 0.2) / 0.8
        NEAR(normalize(s, -30000), -1.f, 0.f);
    }

    // Bands: the default is CH6 at every raw value; two and six bands, edges belong to the band above.
    {
        const RadioConfig r = radiomaster_config("R");
        for (int raw = -32768; raw <= 32767; ++raw)
            if (band_profile(r.profile, static_cast<std::int16_t>(raw)) != PhaseE::ch6_profile(static_cast<std::int16_t>(raw))) {
                CHECK(false);
                break;
            }
        RadioConfig::Profile p = r.profile;
        p.band_count = 2;
        p.bands[0] = {0.f, param::k_profile_hold};
        p.bands[1] = {1.f, param::k_profile_agile};
        CHECK(band_profile(p, -32768) == param::k_profile_hold);
        CHECK(band_profile(p, -1) == param::k_profile_hold);
        CHECK(band_profile(p, 0) == param::k_profile_agile);
        CHECK(band_profile(p, 32767) == param::k_profile_agile);
        p.band_count = 6;
        const float edges[6] = {-0.75f, -0.5f, -0.25f, 0.25f, 0.5f, 1.f};
        const std::uint8_t ids[6] = {0, 1, 2, 3, 0, 1};
        for (int i = 0; i < 6; ++i) p.bands[i] = {edges[i], ids[i]};
        for (int raw = -32768; raw <= 32767; ++raw) {  // the first band whose upper is above raw / 32767 (clamped)
            const double v = std::fmax(-1.0, raw / 32767.0);
            int want = 5;
            while (want > 0 && v < edges[want - 1]) --want;
            if (band_profile(p, static_cast<std::int16_t>(raw)) != ids[want]) {
                CHECK(band_profile(p, static_cast<std::int16_t>(raw)) == ids[want]);
                break;
            }
        }
        CHECK(band_profile(p, -8192) == ids[2] && band_profile(p, -8191) == ids[3]);  // -0.25: -8191.75
        CHECK(band_profile(p, 16383) == ids[4] && band_profile(p, 16384) == ids[5]);  // 0.5: 16383.5
    }

    // The built-in configs fly exactly as phase E did (tests/test_ground_pilot.cpp is that behaviour).
    check_default_matches(true);
    check_default_matches(false);
    CHECK(default_config("EdgeTX Radiomaster Pocket Joystick").arm.source == ArmSource::kAxis);
    CHECK(default_config("Microsoft X-Box One S pad").arm.source == ArmSource::kButton);
    CHECK(default_config("Microsoft X-Box One S pad").device_name == "Microsoft X-Box One S pad");

    // Profile buttons on a reassigned layout: releases change nothing, unmapped buttons nothing.
    {
        RadioConfig c = radiomaster_config("R");
        c.profile.source = ProfileSource::kButtons;
        c.profile.buttons = no_buttons();
        c.profile.buttons[7] = param::k_profile_agile;
        c.profile.buttons[1] = param::k_profile_freestyle;
        Pilot p(c);
        p.event(kAxis, 5, 32767);  // the old CH6 no longer selects
        p.update();
        CHECK(p.profile() == param::k_profile_hold);
        p.event(kButton, 7, 1);
        CHECK(p.profile() == param::k_profile_agile);
        p.event(kButton, 1, 0);
        CHECK(p.profile() == param::k_profile_agile);
        p.event(kButton, 2, 1);
        CHECK(p.profile() == param::k_profile_agile);
        p.event(kButton, 1, 1);
        CHECK(p.profile() == param::k_profile_freestyle);
        p.event(kButton, 31, 1);  // the last button, unmapped
        p.event(kButton, 200, 1);  // beyond kMaxButtons: ignored
        CHECK(p.profile() == param::k_profile_freestyle);
    }

    // Arming interlock: a switch on another axis with its own threshold; a switch already on does not arm; a disarm
    // button; an arm button that needs the throttle at the bottom.
    {
        RadioConfig c = radiomaster_config("R");
        c.arm.index = 6;
        c.arm.on_above = 0.5f;
        c.arm.disarm_button = 3;
        Pilot p(c);
        p.event(kAxis, 2, -32767);  // throttle at the bottom
        p.event(kAxis, 6, 32767);   // already on when first seen
        p.update();
        CHECK(!p.fly() && !p.refused());
        p.event(kAxis, 6, 16383);  // 0.49998: not above the threshold, off
        p.update();
        CHECK(!p.fly());
        p.event(kAxis, 6, 16384);  // 0.50002: on, throttle low: armed
        p.update();
        CHECK(p.fly());
        p.event(kButton, 3, 1);  // disarm button
        CHECK(!p.fly());
        p.update();
        CHECK(!p.fly());  // the switch is still on: it must be cycled
        p.event(kAxis, 6, -32767);
        p.update();
        p.event(kAxis, 2, 0);  // throttle centred
        p.event(kAxis, 6, 32767);
        p.update();
        CHECK(!p.fly() && p.refused());
        p.event(kAxis, 2, -32767);  // throttle down while on: still refused
        p.update();
        CHECK(!p.fly() && p.refused());

        RadioConfig b = xbox_config("X");
        b.arm.require_throttle_low = true;
        Pilot q(b);
        q.event(kAxis, 1, 0);  // Xbox left Y centred: throttle 0
        q.update();
        q.event(kButton, 0, 1);
        CHECK(!q.fly() && q.refused());
        q.event(kAxis, 1, 32767);  // left Y down (reversed): throttle at the bottom
        q.event(kButton, 0, 1);
        CHECK(q.fly() && !q.refused());
        q.event(kButton, 1, 1);
        CHECK(!q.fly());
    }

    // JSON: both built-ins and a six-band config round trip; the text is tidy.
    {
        RadioConfig six = radiomaster_config("EdgeTX Radiomaster Pocket Joystick");
        six.roll = {7, -32000, 12, 32001, true, 0.25f};
        six.arm.disarm_button = 9;
        six.arm.on_above = -0.3f;
        six.profile.band_count = 6;
        const float edges[6] = {-0.8f, -0.33f, 0.f, 0.1f, 0.6f, 1.f};
        for (int i = 0; i < 6; ++i) six.profile.bands[i] = {edges[i], static_cast<std::uint8_t>(i % 4)};
        const RadioConfig cases[] = {radiomaster_config("R"), xbox_config("Microsoft X-Box One S pad"), six};
        for (const RadioConfig& c : cases) {
            const json::value v = to_json(c);
            RadioConfig back;
            const std::string why = from_json(json::parse(json::serialize(v)), back);
            CHECK(why.empty());
            if (!why.empty()) std::printf("  %s\n", why.c_str());
            CHECK(json::serialize(to_json(back)) == json::serialize(v));
            CHECK(back.roll.deadband == c.roll.deadband && back.arm.on_above == c.arm.on_above);
        }
        const std::string text = json::serialize(to_json(radiomaster_config("R")));
        CHECK(text.find("\"deadband\":1E-1") != std::string::npos || text.find("\"deadband\":0.1") != std::string::npos);
        const json::value parsed = json::parse(text);
        const json::array& bands = parsed.at("profile").at("bands").as_array();
        CHECK(bands.size() == 4 && bands[0].at("upper").as_double() == -0.50002 && bands[1].at("upper").to_number<double>() == 0.0 &&
              bands[2].at("upper").as_double() == 0.5 && bands[3].at("upper").to_number<double>() == 1.0);
        CHECK(bands[0].at("profile").as_string() == "hold" && bands[3].at("profile").as_string() == "agile");
        const std::string x = json::serialize(to_json(xbox_config("X")));
        CHECK(x.find("\"profile\":{\"source\":\"buttons\",\"buttons\":{\"2\":\"hold\",\"3\":\"stabilized\",\"4\":\"freestyle\",\"5\":\"agile\"}}") !=
              std::string::npos);
        CHECK(x.find("\"arm\":{\"source\":\"button\",\"button\":0,\"require_throttle_low\":false,\"disarm_button\":1}") !=
              std::string::npos);
        json::value xi = to_json(xbox_config("X"));  // "index" read for "button" when "button" is absent
        xi.as_object()["arm"].as_object().erase("button");
        xi.as_object()["arm"].as_object()["index"] = 7;
        RadioConfig ci;
        CHECK(from_json(xi, ci).empty() && ci.arm.index == 7);
        // "source" (GET's) and unknown keys are ignored; a missing disarm_button is none.
        json::value v = to_json(radiomaster_config("R"));
        v.as_object()["source"] = "stored";
        v.as_object()["arm"].as_object().erase("disarm_button");
        RadioConfig c;
        CHECK(from_json(v, c).empty() && c.arm.disarm_button == -1);
    }

    // Refusals, each naming the field.
    {
        const auto has = [](const std::string& why, const char* path) {
            const bool ok = why.rfind(path, 0) == 0;
            if (!ok) std::printf("  expected %s..., got \"%s\"\n", path, why.c_str());
            return ok;
        };
        CHECK(has(refusal([](json::object& o) { o["version"] = 2; }), "version"));
        CHECK(has(refusal([](json::object& o) { o["device_name"] = ""; }), "device_name"));
        CHECK(has(refusal([](json::object& o) { o.erase("sticks"); }), "sticks: missing"));
        CHECK(has(refusal([](json::object& o) { o["sticks"].as_object()["yaw"].as_object()["axis"] = 16; }), "sticks.yaw.axis"));
        CHECK(has(refusal([](json::object& o) { o["sticks"].as_object()["roll"].as_object()["axis"] = 1.5; }), "sticks.roll.axis"));
        CHECK(has(refusal([](json::object& o) { o["sticks"].as_object()["roll"].as_object()["min"] = 0; }), "sticks.roll.min"));
        CHECK(has(refusal([](json::object& o) { o["sticks"].as_object()["roll"].as_object()["max"] = -5; }), "sticks.roll.max"));
        CHECK(has(refusal([](json::object& o) { o["sticks"].as_object()["roll"].as_object()["max"] = 40000; }), "sticks.roll.max"));
        CHECK(has(refusal([](json::object& o) { o["sticks"].as_object()["roll"].as_object()["reverse"] = 1; }), "sticks.roll.reverse"));
        CHECK(has(refusal([](json::object& o) { o["sticks"].as_object()["pitch"].as_object()["deadband"] = 0.51; }), "sticks.pitch.deadband"));
        CHECK(refusal([](json::object& o) { o["sticks"].as_object()["pitch"].as_object()["deadband"] = 0.5; }).empty());
        CHECK(has(refusal([](json::object& o) { o["sticks"].as_object()["pitch"].as_object()["axis"] = 0; }), "sticks.pitch: axis 0 is already sticks.roll"));
        CHECK(has(refusal([](json::object& o) { o["throttle_centre_hold"] = false; }), "throttle_centre_hold"));
        CHECK(has(refusal([](json::object& o) { o["arm"].as_object()["source"] = "switch"; }), "arm.source"));
        CHECK(has(refusal([](json::object& o) { o["arm"].as_object()["index"] = 2; }), "arm: axis 2 is already sticks.throttle"));
        CHECK(has(refusal([](json::object& o) { o["arm"].as_object()["on_above"] = 1.5; }), "arm.on_above"));
        CHECK(has(refusal([](json::object& o) { o["arm"].as_object()["disarm_button"] = 32; }), "arm.disarm_button"));
        CHECK(has(refusal([](json::object& o) { o["arm"].as_object()["disarm_button"] = nullptr; }, false), "arm.disarm_button: required"));
        CHECK(has(refusal([](json::object& o) { o["arm"].as_object()["disarm_button"] = 0; }, false), "arm.disarm_button: the arm button"));
        CHECK(has(refusal([](json::object& o) { o["profile"].as_object()["index"] = 4; }), "profile: axis 4 is already arm"));
        CHECK(has(refusal([](json::object& o) { o["profile"].as_object()["source"] = "dial"; }), "profile.source"));
        const auto bands = [](std::initializer_list<std::pair<double, const char*>> b) {
            return [b](json::object& o) {
                json::array a;
                for (const auto& [u, p] : b) a.push_back(json::object{{"upper", u}, {"profile", p}});
                o["profile"].as_object()["bands"] = a;
            };
        };
        CHECK(has(refusal(bands({{1.0, "hold"}})), "profile.bands: not 2 to 6"));
        CHECK(has(refusal(bands({{-0.8, "hold"}, {-0.6, "hold"}, {-0.4, "hold"}, {-0.2, "hold"}, {0.0, "hold"}, {0.5, "hold"}, {1.0, "hold"}})),
                  "profile.bands: not 2 to 6"));
        CHECK(has(refusal(bands({{0.5, "hold"}, {0.5, "agile"}, {1.0, "hold"}})), "profile.bands[1].upper: not above"));
        CHECK(has(refusal(bands({{0.5, "hold"}, {0.2, "agile"}, {1.0, "hold"}})), "profile.bands[1].upper: not above"));
        CHECK(has(refusal(bands({{0.0, "hold"}, {0.9, "agile"}})), "profile.bands[1].upper: the last"));
        CHECK(has(refusal(bands({{-1.0, "hold"}, {1.0, "agile"}})), "profile.bands[0].upper: not above -1"));
        CHECK(has(refusal(bands({{0.0, "hold"}, {1.0, "turbo"}})), "profile.bands[1].profile: not a profile id"));
        CHECK(refusal(bands({{0.0, "hold"}, {1.0, "agile"}})).empty());
        CHECK(has(refusal([](json::object& o) { o["profile"].as_object()["buttons"].as_object()["2"] = "turbo"; }, false),
                  "profile.buttons.2: not a profile id"));
        CHECK(has(refusal([](json::object& o) { o["profile"].as_object()["buttons"].as_object()["x"] = "hold"; }, false),
                  "profile.buttons.x: not a button"));
        CHECK(has(refusal([](json::object& o) { o["profile"].as_object()["buttons"].as_object()["32"] = "hold"; }, false),
                  "profile.buttons.32: not a button"));
        CHECK(has(refusal([](json::object& o) { o["profile"].as_object()["buttons"].as_object()["1"] = "hold"; }, false),
                  "profile.buttons.1: the button already"));
        CHECK(has(refusal([](json::object& o) { o["profile"].as_object()["buttons"] = json::object{}; }, false), "profile.buttons: empty"));
        CHECK(refusal([](json::object& o) { o["profile"] = json::object{{"source", "none"}}; }).empty());
        // Against a device's counts: 6 axes, 4 buttons.
        RadioConfig c;
        CHECK(has(from_json(to_json(radiomaster_config("R")), c, 6, 4), ""));
        CHECK(has(from_json(to_json(xbox_config("X")), c, 8, 4), "profile.buttons.4: not a button 0..3"));
        CHECK(has(from_json(to_json(radiomaster_config("R")), c, 5, 4), "profile.index"));
        CHECK(has(from_json(json::value(3), c), "not a JSON object"));
    }

    // The device's file name.
    CHECK(slug("EdgeTX Radiomaster Pocket Joystick") == "edgetx-radiomaster-pocket-joystick");
    CHECK(slug("Microsoft X-Box One S pad") == "microsoft-x-box-one-s-pad");
    CHECK(slug("  ../Pad (v2)!! ") == "pad-v2");
    CHECK(slug("") == "joystick");
    CHECK(slug("///") == "joystick");

    // The file: saved under $XDG_CONFIG_HOME, read back, refused when it names another device, removed.
    {
        char tmpl[] = "/tmp/marv-radio-XXXXXX";
        const std::string root = ::mkdtemp(tmpl);
        ::setenv("XDG_CONFIG_HOME", root.c_str(), 1);
        CHECK(radio_path("Microsoft X-Box One S pad") == root + "/marv/radio/microsoft-x-box-one-s-pad.json");
        RadioConfig c;
        std::string why;
        CHECK(!load_radio("Microsoft X-Box One S pad", c, why) && why.empty());
        RadioConfig x = xbox_config("Microsoft X-Box One S pad");
        x.profile.buttons[5] = param::k_profile_hold;
        CHECK(save_radio(x).empty());
        CHECK(load_radio("Microsoft X-Box One S pad", c, why) && c.profile.buttons[5] == param::k_profile_hold);
        CHECK(std::distance(std::filesystem::directory_iterator(root + "/marv/radio"), {}) == 1);  // no temporary left
        CHECK(!load_radio("Microsoft X-Box One S pad!", c, why) && why.find("not this device") != std::string::npos);
        std::FILE* f = std::fopen(radio_path("Bad").c_str(), "w");
        std::fputs("{\"version\": 1", f);
        std::fclose(f);
        CHECK(!load_radio("Bad", c, why) && why.rfind(radio_path("Bad"), 0) == 0);
        CHECK(remove_radio("Microsoft X-Box One S pad").empty() && remove_radio("Microsoft X-Box One S pad").empty());
        CHECK(!load_radio("Microsoft X-Box One S pad", c, why) && why.empty());
        ::unsetenv("XDG_CONFIG_HOME");
        std::filesystem::remove_all(root);
    }

    std::printf("%s (%d failures)\n", failures ? "FAIL" : "PASS", failures);
    return failures ? EXIT_FAILURE : EXIT_SUCCESS;
}
