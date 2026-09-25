// The pilot's device, configured: which axis is which stick and its calibration, the arm switch or buttons, and which
// switch or buttons select the flight profile. Built-in defaults for the RadioMaster and the Xbox pad; a stored one per
// device (radio_json.hpp) replaces them. Shared by marv_ground manual, marv_gcs's radio page and their tests.
#pragma once

#include <cctype>
#include <cmath>
#include <array>
#include <cstdint>
#include <string>

#include <marv/fsw/params.hpp>

namespace marv::ground {

inline constexpr int kMaxAxes = 16, kMaxButtons = 32, kMinBands = 2, kMaxBands = 6;
inline constexpr float kThrottleLow = -0.95f;  // arming with require_throttle_low: the throttle at the bottom

// One stick: raw -32768..32767 to -1..1, linear from min to center and center to max, reversed, then the deadband
// cut out and the rest rescaled so the output starts at 0 at the band's edge.
struct StickAxis {
    int axis = 0;
    int min = -32767, center = 0, max = 32767;
    bool reverse = false;
    float deadband = 0.1f;
};

struct Band {
    float upper;  // the first band whose upper is above the switch value holds it; the last band's upper is 1
    std::uint8_t profile;
};

inline constexpr std::array<std::int8_t, kMaxButtons> no_buttons() {
    std::array<std::int8_t, kMaxButtons> b{};
    for (std::int8_t& x : b) x = -1;
    return b;
}

enum class ArmSource : std::uint8_t { kAxis, kButton };
enum class ProfileSource : std::uint8_t { kAxis, kButtons, kNone };

struct RadioConfig {
    int version = 1;
    std::string device_name;
    StickAxis roll, pitch, throttle, yaw;  // roll = right, pitch = forward, throttle = up (centre holds), yaw = clockwise
    bool throttle_centre_hold = true;
    struct Arm {
        ArmSource source = ArmSource::kAxis;
        int index = 4;          // the axis (switch on while above on_above) or the button (a press arms)
        float on_above = 0.f;   // kAxis: switch value threshold
        bool require_throttle_low = true;
        int disarm_button = -1;  // a press disarms; -1: none (required with kButton)
    } arm;
    struct Profile {
        ProfileSource source = ProfileSource::kAxis;
        int index = 5;  // kAxis: the axis
        int band_count = 0;
        Band bands[kMaxBands] = {};
        std::array<std::int8_t, kMaxButtons> buttons = no_buttons();  // kButtons: the profile a press selects, -1 none
    } profile;
};

inline float normalize(const StickAxis& s, std::int16_t raw) {
    float v = raw >= s.center ? static_cast<float>(raw - s.center) / static_cast<float>(s.max - s.center)
                              : static_cast<float>(raw - s.center) / static_cast<float>(s.center - s.min);
    v = std::fmax(-1.f, std::fmin(v, 1.f));
    if (s.reverse) v = -v;
    const float a = std::fabs(v);
    if (a <= s.deadband) return 0.f;
    return std::copysign(std::fmin((a - s.deadband) / (1.f - s.deadband), 1.f), v);
}

// A switch or knob channel, uncalibrated: raw / 32767, clamped to -1..1.
inline float switch_value(std::int16_t raw) { return std::fmax(-1.f, static_cast<float>(raw) / 32767.f); }

// The profile of the band raw falls in (profile.source kAxis).
inline std::uint8_t band_profile(const RadioConfig::Profile& p, std::int16_t raw) {
    const float v = switch_value(raw);
    for (int i = 0; i + 1 < p.band_count; ++i)
        if (v < p.bands[i].upper) return p.bands[i].profile;
    return p.band_count > 0 ? p.bands[p.band_count - 1].profile : static_cast<std::uint8_t>(param::k_profile_hold);
}

inline bool is_radio(const std::string& name) {
    std::string n;
    for (const char c : name) n += static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return n.find("edgetx") != std::string::npos || n.find("radiomaster") != std::string::npos;
}

// RadioMaster (EdgeTX USB joystick, AETR): a0 aileron = roll, a1 elevator = pitch, a2 throttle (bottom = -32767) with
// the centre holding height, a3 rudder = yaw; a4 (CH5) above 0 arms with the throttle at the bottom; a5 (CH6) picks the
// profile in four equal bands, edges at -16384, 0 and +16384: hold, freestyle, stabilized, agile.
inline RadioConfig radiomaster_config(const std::string& name) {
    RadioConfig c;
    c.device_name = name;
    c.roll.axis = 0;
    c.pitch.axis = 1;
    c.throttle.axis = 2;
    c.yaw.axis = 3;
    c.profile.band_count = 4;
    c.profile.bands[0] = {-0.50002f, param::k_profile_hold};  // raw -16385 (-0.500046) is below, -16384 (-0.500015) not
    c.profile.bands[1] = {0.f, param::k_profile_freestyle};
    c.profile.bands[2] = {0.5f, param::k_profile_stabilized};
    c.profile.bands[3] = {1.f, param::k_profile_agile};
    return c;
}

// Xbox (xpad): a0 left X = yaw, a1 left Y = up, a3/a4 right X/Y = right/forward (up is negative); button A arms, B
// disarms, X hold, Y stabilized, LB freestyle, RB agile.
inline RadioConfig xbox_config(const std::string& name) {
    RadioConfig c;
    c.device_name = name;
    c.roll.axis = 3;
    c.pitch.axis = 4;
    c.pitch.reverse = true;
    c.throttle.axis = 1;
    c.throttle.reverse = true;
    c.yaw.axis = 0;
    c.arm.source = ArmSource::kButton;
    c.arm.index = 0;
    c.arm.require_throttle_low = false;
    c.arm.disarm_button = 1;
    c.profile.source = ProfileSource::kButtons;
    c.profile.index = 0;
    c.profile.buttons[2] = param::k_profile_hold;
    c.profile.buttons[3] = param::k_profile_stabilized;
    c.profile.buttons[4] = param::k_profile_freestyle;
    c.profile.buttons[5] = param::k_profile_agile;
    return c;
}

// The built-in mapping for a device: the RadioMaster's when the name says EdgeTX or RadioMaster, else the Xbox pad's.
inline RadioConfig default_config(const std::string& name) {
    return is_radio(name) ? radiomaster_config(name) : xbox_config(name);
}

// The file name of a device's stored config: lower-case letters and digits, every other run as one '-'.
inline std::string slug(const std::string& name) {
    std::string s;
    for (const char c : name) {
        const unsigned char u = static_cast<unsigned char>(c);
        if (u < 0x80 && std::isalnum(u)) s += static_cast<char>(std::tolower(u));
        else if (!s.empty() && s.back() != '-') s += '-';
        if (s.size() >= 64) break;
    }
    while (!s.empty() && s.back() == '-') s.pop_back();
    return s.empty() ? "joystick" : s;
}

// The default RadioMaster stick and CH6 bands, as marv_ground used them before configs.
inline float stick(std::int16_t raw) { return normalize(StickAxis{}, raw); }
inline std::uint8_t ch6_profile(std::int16_t raw) { return band_profile(radiomaster_config("").profile, raw); }

}  // namespace marv::ground
