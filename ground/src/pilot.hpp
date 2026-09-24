// The pilot's device to a MissionCommand: normalized sticks, the arm switch and the flight profile. Shared by
// marv_ground manual and its test; knows joystick events only as (type, number, value), JS_EVENT_* values.
#pragma once

#include <cmath>
#include <cstdint>

#include <marv/fsw/contracts.hpp>
#include <marv/fsw/params.hpp>

namespace marv::ground {

// Stick axis to -1..1 with a 10% deadband, rescaled so the output starts at 0 at the band's edge.
inline float stick(std::int16_t raw) {
    constexpr float kBand = 0.1f;
    const float v = static_cast<float>(raw) / 32767.f;
    const float a = std::fabs(v);
    if (a <= kBand) return 0.f;
    return std::copysign(std::fmin((a - kBand) / (1.f - kBand), 1.f), v);
}

// RadioMaster CH6 to a profile in four equal bands, edges at -16384, 0 and +16384: below -16384 hold, then freestyle,
// then stabilized from 0, agile from +16384. A four-position mix of -100/-33/+33/+100 % sits mid-band.
inline std::uint8_t ch6_profile(std::int16_t raw) {
    if (raw < -16384) return param::k_profile_hold;
    if (raw < 0) return param::k_profile_freestyle;
    if (raw < 16384) return param::k_profile_stabilized;
    return param::k_profile_agile;
}

// Two mappings, picked by the device name. Each gives up, yaw, forward, right in -1..1 (positive = up, clockwise,
// forward, right), the arm switch and the profile (hold until the pilot picks one).
//   RadioMaster (EdgeTX USB joystick, AETR): a0 aileron = right, a1 elevator = forward, a2 throttle (bottom
//     = -32767, read off the user's radio) = up with the stick centre = hold height, a3 rudder = yaw,
//     a4 (CH5) > 0 = arm, a5 (CH6) = profile (ch6_profile). As on any quad, arming needs the throttle at the bottom
//     when CH5 turns on; otherwise it is refused until CH5 goes off again.
//   Xbox (xpad): a0 left X = yaw, a1 left Y = up, a3/a4 right X/Y = right/forward (up is negative),
//     button A = arm, B = disarm, X = hold, Y = stabilized, LB = freestyle, RB = agile.
class Pilot {
public:
    explicit Pilot(bool radio) : radio_(radio) {}

    // One joystick event; type without the init bit (1 button, 2 axis).
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

    // Once per period, after the period's events: the sticks, the RadioMaster's arm switch and profile.
    void update() {
        sticks_.up = radio_ ? stick(axis_[2]) : 0.f - stick(axis_[1]);
        sticks_.yaw = radio_ ? stick(axis_[3]) : stick(axis_[0]);
        sticks_.fwd = radio_ ? stick(axis_[1]) : 0.f - stick(axis_[4]);
        sticks_.right = radio_ ? stick(axis_[0]) : stick(axis_[3]);
        if (!radio_) return;
        if (seen_[4]) {
            const int sw = axis_[4] > 0 ? 1 : 0;
            if (!sw) fly_ = refused_ = false;
            else if (prev_switch_ == 0 && sticks_.up <= -0.95f) fly_ = true;  // throttle at the bottom
            else if (prev_switch_ == 0) refused_ = true;
            prev_switch_ = sw;
        }
        if (seen_[5]) profile_ = ch6_profile(axis_[5]);
    }

    const Sticks& sticks() const { return sticks_; }
    bool fly() const { return fly_; }
    bool refused() const { return refused_; }
    std::uint8_t profile() const { return profile_; }

    // The frame for this period: the sticks fly (manual) in the selected profile; the reference is a still velocity
    // so a flight controller that ignores the sticks holds.
    MissionCommand command() const { return command(sticks_); }
    // The same with centred sticks: what goes out when the device is lost.
    MissionCommand centred() const { return command(Sticks{0.f, 0.f, 0.f, 0.f}); }

private:
    static constexpr std::uint8_t kButton = 1, kAxis = 2;

    MissionCommand command(const Sticks& s) const {
        MissionCommand c{};
        c.mode = fly_ ? Mode::kFly : Mode::kIdle;
        c.nav = NavSource::kEstimate;
        c.ref.has = kRefVel | kRefYawRate;
        c.ref.q = {1.f, 0.f, 0.f, 0.f};
        c.profile = profile_;
        c.manual = 1;
        c.sticks = s;
        return c;
    }

    bool radio_;
    std::int16_t axis_[8] = {};
    bool seen_[8] = {};
    Sticks sticks_{0.f, 0.f, 0.f, 0.f};
    bool fly_ = false, refused_ = false;
    int prev_switch_ = -1;  // CH5 at the last period; -1 until the device reported it
    std::uint8_t profile_ = param::k_profile_hold;
};

}  // namespace marv::ground
