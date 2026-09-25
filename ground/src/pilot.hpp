// The pilot's device to a MissionCommand: normalized sticks, the arm switch and the flight profile, as a RadioConfig
// maps them. Shared by marv_ground manual, marv_gcs's radio preview and their tests; knows joystick events only as
// (type, number, value), JS_EVENT_* values.
#pragma once

#include <cstdint>
#include <utility>

#include <marv/fsw/contracts.hpp>
#include <marv/fsw/params.hpp>

#include "radio_config.hpp"

namespace marv::ground {

// The configured mapping (radio_config.hpp): sticks to up, yaw, forward, right in -1..1 (positive = up, clockwise,
// forward, right), the arm switch or buttons, and the profile (hold until the pilot picks one). An arm switch arms only
// when it turns on (with require_throttle_low, with the throttle at the bottom; otherwise it is refused until it goes
// off again), so a switch already on when the device appears does not arm; an arm button arms on its press.
class Pilot {
public:
    explicit Pilot(RadioConfig cfg) : cfg_(std::move(cfg)) {}
    // The built-in RadioMaster (radio) or Xbox mapping.
    explicit Pilot(bool radio) : Pilot(radio ? radiomaster_config("") : xbox_config("")) {}

    const RadioConfig& config() const { return cfg_; }

    // One joystick event; type without the init bit (1 button, 2 axis). Buttons act at once.
    void event(std::uint8_t type, std::uint8_t number, std::int16_t value) {
        if (type == kAxis && number < kMaxAxes) {
            axis_[number] = value;
            seen_[number] = true;
        } else if (type == kButton && value && number < kMaxButtons) {
            button(number);
        }
    }

    // Once per period, after the period's events: the sticks, the arm switch and the profile switch.
    void update() {
        sticks_.up = axis(cfg_.throttle);
        sticks_.yaw = axis(cfg_.yaw);
        sticks_.fwd = axis(cfg_.pitch);
        sticks_.right = axis(cfg_.roll);
        const RadioConfig::Arm& a = cfg_.arm;
        if (a.source == ArmSource::kAxis && seen_[a.index]) {
            const int sw = switch_value(axis_[a.index]) > a.on_above ? 1 : 0;
            if (!sw) fly_ = refused_ = false;
            else if (prev_switch_ == 0 && (!a.require_throttle_low || sticks_.up <= kThrottleLow)) fly_ = true;
            else if (prev_switch_ == 0) refused_ = true;
            prev_switch_ = sw;
        }
        const RadioConfig::Profile& p = cfg_.profile;
        if (p.source == ProfileSource::kAxis && seen_[p.index]) profile_ = band_profile(p, axis_[p.index]);
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

    float axis(const StickAxis& s) const { return normalize(s, axis_[s.axis]); }

    void button(int n) {
        const RadioConfig::Arm& a = cfg_.arm;
        if (a.source == ArmSource::kButton && n == a.index) {
            refused_ = a.require_throttle_low && axis(cfg_.throttle) > kThrottleLow;
            fly_ = fly_ || !refused_;
        }
        if (n == a.disarm_button) fly_ = refused_ = false;
        if (cfg_.profile.source == ProfileSource::kButtons && cfg_.profile.buttons[n] >= 0)
            profile_ = static_cast<std::uint8_t>(cfg_.profile.buttons[n]);
    }

    RadioConfig cfg_;
    std::int16_t axis_[kMaxAxes] = {};
    bool seen_[kMaxAxes] = {};
    Sticks sticks_{0.f, 0.f, 0.f, 0.f};
    bool fly_ = false, refused_ = false;
    int prev_switch_ = -1;  // the arm switch at the last period; -1 until the device reported it
    std::uint8_t profile_ = param::k_profile_hold;
};

}  // namespace marv::ground
