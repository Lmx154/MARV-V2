// A RadioConfig as JSON, its validation, and its file: $XDG_CONFIG_HOME/marv/radio/<slug of device_name>.json (else
// ~/.config/marv/radio/...), written atomically. Boost.JSON header-only (boost/json/src.hpp compiled in one TU of the
// program); profile ids are params.hpp's kProfileId, so MARV_PARAMS_TEXT is defined before params.hpp is included.
//
//   {"version": 1, "device_name": "...",
//    "sticks": {"roll"|"pitch"|"throttle"|"yaw": {"axis", "min", "center", "max", "reverse", "deadband"}},
//    "throttle_centre_hold": true,
//    "arm": {"source": "axis", "index", "on_above", "require_throttle_low", "disarm_button": null|N}
//         | {"source": "button", "button" (read as "index" when absent), "require_throttle_low", "disarm_button": N},
//    "profile": {"source": "axis", "index", "bands": [{"upper", "profile"}...]}
//             | {"source": "buttons", "buttons": {"N": "<profile id>", ...}} | {"source": "none"}}
#pragma once

#include <fcntl.h>
#include <unistd.h>

#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

#include <boost/json.hpp>

#include <marv/fsw/params.hpp>

#include "radio_config.hpp"

namespace marv::ground {

namespace radio_json_detail {

namespace json = boost::json;

// A float as the shortest decimal that reads back to it, so 0.1f is written 0.1.
inline double tidy(float f) {
    for (int digits = 1; digits <= 9; ++digits) {
        const double scale = std::pow(10.0, digits);
        const double d = std::round(static_cast<double>(f) * scale) / scale;
        if (static_cast<float>(d) == f) return d;
    }
    return static_cast<double>(f);
}

inline int profile_of(const json::value* v) {
    if (!v || !v->is_string()) return -1;
    for (int p = 0; p < param::kProfileCount; ++p)
        if (v->get_string() == param::kProfileId[p]) return p;
    return -1;
}

// The fields of one object, with the path of each refusal.
struct Reader {
    const json::object& o;
    std::string path;
    std::string& error;

    const json::value* at(const char* key) {
        const json::value* v = o.if_contains(key);
        if (!v && error.empty()) error = path + key + ": missing";
        return v;
    }
    bool fail(const char* key, const std::string& why) {
        if (error.empty()) error = path + key + ": " + why;
        return false;
    }
    bool integer(const char* key, int lo, int hi, int& out) {
        const json::value* v = at(key);
        if (!v) return false;
        if (!v->is_number()) return fail(key, "not a number");
        const double d = v->to_number<double>();
        if (d != std::floor(d)) return fail(key, "not an integer");
        if (d < lo || d > hi) return fail(key, "outside " + std::to_string(lo) + ".." + std::to_string(hi));
        out = static_cast<int>(d);
        return true;
    }
    bool number(const char* key, double lo, double hi, float& out) {
        const json::value* v = at(key);
        if (!v) return false;
        if (!v->is_number()) return fail(key, "not a number");
        const double d = v->to_number<double>();
        if (!(d >= lo && d <= hi)) return fail(key, "outside " + json::serialize(json::value(lo)) + ".." + json::serialize(json::value(hi)));
        out = static_cast<float>(d);
        return true;
    }
    bool boolean(const char* key, bool& out) {
        const json::value* v = at(key);
        if (!v) return false;
        if (!v->is_bool()) return fail(key, "not true or false");
        out = v->get_bool();
        return true;
    }
    const json::object* object(const char* key) {
        const json::value* v = at(key);
        if (v && !v->is_object()) fail(key, "not an object");
        return v && v->is_object() ? &v->get_object() : nullptr;
    }
    bool string(const char* key, std::string& out) {
        const json::value* v = at(key);
        if (!v) return false;
        if (!v->is_string()) return fail(key, "not a string");
        out = std::string(v->get_string());
        return true;
    }
};

}  // namespace radio_json_detail

inline boost::json::object to_json(const RadioConfig& c) {
    namespace json = boost::json;
    using radio_json_detail::tidy;
    const auto stick = [](const StickAxis& s) {
        return json::object{{"axis", s.axis},       {"min", s.min},         {"center", s.center},
                            {"max", s.max},         {"reverse", s.reverse}, {"deadband", tidy(s.deadband)}};
    };
    json::object arm;
    if (c.arm.source == ArmSource::kAxis) arm = {{"source", "axis"}, {"index", c.arm.index}, {"on_above", tidy(c.arm.on_above)}};
    else arm = {{"source", "button"}, {"button", c.arm.index}};
    arm["require_throttle_low"] = c.arm.require_throttle_low;
    arm["disarm_button"] = c.arm.disarm_button < 0 ? json::value(nullptr) : json::value(c.arm.disarm_button);
    json::object profile;
    if (c.profile.source == ProfileSource::kAxis) {
        json::array bands;
        for (int i = 0; i < c.profile.band_count; ++i)
            bands.push_back(json::object{{"upper", tidy(c.profile.bands[i].upper)},
                                         {"profile", param::kProfileId[c.profile.bands[i].profile]}});
        profile = {{"source", "axis"}, {"index", c.profile.index}, {"bands", std::move(bands)}};
    } else if (c.profile.source == ProfileSource::kButtons) {
        json::object buttons;
        for (int b = 0; b < kMaxButtons; ++b)
            if (c.profile.buttons[b] >= 0) buttons[std::to_string(b)] = param::kProfileId[c.profile.buttons[b]];
        profile = {{"source", "buttons"}, {"buttons", std::move(buttons)}};
    } else {
        profile = {{"source", "none"}};
    }
    return json::object{{"version", c.version},
                        {"device_name", c.device_name},
                        {"sticks", json::object{{"roll", stick(c.roll)},
                                                {"pitch", stick(c.pitch)},
                                                {"throttle", stick(c.throttle)},
                                                {"yaw", stick(c.yaw)}}},
                        {"throttle_centre_hold", c.throttle_centre_hold},
                        {"arm", std::move(arm)},
                        {"profile", std::move(profile)}};
}

// Reads and validates a config: every field present and in range, the band edges ascending up to 1, the profile ids
// known, no axis or button doing two jobs, and every index below the device's axes / buttons. Keys it does not know
// ("source" of GET /api/radio/config among them) are ignored. Returns "" or the first refusal, as "path: why".
inline std::string from_json(const boost::json::value& v, RadioConfig& out, int axes = kMaxAxes, int buttons = kMaxButtons) {
    namespace json = boost::json;
    using radio_json_detail::profile_of;
    using radio_json_detail::Reader;
    if (!v.is_object()) return "not a JSON object";
    axes = axes < kMaxAxes ? axes : kMaxAxes;
    buttons = buttons < kMaxButtons ? buttons : kMaxButtons;
    std::string error;
    RadioConfig c;
    Reader top{v.get_object(), "", error};
    if (top.integer("version", 1, 1, c.version) && top.string("device_name", c.device_name) && c.device_name.empty())
        top.fail("device_name", "empty");

    int used_axis[kMaxAxes] = {};  // 1 + the job's number, 0 free
    const char* const kJobs[] = {"sticks.roll", "sticks.pitch", "sticks.throttle", "sticks.yaw", "arm", "profile"};
    const auto claim_axis = [&](int job, int axis) {
        if (!error.empty()) return;
        if (used_axis[axis]) error = std::string(kJobs[job]) + ": axis " + std::to_string(axis) + " is already " + kJobs[used_axis[axis] - 1];
        else used_axis[axis] = job + 1;
    };
    if (const json::object* sticks = top.object("sticks")) {
        StickAxis* const dst[] = {&c.roll, &c.pitch, &c.throttle, &c.yaw};
        const char* const names[] = {"roll", "pitch", "throttle", "yaw"};
        Reader sr{*sticks, "sticks.", error};
        for (int i = 0; i < 4 && error.empty(); ++i) {
            const json::object* o = sr.object(names[i]);
            if (!o) break;
            Reader r{*o, std::string("sticks.") + names[i] + ".", error};
            StickAxis& s = *dst[i];
            if (r.integer("axis", 0, axes - 1, s.axis) && r.integer("min", -32768, 32767, s.min) &&
                r.integer("center", -32768, 32767, s.center) && r.integer("max", -32768, 32767, s.max) &&
                r.boolean("reverse", s.reverse) && r.number("deadband", 0.0, 0.5, s.deadband)) {
                if (!(s.min < s.center)) r.fail("min", "not below center");
                else if (!(s.center < s.max)) r.fail("max", "not above center");
                claim_axis(i, s.axis);
            }
        }
    }
    if (error.empty() && top.boolean("throttle_centre_hold", c.throttle_centre_hold) && !c.throttle_centre_hold)
        top.fail("throttle_centre_hold", "only true (the throttle's centre holds height) is supported");

    if (const json::object* arm = error.empty() ? top.object("arm") : nullptr) {
        Reader r{*arm, "arm.", error};
        std::string source;
        RadioConfig::Arm& a = c.arm;
        if (r.string("source", source)) {
            if (source == "axis") {
                a.source = ArmSource::kAxis;
                if (r.integer("index", 0, axes - 1, a.index) && r.number("on_above", -1.0, 1.0, a.on_above))
                    claim_axis(4, a.index);
            } else if (source == "button") {
                a.source = ArmSource::kButton;
                r.integer(arm->contains("button") || !arm->contains("index") ? "button" : "index", 0, buttons - 1, a.index);
            } else {
                r.fail("source", "not \"axis\" or \"button\"");
            }
        }
        if (error.empty()) r.boolean("require_throttle_low", a.require_throttle_low);
        const json::value* d = arm->if_contains("disarm_button");
        a.disarm_button = -1;
        if (error.empty() && d && !d->is_null()) r.integer("disarm_button", 0, buttons - 1, a.disarm_button);
        if (error.empty() && a.source == ArmSource::kButton && a.disarm_button < 0)
            r.fail("disarm_button", "required with an arm button");
        if (error.empty() && a.source == ArmSource::kButton && a.disarm_button == a.index)
            r.fail("disarm_button", "the arm button itself");
    }

    if (const json::object* profile = error.empty() ? top.object("profile") : nullptr) {
        Reader r{*profile, "profile.", error};
        std::string source;
        RadioConfig::Profile& p = c.profile;
        p.index = 0;
        p.band_count = 0;
        p.buttons = no_buttons();
        if (r.string("source", source)) {
            if (source == "axis") {
                p.source = ProfileSource::kAxis;
                const json::value* bands = r.integer("index", 0, axes - 1, p.index) ? r.at("bands") : nullptr;
                claim_axis(5, p.index);
                if (error.empty() && bands && !bands->is_array()) r.fail("bands", "not an array");
                if (error.empty() && bands) {
                    const json::array& b = bands->get_array();
                    if (b.size() < kMinBands || b.size() > kMaxBands)
                        r.fail("bands", "not " + std::to_string(kMinBands) + " to " + std::to_string(kMaxBands) + " bands");
                    for (std::size_t i = 0; error.empty() && i < b.size(); ++i) {
                        const std::string at = "profile.bands[" + std::to_string(i) + "].";
                        if (!b[i].is_object()) {
                            error = at + ": not an object";
                            break;
                        }
                        Reader br{b[i].get_object(), at, error};
                        Band& band = p.bands[i];
                        if (!br.number("upper", -1.0, 1.0, band.upper)) break;
                        const int id = profile_of(br.at("profile"));
                        if (!error.empty()) break;
                        if (id < 0) br.fail("profile", "not a profile id");
                        else if (i > 0 && !(band.upper > p.bands[i - 1].upper)) br.fail("upper", "not above the band before");
                        else if (band.upper <= -1.f) br.fail("upper", "not above -1");
                        else if (i + 1 == b.size() && band.upper != 1.f) br.fail("upper", "the last band's is not 1");
                        band.profile = static_cast<std::uint8_t>(id);
                    }
                    p.band_count = static_cast<int>(b.size());
                }
            } else if (source == "buttons") {
                p.source = ProfileSource::kButtons;
                if (const json::object* map = r.object("buttons")) {
                    if (map->empty()) r.fail("buttons", "empty");
                    for (const auto& kv : *map) {
                        if (!error.empty()) break;
                        const std::string key(kv.key());
                        char* end = nullptr;
                        const long b = std::strtol(key.c_str(), &end, 10);
                        const int id = profile_of(&kv.value());
                        if (key.empty() || *end || b < 0 || b >= buttons)
                            error = "profile.buttons." + key + ": not a button 0.." + std::to_string(buttons - 1);
                        else if (id < 0) error = "profile.buttons." + key + ": not a profile id";
                        else if ((c.arm.source == ArmSource::kButton && b == c.arm.index) || b == c.arm.disarm_button)
                            error = "profile.buttons." + key + ": the button already arms or disarms";
                        else p.buttons[static_cast<std::size_t>(b)] = static_cast<std::int8_t>(id);
                    }
                }
            } else if (source == "none") {
                p.source = ProfileSource::kNone;
            } else {
                r.fail("source", "not \"axis\", \"buttons\" or \"none\"");
            }
        }
    }
    if (error.empty()) out = c;
    return error;
}

// $XDG_CONFIG_HOME/marv/radio (else $HOME/.config/marv/radio).
inline std::string radio_dir() {
    const char* xdg = std::getenv("XDG_CONFIG_HOME");
    if (xdg && *xdg) return std::string(xdg) + "/marv/radio";
    const char* home = std::getenv("HOME");
    return std::string(home ? home : ".") + "/.config/marv/radio";
}

inline std::string radio_path(const std::string& device_name) { return radio_dir() + "/" + slug(device_name) + ".json"; }

// The stored config of a device into out. Returns false with error empty when there is none, false with the reason
// when there is one that does not read (or names another device), true when out holds it.
inline bool load_radio(const std::string& device_name, RadioConfig& out, std::string& error) {
    error.clear();
    const std::string path = radio_path(device_name);
    std::ifstream f(path);
    if (!f) return false;
    std::stringstream ss;
    ss << f.rdbuf();
    boost::system::error_code ec;
    const boost::json::value v = boost::json::parse(ss.str(), ec);
    if (ec) error = path + ": " + ec.message();
    else error = from_json(v, out);
    if (error.empty() && out.device_name != device_name) error = "names \"" + out.device_name + "\", not this device";
    if (!error.empty() && error.rfind(path, 0) != 0) error = path + ": " + error;
    return error.empty();
}

// Writes the config to its file (the directories made as needed) through a temporary file and a rename, so a reader
// sees the old file or the new one. Returns "" or why not.
inline std::string save_radio(const RadioConfig& c) {
    const std::string dir = radio_dir();
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
    if (ec) return "mkdir " + dir + ": " + ec.message();
    const std::string path = radio_path(c.device_name);
    const std::string tmp = path + ".tmp" + std::to_string(::getpid());
    const std::string text = boost::json::serialize(to_json(c)) + "\n";
    const int fd = ::open(tmp.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
    if (fd < 0) return "open " + tmp + ": " + std::strerror(errno);
    const bool ok = ::write(fd, text.data(), text.size()) == static_cast<ssize_t>(text.size()) && ::fsync(fd) == 0;
    const int err = errno;
    ::close(fd);
    if (!ok || ::rename(tmp.c_str(), path.c_str()) != 0) {
        const std::string why = std::string(ok ? "rename to " + path : "write " + tmp) + ": " + std::strerror(ok ? errno : err);
        ::unlink(tmp.c_str());
        return why;
    }
    return "";
}

// Removes a device's stored config (back to the built-in one). Returns "" or why not; none stored is not an error.
inline std::string remove_radio(const std::string& device_name) {
    const std::string path = radio_path(device_name);
    if (::unlink(path.c_str()) != 0 && errno != ENOENT) return "unlink " + path + ": " + std::strerror(errno);
    return "";
}

}  // namespace marv::ground
