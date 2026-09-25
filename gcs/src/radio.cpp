#include "radio.hpp"

#include <fcntl.h>
#include <linux/joystick.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <utility>

#include <boost/asio/posix/stream_descriptor.hpp>
#include <boost/json.hpp>

#include "pilot.hpp"
#include "radio_json.hpp"

namespace marv::gcs {
namespace asio = boost::asio;
namespace json = boost::json;

JoystickPorts linux_joysticks() {
    JoystickPorts p;
    p.paths = [] {
        std::vector<std::string> out;
        std::error_code ec;
        for (const auto& e : std::filesystem::directory_iterator("/dev/input", ec)) {
            const std::string n = e.path().filename().string();
            if (n.size() > 2 && n.rfind("js", 0) == 0 && n.find_first_not_of("0123456789", 2) == std::string::npos)
                out.push_back(e.path().string());
        }
        std::sort(out.begin(), out.end());
        return out;
    };
    p.open = [](const std::string& path, JoystickInfo& info) {
        const int fd = ::open(path.c_str(), O_RDONLY | O_NONBLOCK | O_CLOEXEC);
        if (fd < 0) return -1;
        char name[128] = "";
        unsigned char axes = 0, buttons = 0;
        if (::ioctl(fd, JSIOCGNAME(sizeof(name)), name) < 0 || ::ioctl(fd, JSIOCGAXES, &axes) < 0 ||
            ::ioctl(fd, JSIOCGBUTTONS, &buttons) < 0) {
            ::close(fd);
            return -1;
        }
        info = {path, name, axes, buttons};
        return fd;
    };
    return p;
}

struct Radio::Device {
    Device(asio::io_context& io, int fd, JoystickInfo i)
        : info(std::move(i)),
          sd(io, fd),
          axes(static_cast<std::size_t>(info.axes), 0),
          seen(static_cast<std::size_t>(info.axes), false),
          buttons(static_cast<std::size_t>(info.buttons), 0),
          pilot(ground::default_config(info.name)) {}
    JoystickInfo info;
    asio::posix::stream_descriptor sd;
    std::vector<std::int16_t> axes;
    std::vector<bool> seen;
    std::vector<std::uint8_t> buttons;
    ground::Pilot pilot;
    bool stored = false;
    std::array<unsigned char, 64 * sizeof(js_event)> buf{};
    std::size_t have = 0;
};

namespace {

std::string error_body(const std::string& why) { return json::serialize(json::object{{"error", why}}); }

// A config for the HTTP replies: the config's fields and "source"; "error" when a stored one was ignored.
std::string config_body(const ground::RadioConfig& c, const char* source, const std::string& why = "") {
    json::object o = ground::to_json(c);
    o["source"] = source;
    if (!why.empty()) o["error"] = why;
    return json::serialize(o);
}

}  // namespace

Radio::Radio(asio::io_context& io, Broadcast broadcast, JoystickPorts ports)
    : io_(io), broadcast_(std::move(broadcast)), ports_(std::move(ports)), timer_(io) {
    rescan();
}

Radio::~Radio() = default;

void Radio::rescan() {
    bool changed = false;
    for (const std::string& path : ports_.paths()) {
        if (std::any_of(devices_.begin(), devices_.end(), [&](const auto& d) { return d->info.path == path; })) continue;
        JoystickInfo info;
        const int fd = ports_.open(path, info);
        if (fd < 0) continue;
        info.axes = std::clamp(info.axes, 0, 255);
        info.buttons = std::clamp(info.buttons, 0, 255);
        try {
            devices_.push_back(std::make_unique<Device>(io_, fd, info));
        } catch (const boost::system::system_error& e) {  // not pollable
            std::printf("marv_gcs: joystick %s: %s\n", path.c_str(), e.what());
            ::close(fd);
            continue;
        }
        configure(*devices_.back());
        read(*devices_.back());
        changed = true;
        std::printf("marv_gcs: joystick %s \"%s\" (%d axes, %d buttons)\n", path.c_str(), info.name.c_str(), info.axes,
                    info.buttons);
        std::fflush(stdout);
    }
    if (changed) broadcast_(devices_message(), false);
    timer_.expires_after(std::chrono::seconds(1));
    timer_.async_wait([this](const boost::system::error_code& ec) {
        if (!ec) rescan();
    });
}

void Radio::read(Device& d) {
    d.sd.async_read_some(asio::buffer(d.buf.data() + d.have, d.buf.size() - d.have),
                         [this, &d](const boost::system::error_code& ec, std::size_t n) {
                             if (ec == asio::error::operation_aborted) return;
                             if (ec) {  // unplugged
                                 std::printf("marv_gcs: joystick %s gone (%s)\n", d.info.path.c_str(), ec.message().c_str());
                                 std::fflush(stdout);
                                 devices_.erase(std::find_if(devices_.begin(), devices_.end(),
                                                             [&d](const auto& p) { return p.get() == &d; }));
                                 broadcast_(devices_message(), false);
                                 return;
                             }
                             d.have += n;
                             std::size_t at = 0;
                             for (; at + sizeof(js_event) <= d.have; at += sizeof(js_event)) {
                                 js_event e;
                                 std::memcpy(&e, d.buf.data() + at, sizeof(e));
                                 const std::uint8_t type = e.type & static_cast<std::uint8_t>(~JS_EVENT_INIT);
                                 if (type == JS_EVENT_AXIS) {
                                     if (e.number >= d.axes.size()) {
                                         d.axes.resize(e.number + 1u, 0);
                                         d.seen.resize(e.number + 1u, false);
                                     }
                                     d.axes[e.number] = e.value;
                                     d.seen[e.number] = true;
                                 } else if (type == JS_EVENT_BUTTON) {
                                     if (e.number >= d.buttons.size()) d.buttons.resize(e.number + 1u, 0);
                                     d.buttons[e.number] = e.value ? 1 : 0;
                                 }
                                 d.pilot.event(type, e.number, e.value);
                             }
                             std::memmove(d.buf.data(), d.buf.data() + at, d.have - at);
                             d.have -= at;
                             read(d);
                         });
}

// The device's stored config, else its built-in one, in a fresh pilot fed the axes seen so far.
void Radio::configure(Device& d) {
    ground::RadioConfig cfg;
    std::string why;
    d.stored = ground::load_radio(d.info.name, cfg, why);
    if (!d.stored) cfg = ground::default_config(d.info.name);
    d.pilot = ground::Pilot(cfg);
    for (std::size_t i = 0; i < d.axes.size() && i < 256; ++i)
        if (d.seen[i]) d.pilot.event(JS_EVENT_AXIS, static_cast<std::uint8_t>(i), d.axes[i]);
}

Radio::Device* Radio::find(const std::string& name) const {
    for (const auto& d : devices_)
        if (d->info.name == name) return d.get();
    return nullptr;
}

bool Radio::present(const std::string& name) const { return find(name) != nullptr; }

json::array Radio::devices() const {
    json::array a;
    for (const auto& d : devices_) {
        std::error_code ec;
        a.push_back(json::object{{"path", d->info.path},
                                 {"name", d->info.name},
                                 {"axes", d->info.axes},
                                 {"buttons", d->info.buttons},
                                 {"has_config", std::filesystem::exists(ground::radio_path(d->info.name), ec)}});
    }
    return a;
}

std::string Radio::devices_message() const {
    return json::serialize(json::object{{"type", "radio_devices"}, {"devices", devices()}});
}

std::string Radio::message(const std::string& name) {
    Device* d = find(name);
    if (!d) return "";
    d->pilot.update();
    const Sticks& s = d->pilot.sticks();
    json::array axes, buttons;
    for (const std::int16_t v : d->axes) axes.push_back(v);
    for (const std::uint8_t v : d->buttons) buttons.push_back(v);
    const bool none = d->pilot.config().profile.source == ground::ProfileSource::kNone;
    return json::serialize(json::object{
        {"type", "radio"},
        {"device", name},
        {"axes", std::move(axes)},
        {"buttons", std::move(buttons)},
        {"normalized", json::object{{"roll", s.right}, {"pitch", s.fwd}, {"throttle", s.up}, {"yaw", s.yaw}}},
        {"arm", d->pilot.fly()},
        {"profile", none ? json::value(nullptr) : json::value(param::kProfileId[d->pilot.profile()])}});
}

HttpReply Radio::http_devices() const { return {200, json::serialize(devices())}; }

HttpReply Radio::http_get(const std::string& device) const {
    if (device.empty()) return {400, error_body("device: missing")};
    ground::RadioConfig cfg;
    std::string why;
    if (ground::load_radio(device, cfg, why)) return {200, config_body(cfg, "stored")};
    return {200, config_body(ground::default_config(device), "default", why)};
}

HttpReply Radio::http_put(const std::string& body) {
    boost::system::error_code ec;
    const json::value v = json::parse(body, ec);
    if (ec) return {400, error_body("not JSON: " + ec.message())};
    // Indices are checked against the device's counts when it is plugged in.
    const json::value* name = v.is_object() ? v.get_object().if_contains("device_name") : nullptr;
    const Device* d = name && name->is_string() ? find(std::string(name->get_string())) : nullptr;
    ground::RadioConfig cfg;
    const std::string why = d ? ground::from_json(v, cfg, d->info.axes, d->info.buttons) : ground::from_json(v, cfg);
    if (!why.empty()) return {400, error_body(why)};
    const std::string failed = ground::save_radio(cfg);
    if (!failed.empty()) return {500, error_body(failed)};
    for (const auto& dev : devices_)
        if (dev->info.name == cfg.device_name) configure(*dev);
    return {200, config_body(cfg, "stored")};
}

HttpReply Radio::http_delete(const std::string& device) {
    if (device.empty()) return {400, error_body("device: missing")};
    const std::string failed = ground::remove_radio(device);
    if (!failed.empty()) return {500, error_body(failed)};
    for (const auto& dev : devices_)
        if (dev->info.name == device) configure(*dev);
    return {200, config_body(ground::default_config(device), "default")};
}

}  // namespace marv::gcs
