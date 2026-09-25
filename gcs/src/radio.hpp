// The pilot's joysticks on this computer, for the radio page: every /dev/input/js* open read-only and non-blocking on
// the io_context thread (other readers, marv_ground among them, see the same events), a rescan every 1 s for hot-plug,
// and each device's stored config (ground/src/radio_json.hpp) applied through ground::Pilot, so the page previews
// exactly what marv_ground manual will send.
//   GET /api/radio/devices, GET|PUT|DELETE /api/radio/config: the Http* members (status and JSON body).
//   WS {type: "radio_subscribe", device}: Server streams message(device) at 20 Hz; {type: "radio_devices"} on hot-plug.
#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <boost/asio/io_context.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/json/array.hpp>

#include "link.hpp"

namespace marv::gcs {

struct JoystickInfo {
    std::string path, name;
    int axes = 0, buttons = 0;
};

// Device access, replaceable by a test: the candidate paths, and an opener returning a non-blocking descriptor with
// the device's name and counts filled in, or -1.
struct JoystickPorts {
    std::function<std::vector<std::string>()> paths;
    std::function<int(const std::string& path, JoystickInfo& info)> open;
};
JoystickPorts linux_joysticks();  // /dev/input/js*, JSIOCGNAME / JSIOCGAXES / JSIOCGBUTTONS

struct HttpReply {
    int status;
    std::string body;  // JSON
};

class Radio {
public:
    Radio(boost::asio::io_context& io, Broadcast broadcast, JoystickPorts ports = linux_joysticks());
    ~Radio();

    // [{path, name, axes, buttons, has_config}], and as {type: "radio_devices", devices}.
    boost::json::array devices() const;
    std::string devices_message() const;
    bool present(const std::string& name) const;
    // {type: "radio", device, axes, buttons, normalized: {roll, pitch, throttle, yaw}, arm, profile}, the device's
    // pilot updated first; empty when no device has that name.
    std::string message(const std::string& name);

    HttpReply http_devices() const;
    HttpReply http_get(const std::string& device) const;
    HttpReply http_put(const std::string& body);
    HttpReply http_delete(const std::string& device);

private:
    struct Device;
    void rescan();
    void read(Device& d);
    void configure(Device& d);
    Device* find(const std::string& name) const;

    boost::asio::io_context& io_;
    Broadcast broadcast_;
    JoystickPorts ports_;
    boost::asio::steady_timer timer_;
    std::vector<std::unique_ptr<Device>> devices_;
};

}  // namespace marv::gcs
