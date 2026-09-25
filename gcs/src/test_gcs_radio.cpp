// The radio page's backend (radio.hpp) against a fake joystick: a FIFO carrying js_event records, opened through the
// injectable JoystickPorts as "EdgeTX Radiomaster Pocket Joystick" (8 axes, 4 buttons). Over HTTP and the WebSocket:
//   1. GET /api/radio/devices lists it; GET /api/radio/config gives the built-in RadioMaster config ("default").
//   2. radio_subscribe streams raw axes and buttons, the normalized sticks, the arm switch with its throttle interlock
//      and the CH6 profile, as marv_ground would send them; an unknown device is refused.
//   3. PUT stores a config (profile by button), the stream follows it at once and a switch left on does not arm;
//      invalid ones (band order, an axis the device lacks, not JSON) are refused with the field; DELETE returns to the
//      default.
//   4. hot-plug: the device leaving and coming back is broadcast as radio_devices.
#include <fcntl.h>
#include <linux/joystick.h>
#include <sys/stat.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <thread>
#include <utility>

#include <boost/asio/io_context.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/json.hpp>

#include "link.hpp"
#include "radio.hpp"
#include "radio_json.hpp"
#include "server.hpp"

namespace {

int g_fails = 0;
#define CHECK(c)                                                                       \
    do {                                                                               \
        if (!(c)) {                                                                    \
            std::fprintf(stderr, "%s:%d: CHECK(%s) failed\n", __FILE__, __LINE__, #c); \
            ++g_fails;                                                                 \
        }                                                                              \
    } while (0)

namespace asio = boost::asio;
namespace beast = boost::beast;
namespace http = beast::http;
namespace websocket = beast::websocket;
namespace json = boost::json;
using tcp = asio::ip::tcp;
using namespace marv;

const std::string kName = "EdgeTX Radiomaster Pocket Joystick";
const std::string kQuery = "?device=EdgeTX%20Radiomaster+Pocket%20Joystick";

std::atomic<bool> g_plugged{true};

gcs::JoystickPorts fake_ports(const std::string& fifo) {
    gcs::JoystickPorts p;
    p.paths = [fifo] { return g_plugged ? std::vector<std::string>{fifo} : std::vector<std::string>{}; };
    p.open = [](const std::string& path, gcs::JoystickInfo& info) {
        const int fd = ::open(path.c_str(), O_RDONLY | O_NONBLOCK);
        if (fd >= 0) info = {path, kName, 8, 4};
        return fd;
    };
    return p;
}

void write_event(int fd, std::uint8_t type, std::uint8_t number, std::int16_t value) {
    js_event e{0, value, type, number};
    CHECK(::write(fd, &e, sizeof(e)) == static_cast<ssize_t>(sizeof(e)));
}

struct Backend {
    explicit Backend(const std::string& fifo)
        : server(io, "127.0.0.1", 0, "/nonexistent"),
          link(io, gcs::LinkConfig{gcs::LinkConfig::Mode::kUdp, "127.0.0.1:9"},
               [this](const std::string& t, bool d) { server.broadcast(t, d); }),
          radio(io, [this](const std::string& t, bool d) { server.broadcast(t, d); }, fake_ports(fifo)) {
        server.set_link(&link);
        server.set_radio(&radio);
        CHECK(link.start());
        thread = std::thread([this] { io.run(); });
    }
    ~Backend() {
        io.stop();
        thread.join();
    }
    asio::io_context io;
    gcs::Server server;
    gcs::Link link;
    gcs::Radio radio;
    std::thread thread;
};

std::pair<int, std::string> request(unsigned short port, http::verb verb, const std::string& target, const std::string& body = "") {
    asio::io_context io;
    tcp::socket s(io);
    s.connect({asio::ip::make_address("127.0.0.1"), port});
    http::request<http::string_body> req{verb, target, 11};
    req.set(http::field::host, "127.0.0.1");
    req.body() = body;
    req.prepare_payload();
    http::write(s, req);
    beast::flat_buffer b;
    http::response<http::string_body> res;
    http::read(s, b, res);
    return {static_cast<int>(res.result_int()), res.body()};
}

struct Client {
    explicit Client(unsigned short port) : ws(io) {
        ws.next_layer().connect({asio::ip::make_address("127.0.0.1"), port});
        ws.handshake("127.0.0.1", "/ws");
        ws.text(true);
    }
    void send(const json::object& m) { ws.write(asio::buffer(json::serialize(m))); }
    json::object next() {
        beast::flat_buffer b;
        ws.read(b);
        return json::parse(beast::buffers_to_string(b.data())).as_object();
    }
    json::object wait(const char* type) {
        for (;;) {
            json::object m = next();
            if (m.at("type").as_string() == type) return m;
        }
    }
    // The first radio message (within 3 s of them) that satisfies ok; an empty object when none does.
    template <class F> json::object radio_until(F ok) {
        for (int i = 0; i < 60; ++i) {
            json::object m = wait("radio");
            if (ok(m)) return m;
        }
        return {};
    }
    asio::io_context io;
    websocket::stream<tcp::socket> ws;
};

double num(const json::value& v) { return v.to_number<double>(); }

}  // namespace

int main() {
    char tmpl[] = "/tmp/marv-gcs-radio-XXXXXX";
    const std::string root = ::mkdtemp(tmpl);
    ::setenv("XDG_CONFIG_HOME", root.c_str(), 1);
    const std::string fifo = root + "/js-fake";
    CHECK(::mkfifo(fifo.c_str(), 0600) == 0);
    int writer = ::open(fifo.c_str(), O_RDWR);  // a writer before the reader: no end of file
    CHECK(writer >= 0);
    const std::int16_t init[8] = {0, 0, -32767, 0, -32767, -10923, 0, 0};  // throttle bottom, CH5 off, CH6 freestyle
    for (std::uint8_t n = 0; n < 8; ++n) write_event(writer, JS_EVENT_AXIS | JS_EVENT_INIT, n, init[n]);
    for (std::uint8_t n = 0; n < 4; ++n) write_event(writer, JS_EVENT_BUTTON | JS_EVENT_INIT, n, 0);
    {
        Backend be(fifo);
        const unsigned short port = be.server.port();

        // 1. The device and its built-in config.
        auto [st, body] = request(port, http::verb::get, "/api/radio/devices");
        json::value v = json::parse(body);
        CHECK(st == 200 && v.as_array().size() == 1);
        const json::object& dev = v.as_array().at(0).as_object();
        CHECK(dev.at("path").as_string() == fifo && dev.at("name").as_string() == kName);
        CHECK(dev.at("axes").as_int64() == 8 && dev.at("buttons").as_int64() == 4 && !dev.at("has_config").as_bool());
        std::tie(st, body) = request(port, http::verb::get, "/api/radio/config" + kQuery);
        v = json::parse(body);
        CHECK(st == 200 && v.at("source").as_string() == "default" && v.at("device_name").as_string() == kName);
        CHECK(v.at("arm").at("source").as_string() == "axis" && v.at("profile").at("bands").as_array().size() == 4);
        std::tie(st, body) = request(port, http::verb::get, "/api/radio/config");
        CHECK(st == 400 && json::parse(body).at("error").as_string() == "device: missing");

        // 2. The stream.
        Client c(port);
        c.send({{"type", "radio_subscribe"}, {"device", "No Such Pad"}});
        json::object m = c.wait("error");
        CHECK(m.at("request").as_string() == "radio_subscribe");
        c.send({{"type", "radio_subscribe"}, {"device", kName}});
        m = c.wait("radio_devices");
        CHECK(m.at("devices").as_array().size() == 1);
        m = c.radio_until([](const json::object& r) { return r.at("axes").as_array().at(2).as_int64() == -32767; });
        CHECK(m.at("device").as_string() == kName);
        CHECK(m.at("axes").as_array().at(2).as_int64() == -32767 && m.at("buttons").as_array().size() == 4);
        CHECK(m.at("profile").as_string() == "freestyle" && !m.at("arm").as_bool());
        CHECK(num(m.at("normalized").at("throttle")) == -1.0 && num(m.at("normalized").at("roll")) == 0.0);
        write_event(writer, JS_EVENT_AXIS, 4, 32767);  // CH5 on, throttle at the bottom: armed
        write_event(writer, JS_EVENT_AXIS, 0, 32767);  // roll right
        write_event(writer, JS_EVENT_AXIS, 5, 32767);  // CH6 agile
        m = c.radio_until([](const json::object& r) { return r.at("arm").as_bool(); });
        CHECK(!m.empty() && m.at("profile").as_string() == "agile" && num(m.at("normalized").at("roll")) == 1.0);

        // 3. A stored config: the profile on buttons 3 (agile) and 2 (hold).
        json::object cfg = ground::to_json(ground::radiomaster_config(kName));
        cfg["source"] = "default";  // what GET gave: ignored
        cfg["profile"] = json::object{{"source", "buttons"}, {"buttons", json::object{{"3", "stabilized"}, {"2", "hold"}}}};
        std::tie(st, body) = request(port, http::verb::put, "/api/radio/config", json::serialize(cfg));
        CHECK(st == 200 && json::parse(body).at("source").as_string() == "stored");
        CHECK(std::filesystem::exists(root + "/marv/radio/edgetx-radiomaster-pocket-joystick.json"));
        std::tie(st, body) = request(port, http::verb::get, "/api/radio/devices");
        CHECK(json::parse(body).as_array().at(0).at("has_config").as_bool());
        std::tie(st, body) = request(port, http::verb::get, "/api/radio/config" + kQuery);
        v = json::parse(body);
        CHECK(v.at("source").as_string() == "stored" && v.at("profile").at("buttons").at("3").as_string() == "stabilized");
        m = c.radio_until([](const json::object& r) { return r.at("profile").as_string() == "hold"; });
        CHECK(!m.empty() && !m.at("arm").as_bool());  // reconfigured with CH5 on: not armed until it is cycled
        write_event(writer, JS_EVENT_BUTTON, 3, 1);
        m = c.radio_until([](const json::object& r) { return r.at("profile").as_string() == "stabilized"; });
        CHECK(!m.empty() && m.at("buttons").as_array().at(3).as_int64() == 1);
        write_event(writer, JS_EVENT_AXIS, 4, -32767);  // CH5 cycled, one period off (the pilot samples at 20 Hz)
        CHECK(!c.radio_until([](const json::object& r) { return r.at("axes").as_array().at(4).as_int64() == -32767; }).empty());
        write_event(writer, JS_EVENT_AXIS, 4, 32767);
        CHECK(!c.radio_until([](const json::object& r) { return r.at("arm").as_bool(); }).empty());

        json::object bad = cfg;
        bad["profile"] = json::object{{"source", "axis"}, {"index", 5},
                                      {"bands", json::array{json::object{{"upper", 0.5}, {"profile", "hold"}},
                                                            json::object{{"upper", 0.2}, {"profile", "agile"}},
                                                            json::object{{"upper", 1.0}, {"profile", "agile"}}}}};
        std::tie(st, body) = request(port, http::verb::put, "/api/radio/config", json::serialize(bad));
        CHECK(st == 400 && json::parse(body).at("error").as_string().starts_with("profile.bands[1].upper"));
        bad = cfg;
        bad["sticks"].as_object()["yaw"].as_object()["axis"] = 8;  // the device has 8 axes
        std::tie(st, body) = request(port, http::verb::put, "/api/radio/config", json::serialize(bad));
        CHECK(st == 400 && json::parse(body).at("error").as_string() == "sticks.yaw.axis: outside 0..7");
        std::tie(st, body) = request(port, http::verb::put, "/api/radio/config", "{\"version\":");
        CHECK(st == 400 && json::parse(body).at("error").as_string().starts_with("not JSON"));
        std::tie(st, body) = request(port, http::verb::post, "/api/radio/config", "{}");
        CHECK(st == 405);
        std::tie(st, body) = request(port, http::verb::get, "/api/radio/config" + kQuery);
        CHECK(json::parse(body).at("profile").at("source").as_string() == "buttons");  // refusals stored nothing

        std::tie(st, body) = request(port, http::verb::delete_, "/api/radio/config" + kQuery);
        CHECK(st == 200 && json::parse(body).at("source").as_string() == "default");
        CHECK(!std::filesystem::exists(root + "/marv/radio/edgetx-radiomaster-pocket-joystick.json"));
        m = c.radio_until([](const json::object& r) { return r.at("profile").as_string() == "agile"; });  // CH6 again
        CHECK(!m.empty());

        // 4. Hot-plug.
        g_plugged = false;
        ::close(writer);
        m = c.wait("radio_devices");
        CHECK(m.at("devices").as_array().empty());
        writer = ::open(fifo.c_str(), O_RDWR);
        g_plugged = true;
        m = c.wait("radio_devices");
        CHECK(m.at("devices").as_array().size() == 1);
        write_event(writer, JS_EVENT_AXIS, 1, -32767);
        m = c.radio_until([](const json::object& r) { return r.at("normalized").at("pitch").to_number<double>() == -1.0; });
        CHECK(!m.empty());  // the subscription outlived the unplug

        c.send({{"type", "radio_subscribe"}, {"device", nullptr}});
        c.send({{"type", "radio_subscribe"}, {"device", 3}});
        m = c.wait("error");
        CHECK(m.at("request").as_string() == "radio_subscribe");
    }
    ::close(writer);
    std::filesystem::remove_all(root);
    std::printf("%s (%d failures)\n", g_fails ? "FAIL" : "PASS", g_fails);
    return g_fails ? EXIT_FAILURE : EXIT_SUCCESS;
}
