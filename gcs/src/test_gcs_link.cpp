// The backend end to end: a scripted flight-controller stub speaking protocol.hpp over UDP loopback, marv_gcs's Link and
// Server on the same io_context thread, and a Beast WebSocket client checking the JSON replies.
//   1. request_setup / set_param (held value echoed, out of range refused) / save / load_factory / set_kind; a vehicle
//      change forwards the re-staged kinds, a kind that does not serve the vehicle is reported refused.
//   2. telemetry at 200 Hz from the stub reaches the client at <= 30 Hz.
//   3. no reply: the request is sent twice, 500 ms apart, then reported as "no reply".
//   4. a flight controller with another schema hash: edits refused before they reach the link.
#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <string>
#include <thread>
#include <vector>

#include <boost/asio/connect.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/json.hpp>

#include <marv/fsw/params.hpp>
#include <marv/fsw/presets.hpp>
#include <marv/link/protocol.hpp>

#include "link.hpp"
#include "schema.hpp"
#include "server.hpp"

namespace {

int g_fails = 0;
#define CHECK(c)                                                                  \
    do {                                                                          \
        if (!(c)) {                                                               \
            std::fprintf(stderr, "%s:%d: CHECK(%s) failed\n", __FILE__, __LINE__, #c); \
            ++g_fails;                                                            \
        }                                                                         \
    } while (0)

namespace asio = boost::asio;
namespace beast = boost::beast;
namespace http = beast::http;
namespace websocket = beast::websocket;
namespace json = boost::json;
using tcp = asio::ip::tcp;
using Clock = std::chrono::steady_clock;
using namespace marv;

// The flight controller's side of the setup exchange (gcs decision (b)), on its own thread.
class FcStub {
public:
    explicit FcStub(std::uint32_t schema_hash) : hash_(schema_hash) {
        fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
        sockaddr_in a{};
        a.sin_family = AF_INET;
        a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        ::bind(fd_, reinterpret_cast<const sockaddr*>(&a), sizeof(a));
        socklen_t len = sizeof(a);
        ::getsockname(fd_, reinterpret_cast<sockaddr*>(&a), &len);
        port_ = ntohs(a.sin_port);
        staged_ = stored_ = running_ = kFactory[0];
        thread_ = std::thread([this] { loop(); });
    }
    ~FcStub() {
        stop_ = true;
        thread_.join();
        ::close(fd_);
    }
    unsigned short port() const { return port_; }
    int received(std::uint8_t id) const { return count_[id]; }

    std::atomic<bool> silent{false};
    std::atomic<bool> telemetry{false};

private:
    template <class T> void put(std::vector<std::uint8_t>& out, const T& msg) {
        std::uint8_t f[link::kMaxFrame];
        const std::size_t n = link::encode(msg, f);
        out.insert(out.end(), f, f + n);
    }
    void header(std::vector<std::uint8_t>& out) {
        link::SetupHeader h{};
        h.schema_hash = hash_;
        h.param_count = param::kParamCount;
        for (int k = 0; k < param::kFamilyCount; ++k) h.kind[k] = staged_.kind[k];
        h.running_crc = param::setup_crc(running_);
        h.staged_crc = param::setup_crc(staged_);
        h.stored_crc = param::setup_crc(stored_);
        h.stored_valid = 1;
        h.armed = 0;
        put(out, h);
    }
    void values(std::vector<std::uint8_t>& out) {
        for (std::uint16_t i = 0; i < param::kParamCount; ++i) put(out, link::ParamValue{i, staged_.values[i]});
    }
    void handle(const link::Packet& p, std::vector<std::uint8_t>& out) {
        ++count_[p.id];
        if (silent) return;
        link::SetupRequest rq;
        link::SetParam sp;
        link::SetKind sk;
        link::SaveSetup sv;
        link::LoadFactory lf;
        link::Reset rs;
        if (p.as(rq)) {
            header(out);
            values(out);
        } else if (p.as(sp)) {
            const bool ok = sp.index < param::kParamCount && sp.value >= param::kParamMeta[sp.index].min &&
                            sp.value <= param::kParamMeta[sp.index].max;
            if (ok) staged_.values[sp.index] = sp.value;
            put(out, link::ParamValue{sp.index, sp.index < param::kParamCount ? staged_.values[sp.index] : 0.f});
        } else if (p.as(sk)) {
            // dispatch.hpp's class rules: a vehicle change re-stages the families that do not serve it; a kind that
            // does not serve the staged vehicle is refused, and the header echoes the held kind.
            if (sk.family == param::k_vehicle && sk.kind < param::kind_count(sk.family)) {
                staged_.kind[sk.family] = sk.kind;
                for (std::uint8_t f = 0; f < param::kFamilyCount; ++f)
                    if (!param::compatible(f, staged_.kind[f], sk.kind)) staged_.kind[f] = param::first_compatible(f, sk.kind);
            } else if (sk.family < param::kFamilyCount && sk.kind < param::kind_count(sk.family) &&
                       param::compatible(sk.family, sk.kind, staged_.kind[param::k_vehicle])) {
                staged_.kind[sk.family] = sk.kind;
            }
            header(out);
        } else if (p.as(sv)) {
            stored_ = staged_;
            header(out);
        } else if (p.as(lf)) {
            if (lf.id < kPresetCount) staged_ = kFactory[lf.id];
            header(out);
            values(out);
        } else if (p.as(rs)) {
            running_ = staged_;
            header(out);
        }
    }
    void loop() {
        link::Decoder dec;
        sockaddr_in peer{};
        bool have_peer = false;
        auto next_tlm = Clock::now();
        while (!stop_) {
            pollfd pfd{fd_, POLLIN, 0};
            ::poll(&pfd, 1, 1);
            std::vector<std::uint8_t> out;
            if (pfd.revents & POLLIN) {
                std::uint8_t buf[2048];
                sockaddr_in from{};
                socklen_t len = sizeof(from);
                const ssize_t n = ::recvfrom(fd_, buf, sizeof(buf), 0, reinterpret_cast<sockaddr*>(&from), &len);
                if (n > 0) {
                    peer = from;
                    have_peer = true;
                    for (ssize_t i = 0; i < n; ++i)
                        if (dec.push(buf[i])) handle(dec.packet(), out);
                }
            }
            if (telemetry && have_peer && Clock::now() >= next_tlm) {
                next_tlm = Clock::now() + std::chrono::milliseconds(5);
                Telemetry t{};
                t.est.q = {1.f, 0.f, 0.f, 0.f};
                t.est.valid = true;
                t.preset = 0;
                put(out, t);
            }
            if (!out.empty() && have_peer)
                ::sendto(fd_, out.data(), out.size(), 0, reinterpret_cast<const sockaddr*>(&peer), sizeof(peer));
        }
    }

    std::uint32_t hash_;
    int fd_;
    unsigned short port_;
    param::Setup staged_{}, stored_{}, running_{};
    std::atomic<int> count_[256] = {};
    std::atomic<bool> stop_{false};
    std::thread thread_;
};

// marv_gcs's Link and Server on an io_context thread, linked to a stub.
struct Backend {
    explicit Backend(unsigned short fc_port)
        : server(io, "127.0.0.1", 0, "/nonexistent"),
          link(io, gcs::LinkConfig{false, "127.0.0.1:" + std::to_string(fc_port)},
               [this](const std::string& t, bool d) { server.broadcast(t, d); }) {
        server.set_link(&link);
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
    std::thread thread;
};

std::string http_get(unsigned short port, const std::string& target) {
    asio::io_context io;
    tcp::socket s(io);
    s.connect({asio::ip::make_address("127.0.0.1"), port});
    http::request<http::empty_body> req{http::verb::get, target, 11};
    req.set(http::field::host, "127.0.0.1");
    http::write(s, req);
    beast::flat_buffer b;
    http::response<http::string_body> res;
    http::read(s, b, res);
    return res.body();
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
    // The next message of this type; others (telemetry, link updates) are skipped.
    json::object wait(const char* type) {
        for (;;) {
            json::object m = next();
            if (m.at("type").as_string() == type) return m;
        }
    }
    asio::io_context io;
    websocket::stream<tcp::socket> ws;
};

float f32(const json::value& v) { return static_cast<float>(v.to_number<double>()); }
std::uint64_t u64(const json::value& v) { return v.to_number<std::uint64_t>(); }

bool values_equal(const json::object& setup, const param::Setup& s) {
    const json::value* v = setup.if_contains("values");
    if (!v || v->as_array().size() != param::kParamCount) return false;
    for (std::size_t i = 0; i < param::kParamCount; ++i)
        if (f32(v->as_array()[i]) != s.values[i]) return false;
    return true;
}

void test_exchange() {
    FcStub fc(param::kSchemaHash);
    Backend be(fc.port());
    Client c(be.server.port());
    const json::object hello = c.wait("link");
    CHECK(hello.at("mode").as_string() == "sitl-udp");

    c.send({{"type", "request_setup"}});
    json::object m = c.wait("setup");
    CHECK(u64(m.at("header").at("schema_hash")) == param::kSchemaHash);
    CHECK(u64(m.at("header").at("param_count")) == param::kParamCount);
    CHECK(m.at("schema_ok").as_bool());
    CHECK(values_equal(m, kFactory[0]));

    const std::uint16_t hover = param::k_vehicle_uav_hover_thrust;
    c.send({{"type", "set_param"}, {"index", hover}, {"value", 0.7}});
    m = c.wait("param");
    CHECK(u64(m.at("index")) == hover && f32(m.at("value")) == 0.7f);
    c.send({{"type", "set_param"}, {"index", hover}, {"value", 100.0}});  // above max: the FC holds 0.7
    m = c.wait("param");
    CHECK(u64(m.at("index")) == hover && f32(m.at("value")) == 0.7f);

    c.send({{"type", "save"}});
    m = c.wait("setup");
    CHECK(u64(m.at("header").at("stored_crc")) == u64(m.at("header").at("staged_crc")));
    CHECK(!m.contains("values"));
    CHECK(u64(m.at("header").at("staged_crc")) != param::setup_crc(kFactory[0]));

    c.send({{"type", "load_factory"}, {"id", 2}});
    m = c.wait("setup");
    CHECK(u64(m.at("header").at("kind").at(param::k_estimator)) == param::k_estimator_mahony);
    CHECK(values_equal(m, kFactory[2]));
    CHECK(u64(m.at("header").at("staged_crc")) == param::setup_crc(kFactory[2]));

    c.send({{"type", "set_kind"}, {"family", static_cast<int>(param::k_estimator)}, {"kind", param::k_estimator_ekf}});
    m = c.wait("setup");
    CHECK(u64(m.at("header").at("kind").at(param::k_estimator)) == param::k_estimator_ekf);

    // A vehicle change: the stub re-stages the families that do not serve a rocket; the header carries them.
    c.send({{"type", "set_kind"}, {"family", static_cast<int>(param::k_vehicle)}, {"kind", param::k_vehicle_rocket}});
    m = c.wait("setup");
    for (std::uint8_t f = 0; f < param::kFamilyCount; ++f) {
        const std::uint64_t k = u64(m.at("header").at("kind").at(f));
        CHECK(param::compatible(f, static_cast<std::uint8_t>(k), param::k_vehicle_rocket));
    }
    CHECK(u64(m.at("header").at("kind").at(param::k_vehicle)) == param::k_vehicle_rocket);
    CHECK(u64(m.at("header").at("kind").at(param::k_estimator)) == param::k_estimator_ekf);  // serves both: kept
    CHECK(u64(m.at("header").at("kind").at(param::k_guidance)) == param::k_guidance_apogee_predictor);
    CHECK(u64(m.at("header").at("kind").at(param::k_allocation)) == param::k_allocation_rocket_brake);

    // A kind that does not serve the rocket: the header echoes the held kind, and the requester gets the refusal.
    c.send({{"type", "set_kind"}, {"family", static_cast<int>(param::k_guidance)}, {"kind", param::k_guidance_passthrough}});
    m = c.wait("setup");
    CHECK(u64(m.at("header").at("kind").at(param::k_guidance)) == param::k_guidance_apogee_predictor);
    m = c.wait("error");
    CHECK(m.at("request").as_string() == "set_kind");
    CHECK(u64(m.at("family")) == param::k_guidance);
    CHECK(m.at("error").as_string().find("does not serve the rocket") != std::string::npos);

    c.send({{"type", "load_factory"}, {"id", 0}});
    m = c.wait("setup");
    CHECK(u64(m.at("header").at("kind").at(param::k_vehicle)) == param::k_vehicle_uav);

    c.send({{"type", "set_param"}, {"index", param::kParamCount}, {"value", 1.0}});  // bad index: refused here
    m = c.wait("error");
    CHECK(m.at("request").as_string() == "set_param");
    CHECK(fc.received(link::kSetParam) == 2);

    // Telemetry: 200 Hz in, at most 30 Hz out.
    fc.telemetry = true;
    int n = 0;
    const auto until = Clock::now() + std::chrono::seconds(1);
    while (Clock::now() < until) {
        m = c.next();
        if (m.at("type").as_string() == "telemetry") {
            ++n;
            CHECK(f32(m.at("est").at("q").at(0)) == 1.f);
        }
    }
    fc.telemetry = false;
    std::printf("telemetry: %d messages in 1 s\n", n);
    CHECK(n >= 15 && n <= 31);

    // No reply: sent, sent again after 500 ms, then reported.
    fc.silent = true;
    const int saves = fc.received(link::kSaveSetup);
    const auto t0 = Clock::now();
    c.send({{"type", "save"}});
    m = c.wait("error");
    const double dt = std::chrono::duration<double>(Clock::now() - t0).count();
    CHECK(m.at("request").as_string() == "save" && m.at("error").as_string() == "no reply");
    CHECK(fc.received(link::kSaveSetup) - saves == 2);
    CHECK(dt >= 0.95 && dt < 1.5);
    std::printf("no reply after %.3f s, %d sends\n", dt, fc.received(link::kSaveSetup) - saves);
    fc.silent = false;

    // flash over UDP: explained, not attempted.
    c.send({{"type", "flash"}});
    m = c.wait("flash_log");
    CHECK(m.at("line").as_string().find("--serial") != std::string::npos);

    // HTTP.
    CHECK(http_get(be.server.port(), "/api/schema") == gcs::schema_text());
    const json::value link = json::parse(http_get(be.server.port(), "/api/link"));
    CHECK(link.at("mode").as_string() == "sitl-udp");
}

void test_mismatch() {
    const std::uint32_t other = param::kSchemaHash ^ 0x5a5a5a5au;
    FcStub fc(other);
    Backend be(fc.port());
    Client c(be.server.port());
    c.wait("link");

    // Edits before any header: refused.
    c.send({{"type", "save"}});
    CHECK(c.wait("error").at("request").as_string() == "save");

    c.send({{"type", "request_setup"}});
    json::object m = c.wait("setup");
    CHECK(u64(m.at("header").at("schema_hash")) == other);
    CHECK(!m.at("schema_ok").as_bool());
    CHECK(m.at("error").as_string().find("schema mismatch") != std::string::npos);

    const json::object edits[] = {
        {{"type", "set_param"}, {"index", 0}, {"value", 1.7}},
        {{"type", "set_kind"}, {"family", 2}, {"kind", 1}},
        {{"type", "load_factory"}, {"id", 1}},
        {{"type", "save"}},
    };
    for (const json::object& e : edits) {
        c.send(e);
        m = c.wait("error");
        CHECK(m.at("request") == e.at("type"));
        CHECK(m.at("error").as_string().find("schema mismatch") != std::string::npos);
    }
    CHECK(fc.received(link::kSetParam) == 0 && fc.received(link::kSetKind) == 0 &&
          fc.received(link::kLoadFactory) == 0 && fc.received(link::kSaveSetup) == 0);

    const json::value link = json::parse(http_get(be.server.port(), "/api/link"));
    CHECK(link.at("schema_ok") == false);
    CHECK(link.at("connected") == true);
    CHECK(link.at("error").as_string().find("schema mismatch") != std::string::npos);
}

}  // namespace

int main() {
    ::alarm(60);  // a lost reply fails the test instead of hanging it
    test_exchange();
    test_mismatch();
    if (g_fails) std::fprintf(stderr, "test_gcs_link: %d failures\n", g_fails);
    else std::printf("test_gcs_link: OK\n");
    return g_fails ? 1 : 0;
}
