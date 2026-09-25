#include "link.hpp"

#include <fcntl.h>
#include <glob.h>
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <type_traits>
#include <utility>

#include <boost/asio/buffers_iterator.hpp>
#include <boost/asio/read_until.hpp>
#include <boost/json.hpp>

#include <marv/fsw/geo.hpp>

#include "schema.hpp"

namespace marv::gcs {
namespace asio = boost::asio;
namespace json = boost::json;
using namespace std::chrono_literals;

namespace {

constexpr auto kPoll = 2ms;
constexpr auto kReplyTimeout = 500ms;
constexpr auto kKeepalive = 500ms;       // a lone 0x00 at least this often, so the bridge keeps sending to us
constexpr auto kTelemetryPeriod = 34ms;  // at most 30 Hz to the clients
constexpr auto kReopen = 2s;
constexpr auto kScan = 1s;               // kAuto: how often to look for a flight controller on USB
constexpr auto kMissionPeriod = 50ms;    // 20 Hz MissionCommand frames while engaged
constexpr auto kMissionState = 500ms;    // mission_state at least this often while engaged
constexpr std::size_t kMaxQueue = 256;
constexpr std::size_t kMaxValues = 4096;
constexpr const char* kRigLock = "/tmp/marv-rig.lock";
constexpr const char* kFcGlob = "/dev/serial/by-id/usb-MARV_MARV_flight_controller_*";

// The firmware build, then the SWD flash line of CLAUDE.md; run from the repository root under the rig lock.
constexpr const char* kFlashScript =
    "cmake --build build/fw && \"$HOME/pico/openocd-install/bin/openocd\" "
    "-s \"$HOME/pico/openocd-install/share/openocd/scripts\" -f interface/cmsis-dap.cfg -f target/rp2350.cfg "
    "-c \"adapter speed 5000\" -c \"program build/fw/marv_fw.elf verify reset exit\"";

std::string hex(std::uint32_t v) {
    char b[16];
    std::snprintf(b, sizeof(b), "0x%08x", static_cast<unsigned>(v));
    return b;
}

json::array vec3(const Vec3& v) { return {fnum(v.x), fnum(v.y), fnum(v.z)}; }
json::array quat(const Quat& q) { return {fnum(q.w), fnum(q.x), fnum(q.y), fnum(q.z)}; }

json::object header_json(const link::SetupHeader& h) {
    json::array kind;
    for (std::uint8_t k : h.kind) kind.push_back(k);
    return {{"schema_hash", h.schema_hash},     {"param_count", h.param_count},   {"kind", std::move(kind)},
            {"running_crc", h.running_crc},     {"staged_crc", h.staged_crc},     {"stored_crc", h.stored_crc},
            {"stored_valid", h.stored_valid != 0}, {"armed", h.armed != 0}};
}

// A finite JSON number member.
bool number(const json::object& m, const char* key, double& out) {
    const json::value* v = m.if_contains(key);
    if (!v) return false;
    if (v->is_double()) out = v->get_double();
    else if (v->is_int64()) out = static_cast<double>(v->get_int64());
    else if (v->is_uint64()) out = static_cast<double>(v->get_uint64());
    else return false;
    return std::isfinite(out);
}

// An integral JSON number member in [lo, hi].
bool integer(const json::object& m, const char* key, long lo, long hi, long& out) {
    double d;
    if (!number(m, key, d) || d != std::floor(d) || d < static_cast<double>(lo) || d > static_cast<double>(hi))
        return false;
    out = static_cast<long>(d);
    return true;
}

std::uint8_t u8(long v) { return static_cast<std::uint8_t>(v); }

double seconds(std::chrono::steady_clock::time_point t) { return std::chrono::duration<double>(t.time_since_epoch()).count(); }

// {waypoints: [{lat, lon, alt_m}]}: false when it is not that shape.
bool waypoints(const json::object& m, std::vector<Waypoint>& out) {
    const json::value* v = m.if_contains("waypoints");
    if (!v || !v->is_array()) return false;
    for (const json::value& w : v->get_array()) {
        Waypoint p{};
        if (!w.is_object() || !number(w.get_object(), "lat", p.lat) || !number(w.get_object(), "lon", p.lon) ||
            !number(w.get_object(), "alt_m", p.alt_m))
            return false;
        out.push_back(p);
    }
    return true;
}

// The wire name of a flight profile ("#p" when out of range).
std::string profile_name(std::uint8_t p) {
    return p < param::kProfileCount ? std::string(param::kProfileId[p]) : "#" + std::to_string(p);
}

// {profile: "hold" | "freestyle" | "stabilized" | "agile"}: false when absent or not one of them.
bool profile(const json::object& m, std::uint8_t& out) {
    const json::value* v = m.if_contains("profile");
    if (!v || !v->is_string()) return false;
    for (std::uint8_t p = 0; p < param::kProfileCount; ++p)
        if (v->get_string() == param::kProfileId[p]) {
            out = p;
            return true;
        }
    return false;
}

json::object mission_state(const Mission::Status& s) {
    json::object o{{"type", "mission_state"}, {"state", Mission::name(s.state)}, {"wp_index", s.wp_index},
                   {"wp_count", s.wp_count},  {"target", nullptr},                 {"dist_m", nullptr},
                   {"climb_alt_m", nullptr},  {"home", nullptr},                   {"reason", s.reason},
                   {"profile", profile_name(s.profile)}};
    if (s.has_target) {
        o["target"] = json::object{
            {"lat", s.target.lat}, {"lon", s.target.lon}, {"alt_m", fnum(static_cast<float>(s.target.alt_m))}};
        o["dist_m"] = fnum(s.dist_m);
    }
    if (s.has_climb_alt) o["climb_alt_m"] = fnum(s.climb_alt_m);
    if (s.has_home) o["home"] = json::object{{"lat", s.home_lat}, {"lon", s.home_lon}};
    return o;
}

// The wire name of a kind of a family ("#k" when out of range).
std::string kind_name(std::uint8_t family, std::uint8_t kind) {
    if (family >= param::kFamilyCount || kind >= param::kind_count(family)) return "#" + std::to_string(kind);
    std::size_t row = kind;
    for (std::uint8_t f = 0; f < family; ++f) row += param::kind_count(f);
    return param::kKindName[row];
}

}  // namespace

std::string first_fc() {
    std::string path;
    glob_t g{};
    if (::glob(kFcGlob, 0, nullptr, &g) == 0 && g.gl_pathc > 0) path = g.gl_pathv[0];
    ::globfree(&g);
    return path;
}

Holder fc_holder(const std::string& device) { return device_holder("/proc", device, ::getpid(), ::getuid()); }

Link::Link(asio::io_context& io, LinkConfig cfg, Broadcast broadcast)
    : cfg_(std::move(cfg)), broadcast_(std::move(broadcast)), timer_(io), flash_out_(io) {}

Link::~Link() {
    timer_.cancel();
    if (flash_pid_ > 0) {
        ::kill(flash_pid_, SIGTERM);
        ::waitpid(flash_pid_, nullptr, 0);
    }
}

bool Link::start() {
    if (!open() && cfg_.mode == LinkConfig::Mode::kUdp) return false;
    if (!tx_) reopen_at_ = Clock::now() + (cfg_.mode == LinkConfig::Mode::kAuto ? kScan : kReopen);  // it may appear later
    else if (serial_) enqueue(Kind::kSetup, "request_setup", link::SetupRequest{}, {});
    tick();
    return true;
}

// Opens the transport the mode calls for now: false when there is none (kAuto: no flight controller on USB yet).
bool Link::open() {
    const bool automatic = cfg_.mode == LinkConfig::Mode::kAuto;
    const bool serial = cfg_.mode == LinkConfig::Mode::kSerial || (automatic && !sim_);
    const std::string at = automatic && serial ? cfg_.scan() : cfg_.target;
    // kAuto: a flight controller another process has open is not opened (its bytes would be split between the two).
    Holder held;
    bool busy = false;
    if (!at.empty() && automatic && serial && cfg_.holder) {
        held = cfg_.holder(at);
        busy = held.pid > 0;
    }
    if (!at.empty() && !busy) {
        errno = 0;
        tx_ = serial ? cfg_.open_serial(at.c_str()) : cfg_.open_udp(at.c_str());
        busy = !tx_ && automatic && serial && errno == EBUSY;
        if (busy) held.name = "another process";
    }
    if ((busy ? at : std::string()) != busy_at_ || held.pid != busy_.pid) {
        busy_at_ = busy ? at : std::string();
        busy_ = busy ? held : Holder{};
        if (busy) std::fprintf(stderr, "marv_gcs: link: %s is held by %s (pid %ld): not opened\n", at.c_str(), held.name.c_str(), held.pid);
        if (!tx_) broadcast_link();
    }
    if (!tx_) return false;
    serial_ = serial;
    at_ = at;
    dec_ = link::Decoder{};
    if (automatic)
        std::fprintf(stderr, "marv_gcs: link: %s %s\n", serial ? "flight controller via USB on" : "sim bridge at", at.c_str());
    return true;
}

void Link::sim(bool on) {
    if (cfg_.mode != LinkConfig::Mode::kAuto || on == sim_) return;
    sim_ = on;
    const auto now = Clock::now();
    if (tx_) std::fprintf(stderr, "marv_gcs: link: closing %s: %s\n", at_.c_str(), on ? "a sim is starting" : "the sim ended");
    // Another vehicle from here on: stop the executor rather than fly it on the next one, its kIdle out on this link
    // before it closes; and forget this one's setup.
    if (mission_.engaged(seconds(now))) {
        mission_.disarm(seconds(now));
        mission_tick(now);
    }
    tx_.reset();  // before the launcher starts the bridge: target fc needs this port
    have_header_ = have_armed_ = false;
    fail_all(on ? "link switched to the sim bridge" : "the sim ended: link back to USB");
    reopen_at_ = now;
    if (on && !open()) reopen_at_ = now + kReopen;
    broadcast_link();
}

// ---- client requests ---------------------------------------------------------------------------------------------------

void Link::handle(const json::object& m, const Reply& reply) {
    const json::value* t = m.if_contains("type");
    const std::string type = t && t->is_string() ? std::string(t->get_string()) : std::string();
    if (type == "request_setup") return enqueue(Kind::kSetup, type, link::SetupRequest{}, reply);
    if (type == "reset") return enqueue(Kind::kReset, type, link::Reset{}, reply);
    if (type == "reboot") {
        // No reply: the flight controller restarts (a Pico drops off USB). The setup is asked for when it is heard again.
        std::uint8_t f[link::kMaxFrame];
        if (!send(f, link::encode(link::Reboot{}, f))) return error(reply, type, "link not open");
        next_probe_ = Clock::now() + 1s;
        if (serial_) {
            // The Pico comes back as a new tty; a read on the old one reports nothing, not a hangup. Reopen by path.
            tx_.reset();
            reopen_at_ = Clock::now() + kReopen;
        }
        return set_connected(false);
    }
    if (type == "flash") return flash(reply);
    if (type == "arm" || type == "disarm" || type == "climb" || type == "mission_start" || type == "rth" || type == "land" ||
        type == "profile")
        return mission_request(type, m, reply);
    if (type != "set_param" && type != "set_kind" && type != "load_factory" && type != "save")
        return error(reply, type, "unknown request");

    const std::string refusal = edit_refusal();
    if (!refusal.empty()) return error(reply, type, refusal);
    if (type == "save") return enqueue(Kind::kSave, type, link::SaveSetup{}, reply);
    if (type == "set_param") {
        long index;
        double value;
        if (!integer(m, "index", 0, param::kParamCount - 1, index) || !number(m, "value", value))
            return error(reply, type, "want {index: 0.." + std::to_string(param::kParamCount - 1) + ", value: number}");
        const link::SetParam s{static_cast<std::uint16_t>(index), static_cast<float>(value)};
        return enqueue(Kind::kSetParam, type, s, reply, s.index);
    }
    if (type == "set_kind") {
        long family, kind;
        if (!integer(m, "family", 0, param::kFamilyCount - 1, family) || !integer(m, "kind", 0, 255, kind))
            return error(reply, type, "want {family: 0.." + std::to_string(param::kFamilyCount - 1) + ", kind: 0..255}");
        return enqueue(Kind::kSetKind, type, link::SetKind{u8(family), u8(kind)}, reply);
    }
    long id;
    if (!integer(m, "id", 0, 255, id)) return error(reply, type, "want {id: 0..255}");
    enqueue(Kind::kSetup, type, link::LoadFactory{u8(id)}, reply);
}

std::string Link::edit_refusal() const {
    if (!have_header_) return "no setup header from the flight controller yet";
    if (header_.schema_hash != param::kSchemaHash)
        return "schema mismatch: flight controller " + hex(header_.schema_hash) + ", GCS " + hex(param::kSchemaHash) +
               ": edits refused";
    return {};
}

template <class T>
void Link::enqueue(Kind kind, const std::string& name, const T& msg, const Reply& reply, std::uint16_t index) {
    if (queue_.size() >= kMaxQueue) return error(reply, name, "busy: too many requests waiting");
    Request r{kind, name, std::vector<std::uint8_t>(link::kMaxFrame), index, reply};
    r.frame.resize(link::encode(msg, r.frame.data()));
    if constexpr (std::is_same_v<T, link::SetKind>) r.set = msg;
    queue_.push_back(std::move(r));
    if (queue_.size() == 1) issue();
}

// Sends the head request, or fails every request while the link is closed.
void Link::issue() {
    if (queue_.empty()) return;
    if (!tx_) return fail_all("link not open");
    tries_ = 1;
    transmit();
}

void Link::transmit() {
    const Request& r = queue_.front();
    rx_header_ = false;
    rx_count_ = 0;
    deadline_ = Clock::now() + kReplyTimeout;
    send(r.frame.data(), r.frame.size());
}

void Link::complete() {
    queue_.pop_front();
    issue();
}

void Link::fail(const std::string& why) {
    if (verify_flash_) {
        verify_flash_ = false;
        flash_log("flash: the flight controller did not answer the setup request");
    }
    fail_all(why);
}

void Link::fail_all(const std::string& why) {
    std::deque<Request> q;
    q.swap(queue_);
    for (const Request& r : q) error(r.reply, r.name, why);
    set_connected(false);
}

// ---- the link ----------------------------------------------------------------------------------------------------------

void Link::tick() {
    timer_.expires_after(kPoll);
    timer_.async_wait([this](const boost::system::error_code& ec) {
        if (ec) return;
        poll();
        tick();
    });
}

void Link::poll() {
    const auto now = Clock::now();
    if (!tx_ && flash_pid_ < 0 && now >= reopen_at_) {
        if (!open()) {
            reopen_at_ = now + (cfg_.mode == LinkConfig::Mode::kAuto ? kScan : kReopen);
        } else {
            broadcast_link();
            enqueue(Kind::kSetup, "request_setup", link::SetupRequest{}, {});
        }
    }
    for (int i = 0; tx_ && i < 64; ++i) {
        std::uint8_t buf[4096];
        const long n = tx_->recv(buf, sizeof(buf), 0);
        if (n < 0) {
            close("link lost");
            break;
        }
        if (n == 0) break;
        last_rx_ = now;
        for (long k = 0; k < n; ++k)
            if (dec_.push(buf[k])) receive(dec_.packet());
    }
    if (tx_ && !serial_ && now - last_tx_ >= kKeepalive) {
        const std::uint8_t delim = 0;
        send(&delim, 1);
    }
    if (!queue_.empty() && now >= deadline_) {
        if (tries_ < 2) {
            ++tries_;
            transmit();
        } else {
            fail("no reply");
        }
    }
    if (now >= next_mission_) {
        next_mission_ = now + kMissionPeriod;
        mission_tick(now);
    }
    if (tlm_pending_ && now - last_tlm_ >= kTelemetryPeriod) {
        last_tlm_ = now;
        tlm_pending_ = false;
        broadcast_(telemetry_message(), true);
    }
}

bool Link::send(const std::uint8_t* p, std::size_t n) {
    if (!tx_) return false;
    last_tx_ = Clock::now();
    return tx_->send(p, n);
}

void Link::close(const char* why) {
    std::fprintf(stderr, "marv_gcs: %s: %s\n", at_.c_str(), why);
    tx_.reset();
    reopen_at_ = Clock::now() + kReopen;
    fail_all(why);
    broadcast_link();
}

void Link::set_connected(bool connected) {
    if (connected == connected_) return;
    connected_ = connected;
    broadcast_link();
}

void Link::receive(const link::Packet& p) {
    link::SetupHeader h;
    link::ParamValue v;
    Telemetry t;
    ActuatorCommand a;
    if (p.as(h)) return on_header(h);
    if (p.as(v)) return on_value(v);
    const auto now = Clock::now();
    if (p.as(t)) {
        tlm_ = t;
        tlm_pending_ = true;
        mission_.telemetry(t, have_armed_ ? armed_ : have_header_ && header_.armed != 0, seconds(now));
    } else if (p.as(a)) {
        armed_ = a.armed;
        have_armed_ = true;
        std::copy(a.motor, a.motor + kMotorCount, motor_);
    }
    // Heard, but not connected: ask for the setup (after a reboot, or a bridge started after us).
    if (!connected_ && queue_.empty() && now >= next_probe_) {
        next_probe_ = now + 2s;
        enqueue(Kind::kSetup, "request_setup", link::SetupRequest{}, {});
    }
}

void Link::on_header(const link::SetupHeader& h) {
    const bool changed = !have_header_ || !connected_ || h.schema_hash != header_.schema_hash;
    header_ = h;
    have_header_ = true;
    connected_ = true;
    if (changed) broadcast_link();
    if (!queue_.empty() && !rx_header_) {
        switch (queue_.front().kind) {
        case Kind::kSetup:
            rx_header_ = true;
            rx_values_.assign(std::min<std::size_t>(h.param_count, kMaxValues), std::numeric_limits<float>::quiet_NaN());
            rx_count_ = 0;
            deadline_ = Clock::now() + kReplyTimeout;
            if (rx_values_.empty()) on_value({0, 0.f});
            return;
        case Kind::kSetKind: {
            broadcast_(setup_message(false), false);
            const Request& r = queue_.front();
            const std::string refusal = kind_refusal(r.set, h);
            if (!refusal.empty() && r.reply)
                r.reply(json::serialize(json::object{
                    {"type", "error"}, {"request", r.name}, {"family", r.set.family}, {"error", refusal}}));
            return complete();
        }
        case Kind::kSave:
            broadcast_(setup_message(false), false);
            return complete();
        case Kind::kReset:
            // The flight controller refuses a reset while armed: the header comes back armed, running != staged.
            broadcast_(setup_message(false), false);
            if (h.armed != 0)
                error(queue_.front().reply, queue_.front().name, "refused while armed: the flight software keeps running");
            return complete();
        case Kind::kSetParam:
            break;
        }
    }
    broadcast_(setup_message(false), false);
}

void Link::on_value(const link::ParamValue& v) {
    const auto param = [&v] {
        return json::serialize(json::object{{"type", "param"}, {"index", v.index}, {"value", fnum(v.value)}});
    };
    if (!queue_.empty()) {
        const Request& r = queue_.front();
        if (r.kind == Kind::kSetup && rx_header_) {
            if (v.index < rx_values_.size()) rx_values_[v.index] = v.value;
            deadline_ = Clock::now() + kReplyTimeout;
            if (++rx_count_ < rx_values_.size()) return;
            values_ = rx_values_;
            broadcast_(setup_message(true), false);
            if (verify_flash_) {
                verify_flash_ = false;
                const std::string refusal = edit_refusal();
                flash_log(refusal.empty() ? "flash: setup answered, schema hash " + hex(header_.schema_hash) + " matches"
                                          : "flash: " + refusal);
            }
            return complete();
        }
        if (r.kind == Kind::kSetParam && v.index == r.index) {
            if (v.index < values_.size()) values_[v.index] = v.value;
            broadcast_(param(), false);
            return complete();
        }
    }
    if (v.index < values_.size()) values_[v.index] = v.value;
    broadcast_(param(), false);
}

// ---- messages to the clients -------------------------------------------------------------------------------------------

json::object Link::state() const {
    const LinkConfig::Mode mode = cfg_.mode;
    json::value via = nullptr, target = nullptr;
    if (tx_) {
        via = serial_ ? "usb" : "sim-bridge";
        target = at_;
    } else if (!busy_at_.empty()) {
        via = "busy";
        target = busy_at_;
    } else if (mode != LinkConfig::Mode::kAuto) {
        target = cfg_.target;
    }
    json::object o{{"mode", mode == LinkConfig::Mode::kAuto ? "auto" : mode == LinkConfig::Mode::kUdp ? "udp" : "serial"},
                   {"via", std::move(via)},
                   {"target", std::move(target)},
                   {"open", tx_ != nullptr},
                   {"connected", connected_},
                   {"schema_hash", param::kSchemaHash},
                   {"flashing", flash_pid_ > 0}};
    if (!tx_ && !busy_at_.empty()) o["holder"] = json::object{{"name", busy_.name}, {"pid", busy_.pid}};
    if (have_header_) {
        o["schema_ok"] = header_.schema_hash == param::kSchemaHash;
        o["header"] = header_json(header_);
        const std::string refusal = edit_refusal();
        if (!refusal.empty()) o["error"] = refusal;
    } else {
        o["schema_ok"] = nullptr;
        o["header"] = nullptr;
    }
    return o;
}

std::string Link::link_message() const {
    json::object m{{"type", "link"}};
    for (const auto& kv : state()) m[kv.key()] = kv.value();
    return json::serialize(m);
}

std::string Link::setup_message(bool with_values) const {
    json::object m{{"type", "setup"},
                   {"header", header_json(header_)},
                   {"schema_ok", header_.schema_hash == param::kSchemaHash}};
    const std::string refusal = edit_refusal();
    if (!refusal.empty()) m["error"] = refusal;
    if (with_values) {
        json::array a;
        for (float v : values_) a.push_back(fnum(v));
        m["values"] = std::move(a);
    }
    return json::serialize(m);
}

std::string Link::telemetry_message() const {
    const State& e = tlm_.est;
    const bool armed = have_armed_ ? armed_ : have_header_ && header_.armed != 0;
    json::value geo = nullptr;
    if (tlm_.home_valid && e.valid) {
        LocalFrame f;
        f.set(tlm_.home);
        const GeoPoint g = f.to_geo(e.p_ned);
        geo = json::object{{"lat", static_cast<double>(g.lat_e7) * 1e-7},
                           {"lon", static_cast<double>(g.lon_e7) * 1e-7},
                           {"alt_m", fnum(-e.p_ned.z)}};
    }
    json::array motor;
    for (float m : motor_) motor.push_back(fnum(m));
    return json::serialize(json::object{
        {"type", "telemetry"},
        {"t_us", tlm_.t_us},
        {"preset", tlm_.preset},
        {"profile", profile_name(tlm_.profile)},
        {"armed", armed},
        {"est", json::object{{"t_us", e.t_us}, {"p_ned", vec3(e.p_ned)}, {"v_ned", vec3(e.v_ned)}, {"q", quat(e.q)},
                             {"w_frd", vec3(e.w_frd)}, {"valid", e.valid}}},
        {"thrust_ned", vec3(tlm_.req.thrust_ned)},
        {"torque_frd", vec3(tlm_.req.torque_frd)},
        {"thrust_hover", fnum(tlm_.req.thrust_hover)},
        {"brake", fnum(tlm_.req.brake)},
        {"home_valid", tlm_.home_valid},
        {"home", json::object{{"lat_e7", tlm_.home.lat_e7}, {"lon_e7", tlm_.home.lon_e7}, {"alt_m", fnum(tlm_.home.alt_m)}}},
        {"geo", std::move(geo)},
        {"motor", std::move(motor)}});
}

// Empty when the header holds the requested kind; else why the flight controller kept the one it holds.
std::string Link::kind_refusal(const link::SetKind& k, const link::SetupHeader& h) const {
    if (k.family >= param::kFamilyCount || h.kind[k.family] == k.kind) return {};
    const char* family = param::kSchema[k.family].id;
    std::string why = "refused: " + std::string(family) + " holds " + kind_name(k.family, h.kind[k.family]) + "; ";
    if (k.kind >= param::kind_count(k.family)) return why + kind_name(k.family, k.kind) + " is not a " + family + " kind";
    if (!param::compatible(k.family, k.kind, h.kind[param::k_vehicle]))
        return why + kind_name(k.family, k.kind) + " does not serve the " +
               kind_name(param::k_vehicle, h.kind[param::k_vehicle]) + " vehicle";
    return why + "the flight controller kept it";
}

// ---- mission ------------------------------------------------------------------------------------------------------------

void Link::mission_request(const std::string& type, const json::object& m, const Reply& reply) {
    const auto now = Clock::now();
    std::string refusal;
    if (type == "arm") {
        refusal = mission_.arm(tx_ != nullptr, seconds(now));
    } else if (type == "disarm") {
        refusal = mission_.disarm(seconds(now));
    } else if (type == "climb") {
        double alt;
        refusal = number(m, "alt_m", alt) ? mission_.climb(alt) : "want {alt_m: number}";
    } else if (type == "mission_start") {
        std::vector<Waypoint> wps;
        double speed = 0.0;
        std::uint8_t p = 0;
        const bool has_p = m.if_contains("profile");
        if (!waypoints(m, wps) || (m.if_contains("speed_mps") && !number(m, "speed_mps", speed)) ||
            (has_p && !profile(m, p)))
            refusal = "want {waypoints: [{lat, lon, alt_m}], speed_mps?: number, profile?: hold|freestyle|stabilized|agile}";
        else
            refusal = mission_.start(wps, speed);
        if (refusal.empty() && has_p) mission_.set_profile(p);
    } else if (type == "profile") {
        std::uint8_t p = 0;
        refusal = profile(m, p) ? mission_.set_profile(p) : "want {profile: hold|freestyle|stabilized|agile}";
    } else if (type == "rth") {
        refusal = mission_.rth();
    } else {
        refusal = mission_.land();
    }
    if (!refusal.empty()) return error(reply, type, refusal);
    next_mission_ = now;  // the new frame and state go out on the next poll
}

void Link::broadcast_link() {
    broadcast_(link_message(), false);
    state_key_.clear();
}

std::string Link::mission_message() const { return json::serialize(mission_state(mission_.status(seconds(Clock::now())))); }

// One 20 Hz period: the executor's frame, and mission_state on a change or every 500 ms while engaged.
void Link::mission_tick(Clock::time_point now) {
    const double t = seconds(now);
    MissionCommand c;
    if (mission_.tick(t, c)) {
        std::uint8_t f[link::kMaxFrame];
        send(f, link::encode(c, f));
    }
    const json::object o = mission_state(mission_.status(t));
    json::object key = o;
    key.erase("dist_m");
    std::string k = json::serialize(key);
    if (k == state_key_ && !(mission_.engaged(t) && now - last_state_ >= kMissionState)) return;
    state_key_ = std::move(k);
    last_state_ = now;
    broadcast_(json::serialize(o), false);
}

void Link::error(const Reply& reply, const std::string& request, const std::string& what) const {
    if (!reply) {
        std::fprintf(stderr, "marv_gcs: %s: %s\n", request.c_str(), what.c_str());
        return;
    }
    reply(json::serialize(json::object{{"type", "error"}, {"request", request}, {"error", what}}));
}

// ---- flash -------------------------------------------------------------------------------------------------------------

void Link::flash_log(const std::string& line) const {
    std::printf("%s\n", line.c_str());
    std::fflush(stdout);
    broadcast_(json::serialize(json::object{{"type", "flash_log"}, {"line", line}}), false);
}

void Link::flash(const Reply& reply) {
    const auto say = [&reply](const std::string& line) {
        if (reply) reply(json::serialize(json::object{{"type", "flash_log"}, {"line", line}}));
    };
    if (cfg_.mode == LinkConfig::Mode::kAuto && sim_)
        return say("flash: a sim is running and its bridge may hold the flight controller: sim_stop it first");
    if (cfg_.mode == LinkConfig::Mode::kUdp)
        return say("flash needs marv_gcs --serial DEV: the Pico must not be in use by a bridge; stop the bridge and "
                   "restart marv_gcs with --serial");
    if (flash_pid_ > 0) return say("flash: already running");
    int fds[2];
    if (::pipe2(fds, O_CLOEXEC) != 0) return say(std::string("flash: pipe: ") + std::strerror(errno));

    const std::string port = cfg_.mode == LinkConfig::Mode::kAuto ? (tx_ ? at_ : std::string("no USB port open")) : cfg_.target;
    tx_.reset();
    fail_all("flashing");
    flash_log("flash: closed " + port + "; flock " + kRigLock + " (cmake --build build/fw, openocd program)");
    const pid_t pid = ::fork();
    if (pid == 0) {
        ::dup2(fds[1], 1);
        ::dup2(fds[1], 2);
        if (::chdir(MARV_GCS_REPO_DIR) != 0) ::_exit(126);
        ::execlp("flock", "flock", kRigLock, "sh", "-c", kFlashScript, static_cast<char*>(nullptr));
        ::_exit(127);
    }
    ::close(fds[1]);
    if (pid < 0) {
        ::close(fds[0]);
        reopen_at_ = Clock::now();
        return flash_log(std::string("flash: fork: ") + std::strerror(errno));
    }
    flash_pid_ = pid;
    flash_out_.assign(fds[0]);
    broadcast_link();
    read_flash();
}

void Link::read_flash() {
    asio::async_read_until(flash_out_, flash_buf_, '\n', [this](const boost::system::error_code& ec, std::size_t n) {
        const auto begin = asio::buffers_begin(flash_buf_.data());
        if (!ec) {
            std::string line(begin, begin + static_cast<std::ptrdiff_t>(n));
            flash_buf_.consume(n);
            while (!line.empty() && (line.back() == '\n' || line.back() == '\r')) line.pop_back();
            flash_log(line);
            return read_flash();
        }
        if (ec == asio::error::operation_aborted) return;
        if (flash_buf_.size() > 0) {
            const std::string rest(begin, asio::buffers_end(flash_buf_.data()));
            flash_buf_.consume(flash_buf_.size());
            flash_log(rest);
        }
        flash_out_.close();
        int status = 0;
        ::waitpid(flash_pid_, &status, 0);
        flash_pid_ = -1;
        const bool ok = WIFEXITED(status) && WEXITSTATUS(status) == 0;
        const std::string port = cfg_.mode == LinkConfig::Mode::kAuto ? std::string("the USB link") : cfg_.target;
        flash_log(ok ? "flash: done; reopening " + port
                     : "flash: FAILED (exit " + std::to_string(WIFEXITED(status) ? WEXITSTATUS(status) : -1) +
                           "); reopening " + port);
        verify_flash_ = true;
        reopen_at_ = Clock::now() + kReopen;  // the Pico re-enumerates after its reset
        broadcast_link();
    });
}

}  // namespace marv::gcs
