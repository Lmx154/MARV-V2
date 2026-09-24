#include "link.hpp"

#include <fcntl.h>
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <utility>

#include <boost/asio/buffers_iterator.hpp>
#include <boost/asio/read_until.hpp>
#include <boost/json.hpp>

#include "schema.hpp"

namespace marv::gcs {
namespace asio = boost::asio;
namespace json = boost::json;
using namespace std::chrono_literals;

namespace {

constexpr auto kPoll = 2ms;
constexpr auto kReplyTimeout = 500ms;
constexpr auto kKeepalive = 500ms;       // a lone 0x00 while nothing arrives, so the bridge learns where to send
constexpr auto kTelemetryPeriod = 34ms;  // at most 30 Hz to the clients
constexpr auto kReopen = 2s;
constexpr std::size_t kMaxQueue = 256;
constexpr std::size_t kMaxValues = 4096;
constexpr const char* kRigLock = "/tmp/marv-rig.lock";

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

}  // namespace

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
    tx_ = cfg_.serial ? ground::open_serial(cfg_.target.c_str()) : ground::open_udp(cfg_.target.c_str());
    if (!tx_ && !cfg_.serial) return false;
    if (!tx_) reopen_at_ = Clock::now() + kReopen;  // the device may appear later
    else if (cfg_.serial) enqueue(Kind::kSetup, "request_setup", link::SetupRequest{}, {});
    tick();
    return true;
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
        return set_connected(false);
    }
    if (type == "flash") return flash(reply);
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
        tx_ = cfg_.serial ? ground::open_serial(cfg_.target.c_str()) : ground::open_udp(cfg_.target.c_str());
        if (!tx_) {
            reopen_at_ = now + kReopen;
        } else {
            dec_ = link::Decoder{};
            broadcast_(link_message(), false);
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
    if (tx_ && !cfg_.serial && now - last_rx_ > 2 * kKeepalive && now - last_tx_ >= kKeepalive) {
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
    std::fprintf(stderr, "marv_gcs: %s: %s\n", cfg_.target.c_str(), why);
    tx_.reset();
    reopen_at_ = Clock::now() + kReopen;
    fail_all(why);
    broadcast_(link_message(), false);
}

void Link::set_connected(bool connected) {
    if (connected == connected_) return;
    connected_ = connected;
    broadcast_(link_message(), false);
}

void Link::receive(const link::Packet& p) {
    link::SetupHeader h;
    link::ParamValue v;
    Telemetry t;
    ActuatorCommand a;
    if (p.as(h)) return on_header(h);
    if (p.as(v)) return on_value(v);
    if (p.as(t)) {
        tlm_ = t;
        tlm_pending_ = true;
    } else if (p.as(a)) {
        armed_ = a.armed;
        have_armed_ = true;
    }
    // Heard, but not connected: ask for the setup (after a reboot, or a bridge started after us).
    const auto now = Clock::now();
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
    if (changed) broadcast_(link_message(), false);
    if (!queue_.empty() && !rx_header_) {
        switch (queue_.front().kind) {
        case Kind::kSetup:
            rx_header_ = true;
            rx_values_.assign(std::min<std::size_t>(h.param_count, kMaxValues), std::numeric_limits<float>::quiet_NaN());
            rx_count_ = 0;
            deadline_ = Clock::now() + kReplyTimeout;
            if (rx_values_.empty()) on_value({0, 0.f});
            return;
        case Kind::kSetKind:
        case Kind::kSave:
        case Kind::kReset:
            broadcast_(setup_message(false), false);
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
    json::object o{{"mode", cfg_.serial ? "serial" : "sitl-udp"},
                   {"target", cfg_.target},
                   {"open", tx_ != nullptr},
                   {"connected", connected_},
                   {"schema_hash", param::kSchemaHash},
                   {"flashing", flash_pid_ > 0}};
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
    return json::serialize(json::object{
        {"type", "telemetry"},
        {"t_us", tlm_.t_us},
        {"preset", tlm_.preset},
        {"armed", armed},
        {"est", json::object{{"t_us", e.t_us}, {"p_ned", vec3(e.p_ned)}, {"v_ned", vec3(e.v_ned)}, {"q", quat(e.q)},
                             {"w_frd", vec3(e.w_frd)}, {"valid", e.valid}}},
        {"force_ned", vec3(tlm_.req.force_ned)},
        {"torque_frd", vec3(tlm_.req.torque_frd)},
        {"home_valid", tlm_.home_valid},
        {"home", json::object{{"lat_e7", tlm_.home.lat_e7}, {"lon_e7", tlm_.home.lon_e7}, {"alt_m", fnum(tlm_.home.alt_m)}}}});
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
    if (!cfg_.serial)
        return say("flash needs marv_gcs --serial DEV: the Pico must not be in use by a bridge; stop the bridge and "
                   "restart marv_gcs with --serial");
    if (flash_pid_ > 0) return say("flash: already running");
    int fds[2];
    if (::pipe2(fds, O_CLOEXEC) != 0) return say(std::string("flash: pipe: ") + std::strerror(errno));

    tx_.reset();
    fail_all("flashing");
    flash_log("flash: closed " + cfg_.target + "; flock " + kRigLock + " (cmake --build build/fw, openocd program)");
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
    broadcast_(link_message(), false);
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
        flash_log(ok ? "flash: done; reopening " + cfg_.target
                     : "flash: FAILED (exit " + std::to_string(WIFEXITED(status) ? WEXITSTATUS(status) : -1) +
                           "); reopening " + cfg_.target);
        verify_flash_ = true;
        reopen_at_ = Clock::now() + kReopen;  // the Pico re-enumerates after its reset
        broadcast_(link_message(), false);
    });
}

}  // namespace marv::gcs
