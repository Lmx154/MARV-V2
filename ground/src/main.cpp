// marv_ground: the mission software. Conveys the mission to the flight controller over link frames.
//
//   marv_ground [--udp HOST:PORT] manual [--js DEV]      RadioMaster if plugged in, else the first joystick
//   marv_ground [--udp HOST:PORT] goto LAT LON ALT [YAW_DEG]
//   marv_ground [--udp HOST:PORT] waypoints FILE        lines `lat lon alt [hold_s]`, '#' comments
//   marv_ground [--udp HOST:PORT] preset N
//   marv_ground [--udp HOST:PORT] reboot
//
// Latitude and longitude in degrees, altitude in m above the WGS84 ellipsoid, all converted to NED about
// the home the flight controller reports. Commands go out at 50 Hz; fly is sent only once the estimate
// is valid. A status line prints every 0.5 s. Default --udp 127.0.0.1:14650 (marv_bridge --ground).
#include <fcntl.h>
#include <linux/joystick.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include <marv/fsw/geo.hpp>
#include <marv/link/protocol.hpp>

#include "transport.hpp"

namespace {

using namespace marv;
using Clock = std::chrono::steady_clock;

constexpr float kPi = 3.14159265358979f;
constexpr auto kPeriod = std::chrono::milliseconds(20);  // 50 Hz
constexpr float kDt = 0.02f;
constexpr float kArrive = 0.5f;  // m

volatile std::sig_atomic_t g_stop = 0;

int usage() {
    std::fprintf(stderr,
                 "usage: marv_ground [--udp HOST:PORT] manual [--js DEV]\n"
                 "       marv_ground [--udp HOST:PORT] goto LAT LON ALT [YAW_DEG]\n"
                 "       marv_ground [--udp HOST:PORT] waypoints FILE   (lines: lat lon alt [hold_s])\n"
                 "       marv_ground [--udp HOST:PORT] preset N\n"
                 "       marv_ground [--udp HOST:PORT] reboot\n");
    return 2;
}

float yaw_of(const Quat& q) {
    return std::atan2(2.f * (q.w * q.z + q.x * q.y), 1.f - 2.f * (q.y * q.y + q.z * q.z));
}

float dist(const Vec3& a, const Vec3& b) {
    const float x = a.x - b.x, y = a.y - b.y, z = a.z - b.z;
    return std::sqrt(x * x + y * y + z * z);
}

GeoPoint geo(double lat_deg, double lon_deg, double alt_m) {
    return {static_cast<std::int32_t>(std::lround(lat_deg * 1e7)), static_cast<std::int32_t>(std::lround(lon_deg * 1e7)),
            static_cast<float>(alt_m)};
}

// The link session: latest telemetry in, frames out, one 50 Hz period at a time.
class Ground {
public:
    explicit Ground(ground::Transport& tx) : tx_(tx), next_(Clock::now()), status_(next_) {}

    const Telemetry& tlm() const { return tlm_; }
    bool ready() const { return have_ && tlm_.est.valid; }
    bool home() const { return have_ && tlm_.home_valid; }
    void target(const Vec3* t) {
        has_target_ = t != nullptr;
        if (t) target_ = *t;
    }

    // Text appended to the status line (manual: the sticks).
    void note(const char* s) { std::snprintf(note_, sizeof(note_), "%s", s); }

    template <class T> bool send(const T& msg) {
        std::uint8_t frame[link::kMaxFrame];
        last_tx_ = Clock::now();
        return tx_.send(frame, link::encode(msg, frame));
    }

    // One period: sends cmd if given, else a bare delimiter (no frame) so the far end learns where to
    // reply; then reads telemetry until the period ends. Returns false on a link error.
    bool period(const MissionCommand* cmd) {
        if (cmd) {
            mode_ = cmd->mode;
            if (!send(*cmd)) return false;
        } else if (Clock::now() - last_tx_ >= std::chrono::milliseconds(500)) {
            const std::uint8_t delim = 0;
            last_tx_ = Clock::now();
            if (!tx_.send(&delim, 1)) return false;
        }
        next_ += kPeriod;
        for (;;) {
            const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(next_ - Clock::now()).count();
            std::uint8_t buf[2048];
            const long n = tx_.recv(buf, sizeof(buf), left > 0 ? static_cast<int>(left) : 0);
            if (n < 0) return false;
            for (long i = 0; i < n; ++i) {
                Telemetry t;
                if (dec_.push(buf[i]) && dec_.packet().as(t)) {
                    tlm_ = t;
                    have_ = true;
                }
            }
            if (n == 0 && left <= 0) break;
        }
        if (Clock::now() >= status_) {
            print_status();
            status_ += std::chrono::milliseconds(500);
        }
        return true;
    }

private:
    void print_status() const {
        if (!have_) {
            std::printf("waiting for telemetry");
        } else {
            const Vec3& p = tlm_.est.p_ned;
            std::printf("t=%8.2f mode=%-4s preset=%u est%s ned=(%7.2f %7.2f %7.2f)", static_cast<double>(tlm_.t_us) * 1e-6,
                        mode_ == Mode::kFly ? "fly" : "idle", static_cast<unsigned>(tlm_.preset),
                        tlm_.est.valid ? "" : "(invalid)", static_cast<double>(p.x), static_cast<double>(p.y),
                        static_cast<double>(p.z));
            if (has_target_) std::printf(" dist=%.2f", static_cast<double>(dist(p, target_)));
        }
        std::printf("%s\n", note_);
        std::fflush(stdout);
    }

    ground::Transport& tx_;
    link::Decoder dec_;
    Telemetry tlm_{};
    bool have_ = false;
    Clock::time_point last_tx_{};
    char note_[160] = "";
    Mode mode_ = Mode::kIdle;
    bool has_target_ = false;
    Vec3 target_{};
    Clock::time_point next_, status_;
};

MissionCommand command(Mode mode, std::uint8_t has, const Vec3& p, const Vec3& v, float yaw) {
    MissionCommand c{};
    c.mode = mode;
    c.nav = NavSource::kEstimate;
    c.ref.has = has;
    c.ref.p_ned = p;
    c.ref.v_ned = v;
    c.ref.yaw = yaw;
    c.ref.q = {1.f, 0.f, 0.f, 0.f};
    return c;
}

// Waits until the estimate is valid and home is set. Returns 0, or the exit code to leave with.
int wait_ready(Ground& g) {
    while (!(g.ready() && g.home())) {
        if (g_stop) return 130;
        if (!g.period(nullptr)) return 1;
    }
    return 0;
}

// Flies to target (NED about home) and holds for hold_s once within kArrive. Returns 0 or an exit code.
int fly_to(Ground& g, const Vec3& target, float yaw, float hold_s) {
    g.target(&target);
    const MissionCommand cmd = command(Mode::kFly, kRefPos | kRefYaw, target, {0.f, 0.f, 0.f}, yaw);
    while (dist(g.tlm().est.p_ned, target) >= kArrive) {
        if (g_stop) return 130;
        if (!g.period(&cmd)) return 1;
    }
    std::printf("arrived at (%.2f %.2f %.2f) t=%.2f\n", static_cast<double>(target.x), static_cast<double>(target.y),
                static_cast<double>(target.z), static_cast<double>(g.tlm().t_us) * 1e-6);
    for (float t = 0.f; t < hold_s; t += kDt) {
        if (g_stop) return 130;
        if (!g.period(&cmd)) return 1;
    }
    return 0;
}

int run_goto(Ground& g, int argc, char** argv) {
    if (argc != 3 && argc != 4) return usage();
    if (const int rc = wait_ready(g)) return rc;
    LocalFrame frame;
    frame.set(g.tlm().home);
    const Vec3 target = frame.to_ned(geo(std::atof(argv[0]), std::atof(argv[1]), std::atof(argv[2])));
    const float yaw = argc == 4 ? static_cast<float>(std::atof(argv[3])) * kPi / 180.f : yaw_of(g.tlm().est.q);
    return fly_to(g, target, yaw, 0.f);
}

int run_waypoints(Ground& g, int argc, char** argv) {
    if (argc != 1) return usage();
    struct Wp {
        double lat, lon, alt;
        float hold_s;
    };
    std::vector<Wp> wps;
    std::FILE* f = std::fopen(argv[0], "r");
    if (!f) {
        std::fprintf(stderr, "marv_ground: cannot read %s\n", argv[0]);
        return 2;
    }
    char line[256];
    int no = 0;
    while (std::fgets(line, sizeof(line), f)) {
        ++no;
        if (char* c = std::strchr(line, '#')) *c = '\0';
        Wp w{0.0, 0.0, 0.0, 0.f};
        char extra[2];
        const int k = std::sscanf(line, "%lf %lf %lf %f %1s", &w.lat, &w.lon, &w.alt, &w.hold_s, extra);
        if (k <= 0) continue;
        if (k != 3 && k != 4) {
            std::fprintf(stderr, "marv_ground: %s:%d: want `lat lon alt [hold_s]`\n", argv[0], no);
            std::fclose(f);
            return 2;
        }
        wps.push_back(w);
    }
    std::fclose(f);
    if (wps.empty()) return usage();
    if (const int rc = wait_ready(g)) return rc;
    LocalFrame frame;
    frame.set(g.tlm().home);
    const float yaw = yaw_of(g.tlm().est.q);
    for (const Wp& w : wps)
        if (const int rc = fly_to(g, frame.to_ned(geo(w.lat, w.lon, w.alt)), yaw, w.hold_s)) return rc;
    return 0;
}

// Stick axis to -1..1 with a 10% deadband, rescaled so the output starts at 0 at the band's edge.
float stick(std::int16_t raw) {
    constexpr float kBand = 0.1f;
    const float v = static_cast<float>(raw) / 32767.f;
    const float a = std::fabs(v);
    if (a <= kBand) return 0.f;
    return std::copysign(std::fmin((a - kBand) / (1.f - kBand), 1.f), v);
}

bool is_radio(const char* name) { return strcasestr(name, "edgetx") || strcasestr(name, "radiomaster"); }

// Opens the pilot's device: --js PATH, else the RadioMaster if one is plugged in, else the first joystick.
// name is the device's JSIOCGNAME, or the path when it is not a joystick device (a FIFO replaying events).
int open_pilot(const char* path, char* name, std::size_t cap) {
    int fd = -1;
    if (path) {
        std::snprintf(name, cap, "%s", path);
        fd = ::open(path, O_RDONLY);  // blocks until a FIFO has a writer; a device opens at once
        if (fd >= 0) ::ioctl(fd, JSIOCGNAME(cap), name);
    } else {
        for (int i = 0; i < 16; ++i) {
            char dev[32], dev_name[128] = "";
            std::snprintf(dev, sizeof(dev), "/dev/input/js%d", i);
            const int f = ::open(dev, O_RDONLY | O_NONBLOCK);
            if (f < 0) continue;
            ::ioctl(f, JSIOCGNAME(sizeof(dev_name)), dev_name);
            if (fd < 0 || is_radio(dev_name)) {
                if (fd >= 0) ::close(fd);
                fd = f;
                std::snprintf(name, cap, "%s (%s)", dev_name, dev);
                if (is_radio(dev_name)) break;
            } else {
                ::close(f);
            }
        }
    }
    if (fd < 0 || ::fcntl(fd, F_SETFL, O_NONBLOCK) != 0) {
        std::fprintf(stderr, "marv_ground: no joystick%s%s: %s\n", path ? " " : "", path ? path : "", std::strerror(errno));
        if (fd >= 0) ::close(fd);
        return -1;
    }
    return fd;
}

// Two mappings, picked by the device name. Each gives climb, yaw, forward, right in -1..1 (positive =
// up, clockwise, forward, right) and the arm switch.
//   RadioMaster (EdgeTX USB joystick, AETR): a0 aileron = right, a1 elevator = forward, a2 throttle =
//     climb with the stick centre = hold, a3 rudder = yaw, a4 (CH5) > 0 = arm. Arming needs CH5 turned
//     on with the throttle centred; otherwise it is refused until CH5 goes off again.
//   Xbox (xpad): a0 left X = yaw, a1 left Y = climb, a3/a4 right X/Y = right/forward (up is negative),
//     button A = arm, B = disarm.
int run_manual(Ground& g, int argc, char** argv) {
    const char* path = nullptr;
    if (argc == 2 && std::strcmp(argv[0], "--js") == 0) path = argv[1];
    else if (argc != 0) return usage();
    char name[160];
    const int fd = open_pilot(path, name, sizeof(name));
    if (fd < 0) return 1;
    const bool radio = is_radio(name);
    std::printf("pilot: %s, %s mapping\n", name, radio ? "RadioMaster" : "Xbox");
    constexpr float kClimbMax = 1.5f, kYawRateMax = kPi / 2.f, kSpeedMax = 3.f;
    std::int16_t axis[8] = {};
    bool seen[8] = {};
    bool fly = false, have_yaw = false, refused = false;
    // Nothing is sent until the pilot arms: starting manual against a vehicle that is already flying must
    // not disarm it. From the first arm on, the pilot owns the vehicle and disarm sends idle.
    bool engaged = false;
    int prev_switch = -1;  // CH5 at the last period; -1 until the device reported it
    float yaw = 0.f;
    std::uint8_t ev_buf[sizeof(js_event)];
    std::size_t ev_n = 0;
    int rc = 0;
    while (!g_stop) {
        bool lost = false;
        for (int k_ev = 0; k_ev < 512; ++k_ev) {  // the radio streams without pause: bounded, never "until quiet"
            const ssize_t k = ::read(fd, ev_buf + ev_n, sizeof(ev_buf) - ev_n);
            if (k < 0 && errno == EAGAIN) break;
            if (k <= 0) {
                lost = true;
                break;
            }
            ev_n += static_cast<std::size_t>(k);
            if (ev_n < sizeof(ev_buf)) continue;
            ev_n = 0;
            js_event e;
            std::memcpy(&e, ev_buf, sizeof(e));
            const std::uint8_t type = e.type & static_cast<std::uint8_t>(~JS_EVENT_INIT);
            if (type == JS_EVENT_AXIS && e.number < 8) {
                axis[e.number] = e.value;
                seen[e.number] = true;
            } else if (!radio && type == JS_EVENT_BUTTON && e.value && e.number == 0) {
                fly = true;
            } else if (!radio && type == JS_EVENT_BUTTON && e.value && e.number == 1) {
                fly = false;
            }
        }
        if (lost) {
            std::fprintf(stderr, "marv_ground: joystick %s lost, holding\n", name);
            if (engaged && have_yaw) g.send(command(fly ? Mode::kFly : Mode::kIdle, kRefVel | kRefYaw, {}, {0.f, 0.f, 0.f}, yaw));
            rc = 1;
            break;
        }
        const float climb = radio ? stick(axis[2]) : 0.f - stick(axis[1]);
        const float yaw_in = radio ? stick(axis[3]) : stick(axis[0]);
        const float fwd = radio ? stick(axis[1]) : 0.f - stick(axis[4]);
        const float right = radio ? stick(axis[0]) : stick(axis[3]);
        if (radio && seen[4]) {
            const int sw = axis[4] > 0 ? 1 : 0;
            if (!sw) fly = refused = false;
            else if (prev_switch == 0 && climb == 0.f) fly = true;
            else if (prev_switch == 0) refused = true;
            prev_switch = sw;
        }
        char note[160];
        std::snprintf(note, sizeof(note), " | climb=%+.2f yaw=%+.2f fwd=%+.2f right=%+.2f arm=%s", static_cast<double>(climb),
                      static_cast<double>(yaw_in), static_cast<double>(fwd), static_cast<double>(right),
                      fly ? "on" : refused ? "REFUSED(centre throttle, cycle CH5)" : "off");
        g.note(note);
        if (!g.ready()) {
            if (!g.period(nullptr)) {
                rc = 1;
                break;
            }
            continue;
        }
        // Idle tracks the vehicle's heading, so arming keeps the nose where it is.
        if (!fly || !have_yaw) yaw = yaw_of(g.tlm().est.q);
        have_yaw = true;
        yaw = std::remainder(yaw + yaw_in * kYawRateMax * kDt, 2.f * kPi);
        const float c = std::cos(yaw), s = std::sin(yaw);
        const Vec3 v{(c * fwd - s * right) * kSpeedMax, (s * fwd + c * right) * kSpeedMax, -climb * kClimbMax};
        const MissionCommand cmd = command(fly ? Mode::kFly : Mode::kIdle, kRefVel | kRefYaw, {}, v, yaw);
        engaged = engaged || fly;
        if (!g.period(engaged ? &cmd : nullptr)) {
            rc = 1;
            break;
        }
    }
    ::close(fd);
    return g_stop ? 130 : rc;
}

// Sends one frame, then shows telemetry for a second.
template <class T> int run_once(Ground& g, const T& msg) {
    if (!g.send(msg)) {
        std::fprintf(stderr, "marv_ground: send failed\n");
        return 1;
    }
    for (int i = 0; i < 50 && !g_stop; ++i)
        if (!g.period(nullptr)) return 1;
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    const char* udp = "127.0.0.1:14650";
    int i = 1;
    if (i + 1 < argc && std::strcmp(argv[i], "--udp") == 0) {
        udp = argv[i + 1];
        i += 2;
    }
    if (i >= argc) return usage();
    const std::string sub = argv[i++];
    char** rest = argv + i;
    const int n = argc - i;

    std::signal(SIGINT, [](int) { g_stop = 1; });
    std::signal(SIGTERM, [](int) { g_stop = 1; });

    auto tx = ground::open_udp(udp);
    if (!tx) return 1;
    Ground g(*tx);
    if (sub == "manual") return run_manual(g, n, rest);
    if (sub == "goto") return run_goto(g, n, rest);
    if (sub == "waypoints") return run_waypoints(g, n, rest);
    if (sub == "preset" && n == 1) return run_once(g, link::SetPreset{static_cast<std::uint8_t>(std::atoi(rest[0]))});
    if (sub == "reboot" && n == 0) return run_once(g, link::Reboot{});
    return usage();
}
