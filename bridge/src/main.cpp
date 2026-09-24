// marv_bridge: runs the Gazebo world and the flight software in lockstep over the wire protocol.
//
//   marv_bridge (--sitl | --port /dev/ttyACMx) --seconds S [--mission FILE | --ground] [--log out.csv]
//               [--max-rot-velocity RAD_S] [--link NAME]
//
// --max-rot-velocity: the rotor speed a motor command of 1 asks of the world's motor models, its SDF's
// maxRotVelocity (default 800, the X3's; scripts/sim.sh passes the airframe's). --link: the model's link that carries
// the sensors (default X3/base_link; the worlds of the airframe catalog sitl/airframes name it base_link).
//
// Per block of bridge::kStepsPerBlock physics steps: step the world, then for each step in order send the
// truth State it produced (when valid), the mission command (when the active mission line changed, and
// once at the start), then the SensorBus; collect the Telemetry and wait for the ActuatorCommand that
// echoes its t_us. The block's last command goes to the motor models before the next block; repeat.
//
// Mission file: one line per command, `t_s mode nav n e d yaw_deg`, mode fly|idle, nav truth|estimate,
// position NED in m relative to the start, yaw in degrees; '#' starts a comment. The line with the
// largest t_s <= the step's time is active; before the first line the vehicle is idle.
//
// --ground: the mission comes from marv_ground instead, over UDP 127.0.0.1:14650. Each datagram from the
// ground is complete link frames, forwarded unchanged between the truth and the SensorBus of the next
// step; every frame the flight controller sends back goes unchanged to every ground address heard in the
// last 2 s, so a ground station and marv_ground can share the link.
// The world is paced to wall-clock time so a pilot flies it in real time.
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <marv/link/protocol.hpp>

#include "endpoint.hpp"
#include "gz_world.hpp"

namespace {

using namespace marv;

int usage() {
    std::fprintf(stderr,
                 "usage: marv_bridge (--sitl | --port /dev/ttyACMx) --seconds S [--mission FILE | --ground] [--log out.csv]\n"
                 "                   [--max-rot-velocity RAD_S] [--link NAME]\n");
    return 2;
}

struct MissionLine {
    std::uint64_t t_us;
    MissionCommand cmd;
};

// Reads a mission file into lines sorted by time. Returns false, with a message on stderr, on an error.
bool load_mission(const char* path, std::vector<MissionLine>& out) {
    std::FILE* f = std::fopen(path, "r");
    if (!f) {
        std::fprintf(stderr, "marv_bridge: cannot read %s\n", path);
        return false;
    }
    char line[256];
    int no = 0;
    bool ok = true;
    while (ok && std::fgets(line, sizeof(line), f)) {
        ++no;
        if (char* c = std::strchr(line, '#')) *c = '\0';
        double t = 0.0;
        char mode[16], nav[16];
        float n = 0.f, e = 0.f, d = 0.f, yaw_deg = 0.f;
        char extra[2];
        const int k = std::sscanf(line, "%lf %15s %15s %f %f %f %f %1s", &t, mode, nav, &n, &e, &d, &yaw_deg, extra);
        if (k <= 0) continue;  // blank or comment
        MissionLine m{};
        ok = k == 7 && t >= 0.0 && (!std::strcmp(mode, "fly") || !std::strcmp(mode, "idle")) &&
             (!std::strcmp(nav, "truth") || !std::strcmp(nav, "estimate"));
        if (!ok) {
            std::fprintf(stderr, "marv_bridge: %s:%d: want `t_s fly|idle truth|estimate n e d yaw_deg`\n", path, no);
            break;
        }
        m.t_us = static_cast<std::uint64_t>(t * 1e6 + 0.5);
        m.cmd.mode = std::strcmp(mode, "fly") == 0 ? Mode::kFly : Mode::kIdle;
        m.cmd.nav = std::strcmp(nav, "truth") == 0 ? NavSource::kTruth : NavSource::kEstimate;
        m.cmd.ref.has = kRefPos | kRefYaw;
        m.cmd.ref.p_ned = {n, e, d};
        m.cmd.ref.yaw = yaw_deg * 3.14159265f / 180.f;
        m.cmd.ref.q = {1.f, 0.f, 0.f, 0.f};
        out.push_back(m);
    }
    std::fclose(f);
    std::stable_sort(out.begin(), out.end(), [](const MissionLine& a, const MissionLine& b) { return a.t_us < b.t_us; });
    return ok;
}

// The ground link of --ground: a UDP socket on 127.0.0.1:14650 and every address heard in the last 2 s.
struct GroundLink {
    struct Peer {
        sockaddr_in addr;
        std::chrono::steady_clock::time_point heard;
    };
    int fd = -1;
    std::vector<Peer> peers;

    bool open(std::string& err) {
        sockaddr_in a{};
        a.sin_family = AF_INET;
        a.sin_port = htons(14650);
        a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        fd = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (fd < 0 || ::bind(fd, reinterpret_cast<const sockaddr*>(&a), sizeof(a)) != 0) {
            err = std::string("ground socket 127.0.0.1:14650: ") + std::strerror(errno);
            return false;
        }
        return true;
    }

    // Appends every datagram waiting that ends a frame; a datagram cut mid-frame would corrupt the next.
    void drain(std::vector<std::uint8_t>& out) {
        const auto now = std::chrono::steady_clock::now();
        for (;;) {
            std::uint8_t buf[2048];
            sockaddr_in from{};
            socklen_t len = sizeof(from);
            const ssize_t n = ::recvfrom(fd, buf, sizeof(buf), MSG_DONTWAIT, reinterpret_cast<sockaddr*>(&from), &len);
            if (n < 0) break;
            const auto same = [&](const Peer& p) {
                return p.addr.sin_addr.s_addr == from.sin_addr.s_addr && p.addr.sin_port == from.sin_port;
            };
            const auto it = std::find_if(peers.begin(), peers.end(), same);
            if (it != peers.end()) it->heard = now;
            else peers.push_back({from, now});
            if (n > 0 && buf[n - 1] == 0) out.insert(out.end(), buf, buf + n);
        }
        peers.erase(std::remove_if(peers.begin(), peers.end(),
                                   [&](const Peer& p) { return now - p.heard > std::chrono::seconds(2); }),
                    peers.end());
    }

    // Sends the complete frames in fc (up to its last delimiter) to every peer and keeps the rest for next time.
    void forward(std::vector<std::uint8_t>& fc) {
        const auto end = std::find(fc.rbegin(), fc.rend(), std::uint8_t{0}).base();
        if (end != fc.begin())
            for (const Peer& p : peers)
                ::sendto(fd, fc.data(), static_cast<std::size_t>(end - fc.begin()), 0,
                         reinterpret_cast<const sockaddr*>(&p.addr), sizeof(p.addr));
        fc.erase(fc.begin(), end);
    }
};

// Sends truth (if valid), mission (if given), the ground's frames and bus, then waits up to 1 s for the
// ActuatorCommand answering bus. The Telemetry of the same tick is kept in tlm; other packets are
// dropped. Every byte read is appended to fc_bytes when given.
bool exchange(bridge::Endpoint& ep, link::Decoder& dec, const State& truth, const MissionCommand* mission,
              const std::vector<std::uint8_t>& ground, const SensorBus& bus, Telemetry& tlm, ActuatorCommand& cmd,
              std::vector<std::uint8_t>* fc_bytes, std::string& err) {
    std::vector<std::uint8_t> out(3 * link::kMaxFrame + ground.size());
    std::size_t n_out = 0;
    if (truth.valid) n_out += link::encode(truth, out.data() + n_out);
    if (mission) n_out += link::encode(*mission, out.data() + n_out);
    std::copy(ground.begin(), ground.end(), out.begin() + static_cast<long>(n_out));
    n_out += ground.size();
    n_out += link::encode(bus, out.data() + n_out);
    if (!ep.write(out.data(), n_out)) {
        err = "write to the flight controller failed";
        return false;
    }
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
    for (;;) {
        const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - std::chrono::steady_clock::now());
        if (left.count() < 0) break;
        std::uint8_t buf[256];
        const long n = ep.read(buf, sizeof(buf), static_cast<int>(left.count()));
        if (n < 0) {
            err = "read from the flight controller failed";
            return false;
        }
        if (fc_bytes) fc_bytes->insert(fc_bytes->end(), buf, buf + n);
        bool found = false;
        for (long i = 0; i < n; ++i) {
            if (!dec.push(buf[i])) continue;
            ActuatorCommand c;
            Telemetry t;
            if (dec.packet().as(t) && t.t_us == bus.t_us) tlm = t;
            else if (dec.packet().as(c) && c.t_us == bus.t_us) {
                cmd = c;
                found = true;
            }
        }
        if (found) return true;
    }
    err = "no ActuatorCommand for t_us=" + std::to_string(bus.t_us) + " within 1 s";
    return false;
}

void log_header(std::FILE* f) {
    std::fprintf(f,
                 "t_us,"
                 "truth_n_m,truth_e_m,truth_d_m,truth_vn_mps,truth_ve_mps,truth_vd_mps,"
                 "truth_qw,truth_qx,truth_qy,truth_qz,"
                 "fresh,accel_frd_x,accel_frd_y,accel_frd_z,gyro_frd_x,gyro_frd_y,gyro_frd_z,"
                 "baro_pa,baro_c,mag_frd_x_ut,mag_frd_y_ut,mag_frd_z_ut,"
                 "gnss_lat_e7,gnss_lon_e7,gnss_alt_m,gnss_vn_mps,gnss_ve_mps,gnss_vd_mps,gnss_fix,"
                 "motor0,motor1,motor2,motor3,"
                 "est_valid,est_n_m,est_e_m,est_d_m,est_vn_mps,est_ve_mps,est_vd_mps,est_qw,est_qx,est_qy,est_qz,"
                 "thrust_n,thrust_e,thrust_d,torque_x,torque_y,torque_z,thrust_hover,brake,preset,home_valid,home_lat_e7,home_lon_e7,home_alt_m\n");
}

void log_row(std::FILE* f, const State& x, const SensorBus& s, const ActuatorCommand& c, const Telemetry& t) {
    std::fprintf(f,
                 "%llu,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,"
                 "%u,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,"
                 "%ld,%ld,%.9g,%.9g,%.9g,%.9g,%d,%.9g,%.9g,%.9g,%.9g,",
                 static_cast<unsigned long long>(s.t_us), x.p_ned.x, x.p_ned.y, x.p_ned.z, x.v_ned.x, x.v_ned.y,
                 x.v_ned.z, x.q.w, x.q.x, x.q.y, x.q.z, static_cast<unsigned>(s.fresh), s.imu.accel_frd.x,
                 s.imu.accel_frd.y, s.imu.accel_frd.z, s.imu.gyro_frd.x, s.imu.gyro_frd.y, s.imu.gyro_frd.z,
                 s.baro.pressure_pa, s.baro.temperature_c, s.mag.field_frd_ut.x, s.mag.field_frd_ut.y,
                 s.mag.field_frd_ut.z, static_cast<long>(s.gnss.lat_e7), static_cast<long>(s.gnss.lon_e7),
                 s.gnss.alt_m, s.gnss.vel_ned.x, s.gnss.vel_ned.y, s.gnss.vel_ned.z, s.gnss.fix ? 1 : 0,
                 c.motor[0], c.motor[1], c.motor[2], c.motor[3]);
    const State& e = t.est;
    std::fprintf(f, "%d,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%u,%d,%ld,%ld,%.9g\n",
                 e.valid ? 1 : 0, e.p_ned.x, e.p_ned.y, e.p_ned.z, e.v_ned.x, e.v_ned.y, e.v_ned.z, e.q.w, e.q.x,
                 e.q.y, e.q.z, t.req.thrust_ned.x, t.req.thrust_ned.y, t.req.thrust_ned.z, t.req.torque_frd.x,
                 t.req.torque_frd.y, t.req.torque_frd.z, t.req.thrust_hover, t.req.brake, static_cast<unsigned>(t.preset), t.home_valid ? 1 : 0,
                 static_cast<long>(t.home.lat_e7), static_cast<long>(t.home.lon_e7), t.home.alt_m);
}

}  // namespace

int main(int argc, char** argv) {
    bool sitl = false;
    const char* port = nullptr;
    const char* log_path = nullptr;
    const char* mission_path = nullptr;
    bool ground_mode = false;
    double seconds = -1.0;
    double max_rot_velocity = 800.0;  // rad/s, maxRotVelocity in sitl/airframes/x3/model.sdf
    std::string link = "X3/base_link";
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        const bool has_value = i + 1 < argc;
        if (a == "--sitl") sitl = true;
        else if (a == "--port" && has_value) port = argv[++i];
        else if (a == "--seconds" && has_value) seconds = std::strtod(argv[++i], nullptr);
        else if (a == "--log" && has_value) log_path = argv[++i];
        else if (a == "--mission" && has_value) mission_path = argv[++i];
        else if (a == "--ground") ground_mode = true;
        else if (a == "--max-rot-velocity" && has_value) max_rot_velocity = std::strtod(argv[++i], nullptr);
        else if (a == "--link" && has_value) link = argv[++i];
        else return usage();
    }
    if (sitl == (port != nullptr) || !(seconds > 0.0) || (ground_mode && mission_path) || !(max_rot_velocity > 0.0))
        return usage();

    std::vector<MissionLine> mission;
    if (mission_path && !load_mission(mission_path, mission)) return 2;

    std::unique_ptr<bridge::Endpoint> ep = sitl ? bridge::make_sitl() : bridge::open_serial(port);
    if (!ep) return 1;

    std::FILE* log = nullptr;
    if (log_path) {
        log = std::fopen(log_path, "w");
        if (!log) {
            std::fprintf(stderr, "marv_bridge: cannot write %s\n", log_path);
            return 1;
        }
        log_header(log);
    }

    // A new run: the flight controller may still hold the state of the previous one.
    {
        std::uint8_t frame[link::kMaxFrame];
        if (!ep->write(frame, link::encode(link::Reset{}, frame))) {
            std::fprintf(stderr, "marv_bridge: write to the flight controller failed\n");
            return 1;
        }
    }

    GroundLink ground;
    std::string err;
    if (ground_mode && !ground.open(err)) {
        std::fprintf(stderr, "marv_bridge: %s\n", err.c_str());
        return 1;
    }
    std::vector<std::uint8_t> ground_in, fc_out;

    bridge::GzWorld world(max_rot_velocity, link);
    if (!world.connect(err)) {
        std::fprintf(stderr, "marv_bridge: %s\n", err.c_str());
        return 1;
    }

    const auto end_us = static_cast<std::uint64_t>(seconds * 1e6 + 0.5);
    link::Decoder dec;
    SensorBus bus{};
    State truth{};
    ActuatorCommand cmd{};
    const MissionCommand idle{Mode::kIdle, NavSource::kEstimate, {}};
    long active = -2;  // index of the mission line sent last; -1 is the idle before the first line
    std::uint64_t steps = 0;
    const auto wall0 = std::chrono::steady_clock::now();
    int rc = 0;
    bridge::GzWorld::Step block[bridge::kStepsPerBlock];
    while (rc == 0 && bus.t_us < end_us) {
        if (!world.step(block, err)) {
            std::fprintf(stderr, "marv_bridge: %s\n", err.c_str());
            rc = 1;
            break;
        }
        for (int k = 0; k < bridge::kStepsPerBlock && bus.t_us < end_us; ++k) {
            bus = block[k].bus;
            truth = block[k].truth;
            long now = -1;
            while (now + 1 < static_cast<long>(mission.size()) && mission[static_cast<std::size_t>(now + 1)].t_us <= bus.t_us) ++now;
            const MissionCommand* send = nullptr;
            if (now != active) {
                send = now < 0 ? &idle : &mission[static_cast<std::size_t>(now)].cmd;
                active = now;
            }
            Telemetry tlm{};
            const float nan = std::nanf("");
            tlm.est = {0, {nan, nan, nan}, {nan, nan, nan}, {nan, nan, nan, nan}, {nan, nan, nan}, false};
            tlm.req = {{nan, nan, nan}, {nan, nan, nan}, nan, nan};
            ground_in.clear();
            if (ground_mode) ground.drain(ground_in);
            if (!exchange(*ep, dec, truth, send, ground_in, bus, tlm, cmd, ground_mode ? &fc_out : nullptr, err)) {
                std::fprintf(stderr, "marv_bridge: %s\n", err.c_str());
                rc = 1;
                break;
            }
            if (ground_mode) {
                ground.forward(fc_out);
                std::this_thread::sleep_until(wall0 + std::chrono::microseconds(bus.t_us));
            }
            if (log) log_row(log, truth, bus, cmd, tlm);
            ++steps;
        }
        // Zero-order hold: the motor models apply the block's last command through the next block.
        if (rc == 0) world.command(cmd);
    }
    const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - wall0).count();
    std::fprintf(stderr, "marv_bridge: %llu steps, sim %.3f s, wall %.3f s, %.3f sim-s per wall-s, %u bad frames\n",
                 static_cast<unsigned long long>(steps), static_cast<double>(bus.t_us) * 1e-6, wall,
                 static_cast<double>(bus.t_us) * 1e-6 / wall, dec.errors());
    if (log) std::fclose(log);
    if (ground.fd >= 0) ::close(ground.fd);
    return rc;
}
