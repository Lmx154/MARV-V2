// marv_bridge: runs the Gazebo world and the flight software in lockstep over the wire protocol.
//
//   marv_bridge (--sitl [--cmd F] | --port /dev/ttyACMx) --seconds S [--log out.csv]
//
// Per physics step: step the world, send the SensorBus it produced, wait for the ActuatorCommand that
// echoes its t_us, hand that to the motor models, repeat.
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>

#include <marv/link/protocol.hpp>

#include "endpoint.hpp"
#include "gz_world.hpp"

namespace {

using namespace marv;

int usage() {
    std::fprintf(stderr, "usage: marv_bridge (--sitl [--cmd F] | --port /dev/ttyACMx) --seconds S [--log out.csv]\n");
    return 2;
}

// Sends bus, then waits up to 1 s for the ActuatorCommand answering it. Other packets are dropped.
bool exchange(bridge::Endpoint& ep, link::Decoder& dec, const SensorBus& bus, ActuatorCommand& cmd,
              std::string& err) {
    std::uint8_t frame[link::kMaxFrame];
    if (!ep.write(frame, link::encode(bus, frame))) {
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
        bool found = false;
        for (long i = 0; i < n; ++i) {
            ActuatorCommand c;
            if (dec.push(buf[i]) && dec.packet().as(c) && c.t_us == bus.t_us) {
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
                 "motor0,motor1,motor2,motor3\n");
}

void log_row(std::FILE* f, const State& x, const SensorBus& s, const ActuatorCommand& c) {
    std::fprintf(f,
                 "%llu,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,"
                 "%u,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,"
                 "%ld,%ld,%.9g,%.9g,%.9g,%.9g,%d,%.9g,%.9g,%.9g,%.9g\n",
                 static_cast<unsigned long long>(s.t_us), x.p_ned.x, x.p_ned.y, x.p_ned.z, x.v_ned.x, x.v_ned.y,
                 x.v_ned.z, x.q.w, x.q.x, x.q.y, x.q.z, static_cast<unsigned>(s.fresh), s.imu.accel_frd.x,
                 s.imu.accel_frd.y, s.imu.accel_frd.z, s.imu.gyro_frd.x, s.imu.gyro_frd.y, s.imu.gyro_frd.z,
                 s.baro.pressure_pa, s.baro.temperature_c, s.mag.field_frd_ut.x, s.mag.field_frd_ut.y,
                 s.mag.field_frd_ut.z, static_cast<long>(s.gnss.lat_e7), static_cast<long>(s.gnss.lon_e7),
                 s.gnss.alt_m, s.gnss.vel_ned.x, s.gnss.vel_ned.y, s.gnss.vel_ned.z, s.gnss.fix ? 1 : 0,
                 c.motor[0], c.motor[1], c.motor[2], c.motor[3]);
}

}  // namespace

int main(int argc, char** argv) {
    bool sitl = false;
    float cmd_f = 0.85f;
    const char* port = nullptr;
    const char* log_path = nullptr;
    double seconds = -1.0;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        const bool has_value = i + 1 < argc;
        if (a == "--sitl") sitl = true;
        else if (a == "--cmd" && has_value) cmd_f = std::strtof(argv[++i], nullptr);
        else if (a == "--port" && has_value) port = argv[++i];
        else if (a == "--seconds" && has_value) seconds = std::strtod(argv[++i], nullptr);
        else if (a == "--log" && has_value) log_path = argv[++i];
        else return usage();
    }
    if (sitl == (port != nullptr) || !(seconds > 0.0)) return usage();

    std::unique_ptr<bridge::Endpoint> ep = sitl ? bridge::make_sitl(cmd_f) : bridge::open_serial(port);
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

    bridge::GzWorld world;
    std::string err;
    if (!world.connect(err)) {
        std::fprintf(stderr, "marv_bridge: %s\n", err.c_str());
        return 1;
    }

    const auto end_us = static_cast<std::uint64_t>(seconds * 1e6 + 0.5);
    link::Decoder dec;
    SensorBus bus{};
    State truth{};
    ActuatorCommand cmd{};
    std::uint64_t steps = 0;
    const auto wall0 = std::chrono::steady_clock::now();
    int rc = 0;
    while (bus.t_us < end_us) {
        if (!world.step(bus, truth, err) || !exchange(*ep, dec, bus, cmd, err)) {
            std::fprintf(stderr, "marv_bridge: %s\n", err.c_str());
            rc = 1;
            break;
        }
        world.command(cmd);
        if (log) log_row(log, truth, bus, cmd);
        ++steps;
    }
    const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - wall0).count();
    std::fprintf(stderr, "marv_bridge: %llu steps, sim %.3f s, wall %.3f s, %.3f sim-s per wall-s, %u bad frames\n",
                 static_cast<unsigned long long>(steps), static_cast<double>(bus.t_us) * 1e-6, wall,
                 static_cast<double>(bus.t_us) * 1e-6 / wall, dec.errors());
    if (log) std::fclose(log);
    return rc;
}
