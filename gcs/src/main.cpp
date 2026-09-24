// marv_gcs: the ground-control backend. Serves the web GCS (gcs/web/build) with its schema, and relays its setup
// requests to the flight controller and the replies and telemetry back, and launches SITL worlds (launcher.hpp).
//
//   marv_gcs [--udp HOST:PORT | --serial DEV] [--http PORT] [--web DIR]
//   marv_gcs --dump-schema          the /api/schema JSON, tab-indented, on stdout
//
// Default link: automatic, the flight controller on USB (/dev/serial/by-id/usb-MARV_MARV_flight_controller_*) while no
// sim runs, the sim's bridge (marv_bridge --ground on 127.0.0.1:14650) while one does. --udp and --serial fix it.
// Default --http 8765 on 127.0.0.1, --web the gcs/web build.
//
// One marv_gcs per user: it holds an flock on $XDG_RUNTIME_DIR/marv-gcs.lock (else /tmp/marv-gcs-UID.lock) holding its
// pid and URL, and a second one exits with status 1 naming the first. MARV_GCS_LOCK=PATH replaces the lock file: for
// tests only, so a test instance runs beside the user's.
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <string>

#include <boost/asio/io_context.hpp>
#include <boost/asio/signal_set.hpp>

#include <unistd.h>

#include "launcher.hpp"
#include "link.hpp"
#include "resources.hpp"
#include "schema.hpp"
#include "server.hpp"

namespace {

int usage() {
    std::fprintf(stderr,
                 "usage: marv_gcs [--udp HOST:PORT | --serial DEV] [--http PORT] [--web DIR]\n"
                 "       marv_gcs --dump-schema\n");
    return 2;
}

}  // namespace

int main(int argc, char** argv) {
    using namespace marv::gcs;
    LinkConfig cfg;
    int port = 8765;
    std::string web = MARV_GCS_WEB_DIR;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        const bool has_value = i + 1 < argc;
        if (a == "--dump-schema") {
            std::fputs(pretty(schema()).c_str(), stdout);
            return 0;
        } else if (a == "--udp" && has_value) {
            cfg.mode = LinkConfig::Mode::kUdp;
            cfg.target = argv[++i];
        } else if (a == "--serial" && has_value) {
            cfg.mode = LinkConfig::Mode::kSerial;
            cfg.target = argv[++i];
        } else if (a == "--http" && has_value) {
            port = std::atoi(argv[++i]);
            if (port <= 0 || port > 65535) return usage();
        } else if (a == "--web" && has_value) {
            web = argv[++i];
        } else {
            return usage();
        }
    }

    const std::string lock_path = instance_lock_path();
    std::string held, lock_error;
    const int lock = lock_instance(lock_path, held, lock_error);
    if (lock < 0) {
        if (!lock_error.empty()) {
            std::fprintf(stderr, "marv_gcs: %s\n", lock_error.c_str());
            return 1;
        }
        const std::size_t space = held.find(' ');
        const std::string pid = held.empty() ? "?" : held.substr(0, space);
        const std::string url = space == std::string::npos ? "its printed URL" : held.substr(space + 1);
        std::fprintf(stderr, "marv_gcs is already running (pid %s) at %s: open it, or stop it first (lock %s)\n",
                     pid.c_str(), url.c_str(), lock_path.c_str());
        return 1;
    }

    boost::asio::io_context io;
    try {
        Server server(io, "127.0.0.1", static_cast<unsigned short>(port), web);
        ResourceTargets targets;
        targets.self = ::getpid();
        targets.uid = ::getuid();
        targets.http = server.port();
        if (cfg.mode != LinkConfig::Mode::kSerial) {
            const int ground = std::atoi(cfg.target.substr(cfg.target.rfind(':') + 1).c_str());
            if (ground > 0 && ground <= 65535) targets.ground_udp = static_cast<unsigned>(ground);
        }
        server.set_resources(targets);
        const std::string url = "http://127.0.0.1:" + std::to_string(server.port()) + "/";
        write_instance(lock, std::to_string(::getpid()) + " " + url);
        Link link(io, cfg, [&server](const std::string& text, bool droppable) { server.broadcast(text, droppable); });
        server.set_link(&link);
        Launcher sim(
            io, cfg.mode == LinkConfig::Mode::kSerial,
            [&server](const std::string& text, bool droppable) { server.broadcast(text, droppable); },
            [&link](bool on) { link.sim(on); });
        server.set_sim(&sim);
        if (!link.start()) return 1;
        boost::asio::signal_set signals(io, SIGINT, SIGTERM);
        signals.async_wait([&io](const boost::system::error_code&, int) { io.stop(); });
        const char* mode = cfg.mode == LinkConfig::Mode::kAuto  ? "auto (USB, or the sim bridge at)"
                           : cfg.mode == LinkConfig::Mode::kUdp ? "udp"
                                                                : "serial";
        std::printf("marv_gcs: http://127.0.0.1:%u/  link %s %s  web %s\n", static_cast<unsigned>(server.port()), mode,
                    cfg.target.c_str(), web.c_str());
        std::fflush(stdout);
        io.run();
    } catch (const std::exception& e) {
        std::fprintf(stderr, "marv_gcs: %s\n", e.what());
        return 1;
    }
    return 0;
}
