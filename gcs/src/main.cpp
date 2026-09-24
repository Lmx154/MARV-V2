// marv_gcs: the ground-control backend. Serves the web GCS (gcs/web/build) with its schema, and relays its setup
// requests to the flight controller and the replies and telemetry back.
//
//   marv_gcs [--udp HOST:PORT | --serial DEV] [--http PORT] [--web DIR]
//   marv_gcs --dump-schema          the /api/schema JSON, tab-indented, on stdout
//
// Default --udp 127.0.0.1:14650 (marv_bridge --ground), --http 8765 on 127.0.0.1, --web the gcs/web build.
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <string>

#include <boost/asio/io_context.hpp>
#include <boost/asio/signal_set.hpp>

#include "link.hpp"
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
            cfg.serial = false;
            cfg.target = argv[++i];
        } else if (a == "--serial" && has_value) {
            cfg.serial = true;
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

    boost::asio::io_context io;
    try {
        Server server(io, "127.0.0.1", static_cast<unsigned short>(port), web);
        Link link(io, cfg, [&server](const std::string& text, bool droppable) { server.broadcast(text, droppable); });
        server.set_link(&link);
        if (!link.start()) return 1;
        boost::asio::signal_set signals(io, SIGINT, SIGTERM);
        signals.async_wait([&io](const boost::system::error_code&, int) { io.stop(); });
        std::printf("marv_gcs: http://127.0.0.1:%u/  link %s %s  web %s\n", static_cast<unsigned>(server.port()),
                    cfg.serial ? "serial" : "udp", cfg.target.c_str(), web.c_str());
        std::fflush(stdout);
        io.run();
    } catch (const std::exception& e) {
        std::fprintf(stderr, "marv_gcs: %s\n", e.what());
        return 1;
    }
    return 0;
}
