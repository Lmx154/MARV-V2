// The SITL launcher of the web Development tab, on the io_context thread: generates a world (worldgen.hpp) and runs
// scripts/sim.sh --world-file on it (Gazebo + marv_bridge --ground, in-process flight software or the Pico) as one child
// process group holding /tmp/marv-rig.lock, one sim at a time. Its output lines go to the WebSocket clients as sim_log.
#pragma once

#include <sys/types.h>

#include <chrono>
#include <string>

#include <boost/asio/io_context.hpp>
#include <boost/asio/posix/stream_descriptor.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/asio/streambuf.hpp>
#include <boost/json/array.hpp>
#include <boost/json/object.hpp>

#include "link.hpp"
#include "worldgen.hpp"

namespace marv::gcs {

class Launcher {
public:
    // serial_link: marv_gcs owns the Pico's port (--serial), so no sim may be launched.
    Launcher(boost::asio::io_context& io, bool serial_link, Broadcast broadcast);
    ~Launcher();  // stops a running sim: SIGTERM to its group, SIGKILL after 5 s
    Launcher(const Launcher&) = delete;
    Launcher& operator=(const Launcher&) = delete;

    // GET /api/sim/airframes: [{id, label, frame, source, specs{...}}], every catalog airframe that parses.
    boost::json::array airframes() const;
    // GET /api/sim/status: {running, starting, stopping, airframe, env, target, gui, started_at, pid, world, log}.
    boost::json::object status() const;
    // status() as a {type: "sim_status"} message.
    std::string status_message() const;
    // sim_launch {airframe, env, gui, target} | sim_stop. False when the message is neither.
    bool handle(const boost::json::object& msg, const Reply& reply);

private:
    using Clock = std::chrono::steady_clock;
    void launch(const boost::json::object& m, const Reply& reply);
    void stop(const Reply& reply);
    void read_out();
    void poll();
    void log(const std::string& line);
    void changed();

    std::string repo_, gen_dir_;
    bool serial_;
    Broadcast broadcast_;
    boost::asio::steady_timer timer_;
    boost::asio::posix::stream_descriptor out_;
    boost::asio::streambuf buf_;

    pid_t pid_ = -1;           // scripts/sim.sh, the leader of the sim's process group
    bool leader_done_ = false;  // ... reaped; its group may still have members
    bool running_ = false;      // the bridge is stepping the world (its log has grown)
    bool stopping_ = false;
    Clock::time_point kill_at_{};
    std::string airframe_, target_, world_, log_;
    worldgen::Env env_{};
    bool gui_ = false;
    double started_at_ = 0;  // unix seconds

    Clock::time_point window_{};  // sim_log rate limit: lines sent in the current second, lines held back
    int window_lines_ = 0;
    long suppressed_ = 0;
};

}  // namespace marv::gcs
