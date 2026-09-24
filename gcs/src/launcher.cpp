#include "launcher.hpp"

#include <fcntl.h>
#include <glob.h>
#include <signal.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <thread>
#include <utility>
#include <vector>

#include <boost/asio/buffers_iterator.hpp>
#include <boost/asio/read_until.hpp>
#include <boost/json.hpp>

extern char** environ;

namespace marv::gcs {
namespace asio = boost::asio;
namespace json = boost::json;
using namespace std::chrono_literals;

namespace {

constexpr const char* kRigLock = "/tmp/marv-rig.lock";
constexpr const char* kFcGlob = "/dev/serial/by-id/usb-MARV_MARV_flight_controller_*";
constexpr const char* kSeconds = "86400";  // a day: the sim runs until sim_stop
constexpr auto kPoll = 100ms;
constexpr auto kKillAfter = 5s;             // SIGTERM, then SIGKILL
constexpr int kLogPerSecond = 50;           // sim_log lines to the clients, at most

void error(const Reply& reply, const std::string& request, const std::string& what) {
    if (reply) reply(json::serialize(json::object{{"type", "error"}, {"request", request}, {"error", what}}));
}

std::string g17(double v) {
    char b[32];
    std::snprintf(b, sizeof(b), "%.17g", v);
    return b;
}

}  // namespace

Launcher::Launcher(asio::io_context& io, bool serial_link, Broadcast broadcast)
    : repo_(MARV_GCS_REPO_DIR),
      gen_dir_(std::string(MARV_GCS_BUILD_DIR) + "/sim/generated"),
      serial_(serial_link),
      broadcast_(std::move(broadcast)),
      timer_(io),
      out_(io) {}

Launcher::~Launcher() {
    timer_.cancel();
    if (pid_ <= 0) return;
    ::kill(-pid_, SIGTERM);
    const auto end = Clock::now() + kKillAfter;
    for (;;) {
        if (!leader_done_ && ::waitpid(pid_, nullptr, WNOHANG) == pid_) leader_done_ = true;
        if (leader_done_ && ::kill(-pid_, 0) != 0) break;
        if (Clock::now() >= end) {
            ::kill(-pid_, SIGKILL);
            if (!leader_done_) ::waitpid(pid_, nullptr, 0);
            break;
        }
        std::this_thread::sleep_for(50ms);
    }
}

json::array Launcher::airframes() const {
    json::array out;
    for (const auto& id : worldgen::airframe_ids(repo_)) {
        worldgen::Airframe a;
        std::string err;
        if (worldgen::load_airframe(repo_, id, a, err)) out.push_back(worldgen::airframe_json(a));
        else std::fprintf(stderr, "marv_gcs: airframe %s: %s\n", id.c_str(), err.c_str());
    }
    return out;
}

json::object Launcher::status() const {
    const bool live = pid_ > 0;
    json::object o{{"running", live && running_}, {"starting", live && !running_ && !stopping_},
                   {"stopping", live && stopping_}, {"airframe", nullptr},
                   {"env", nullptr},                {"target", nullptr},
                   {"gui", live && gui_},           {"started_at", nullptr},
                   {"pid", nullptr},                {"world", nullptr},
                   {"log", nullptr}};
    if (live) {
        o["airframe"] = airframe_;
        o["env"] = worldgen::env_json(env_);
        o["target"] = target_;
        o["started_at"] = started_at_;
        o["pid"] = pid_;
        o["world"] = world_;
        o["log"] = log_;
    }
    return o;
}

std::string Launcher::status_message() const {
    json::object o = status();
    o.emplace("type", "sim_status");
    return json::serialize(o);
}

bool Launcher::handle(const json::object& m, const Reply& reply) {
    const json::value* t = m.if_contains("type");
    if (!t || !t->is_string()) return false;
    if (t->get_string() == "sim_launch") launch(m, reply);
    else if (t->get_string() == "sim_stop") stop(reply);
    else return false;
    return true;
}

void Launcher::launch(const json::object& m, const Reply& reply) {
    const auto refuse = [&](const std::string& why) {
        error(reply, "sim_launch", why);
        if (reply) reply(status_message());
    };
    if (pid_ > 0)
        return refuse("a sim is already running (" + airframe_ + ", pid " + std::to_string(pid_) + "): sim_stop it first");
    if (serial_)
        return refuse("marv_gcs runs with --serial, so it owns the flight controller's port the sim's bridge needs: restart it with "
                      "--udp 127.0.0.1:14650 (scripts/gcs.sh) to launch a sim");
    const json::value* id = m.if_contains("airframe");
    if (!id || !id->is_string()) return refuse("want {airframe: id, env: {...}, gui: bool, target: \"host\"|\"fc\"}");
    worldgen::Airframe a;
    std::string err;
    if (!worldgen::load_airframe(repo_, std::string(id->get_string()), a, err)) return refuse(err);
    worldgen::Env env;
    if (const json::value* e = m.if_contains("env"); e && !e->is_null()) {
        if (!e->is_object()) return refuse("env: want an object");
        if (!worldgen::env_from_json(e->get_object(), env, err)) return refuse(err);
    }
    bool gui = false;
    if (const json::value* g = m.if_contains("gui")) {
        if (!g->is_bool()) return refuse("gui: want true or false");
        gui = g->get_bool();
    }
    std::string target = "host", port;
    if (const json::value* t = m.if_contains("target")) {
        if (!t->is_string() || (t->get_string() != "host" && t->get_string() != "fc"))
            return refuse("target: want \"host\" (firmware compiled for this computer) or \"fc\" (the connected flight controller)");
        target = std::string(t->get_string());
    }
    if (target == "fc") {
        glob_t g{};
        if (::glob(kFcGlob, 0, nullptr, &g) == 0 && g.gl_pathc > 0) port = g.gl_pathv[0];
        ::globfree(&g);
        if (port.empty()) return refuse(std::string("target fc: no ") + kFcGlob + " (is a flight controller running MARV firmware plugged in?)");
    }

    const std::string world = gen_dir_ + "/gcs-" + a.id + ".sdf", log = gen_dir_ + "/gcs-" + a.id + ".csv";
    std::string resource;
    std::vector<std::string> notes;
    if (!worldgen::generate(repo_, a, env, world, resource, notes, err)) return refuse(err);

    const int lock = ::open(kRigLock, O_RDWR | O_CREAT | O_CLOEXEC, 0666);
    if (lock < 0) return refuse(std::string("open ") + kRigLock + ": " + std::strerror(errno));
    if (::flock(lock, LOCK_EX | LOCK_NB) != 0) {
        const int e = errno;
        ::close(lock);
        return refuse(e == EWOULDBLOCK ? std::string("the rig lock ") + kRigLock +
                                             " is held (another sim, a flash or a test run is using Gazebo or the "
                                             "flight controller): wait for it to finish or stop it"
                                       : std::string("flock ") + kRigLock + ": " + std::strerror(e));
    }
    int fds[2];
    if (::pipe2(fds, O_CLOEXEC) != 0) {
        ::close(lock);
        return refuse(std::string("pipe: ") + std::strerror(errno));
    }
    std::remove(log.c_str());  // "running" is this run's log growing

    // Everything the child needs, prepared before the fork.
    std::vector<std::string> args = {repo_ + "/scripts/sim.sh", "--world-file", world, "--max-rot-velocity",
                                     g17(a.specs.max_rot_velocity), "--ground"};
    if (target == "fc") args.insert(args.end(), {"--port", port});
    else args.push_back("--sitl");
    args.insert(args.end(), {"--seconds", kSeconds, "--log", log});
    if (gui) args.push_back("--gui");
    std::vector<char*> argv;
    for (auto& s : args) argv.push_back(s.data());
    argv.push_back(nullptr);
    std::vector<std::string> env_strings;
    const char* old = std::getenv("GZ_SIM_RESOURCE_PATH");
    for (char** e = environ; *e; ++e)
        if (std::strncmp(*e, "GZ_SIM_RESOURCE_PATH=", 21) != 0) env_strings.emplace_back(*e);
    if (!resource.empty())
        env_strings.push_back("GZ_SIM_RESOURCE_PATH=" + resource + (old && *old ? ":" + std::string(old) : ""));
    else if (old)
        env_strings.push_back(std::string("GZ_SIM_RESOURCE_PATH=") + old);
    std::vector<char*> envp;
    for (auto& s : env_strings) envp.push_back(s.data());
    envp.push_back(nullptr);

    const pid_t pid = ::fork();
    if (pid == 0) {
        ::setpgid(0, 0);
        ::dup2(fds[1], 1);
        ::dup2(fds[1], 2);
        const int null = ::open("/dev/null", O_RDONLY);
        if (null >= 0) ::dup2(null, 0);
        ::fcntl(lock, F_SETFD, 0);  // every process of the group holds the lock until the last one exits
        sigset_t none;
        sigemptyset(&none);
        ::sigprocmask(SIG_SETMASK, &none, nullptr);
        ::signal(SIGPIPE, SIG_DFL);
        if (::chdir(repo_.c_str()) != 0) ::_exit(126);
        ::execve(argv[0], argv.data(), envp.data());
        ::_exit(127);
    }
    ::close(fds[1]);
    ::close(lock);
    if (pid < 0) {
        ::close(fds[0]);
        return refuse(std::string("fork: ") + std::strerror(errno));
    }
    ::setpgid(pid, pid);  // as the child does: a kill(-pid) before it ran must still reach the group

    pid_ = pid;
    leader_done_ = running_ = stopping_ = false;
    airframe_ = a.id;
    target_ = target;
    world_ = world;
    log_ = log;
    env_ = env;
    gui_ = gui;
    started_at_ = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
    out_.assign(fds[0]);
    window_ = Clock::now();
    window_lines_ = 0;
    suppressed_ = 0;
    changed();
    std::string line = "sim:";
    for (const auto& s : args) line += " " + s;
    this->log(line);
    for (const auto& n : notes) this->log("sim: " + n);
    read_out();
    timer_.expires_after(kPoll);
    timer_.async_wait([this](const boost::system::error_code& ec) {
        if (!ec) poll();
    });
}

void Launcher::stop(const Reply& reply) {
    if (pid_ <= 0) {
        error(reply, "sim_stop", "no sim is running");
        if (reply) reply(status_message());
        return;
    }
    if (stopping_) {
        if (reply) reply(status_message());
        return;
    }
    ::kill(-pid_, SIGTERM);
    stopping_ = true;
    kill_at_ = Clock::now() + kKillAfter;
    log("sim: stopping: SIGTERM to process group " + std::to_string(pid_));
    changed();
}

void Launcher::read_out() {
    asio::async_read_until(out_, buf_, '\n', [this](const boost::system::error_code& ec, std::size_t n) {
        const auto begin = asio::buffers_begin(buf_.data());
        if (!ec) {
            std::string line(begin, begin + static_cast<std::ptrdiff_t>(n));
            buf_.consume(n);
            while (!line.empty() && (line.back() == '\n' || line.back() == '\r')) line.pop_back();
            log(line);
            return read_out();
        }
        if (ec == asio::error::operation_aborted) return;
        if (buf_.size() > 0) {
            const std::string rest(begin, asio::buffers_end(buf_.data()));
            buf_.consume(buf_.size());
            log(rest);
        }
        out_.close();  // every writer has gone; poll() reaps the group
    });
}

void Launcher::poll() {
    if (pid_ <= 0) return;
    const auto now = Clock::now();
    int st = 0;
    if (!leader_done_ && ::waitpid(pid_, &st, WNOHANG) == pid_) {
        leader_done_ = true;
        log(WIFEXITED(st) ? "sim: scripts/sim.sh exited with status " + std::to_string(WEXITSTATUS(st))
                          : "sim: scripts/sim.sh ended by signal " + std::to_string(WTERMSIG(st)));
    }
    struct stat sb {};
    if (!running_ && !stopping_ && !leader_done_ && ::stat(log_.c_str(), &sb) == 0 && sb.st_size > 0) {
        running_ = true;
        log("sim: the bridge is stepping the world; its log: " + log_);
        changed();
    }
    log("");  // only rolls the rate-limit window
    const bool members = ::kill(-pid_, 0) == 0;
    if (leader_done_ && !members) {
        pid_ = -1;
        running_ = stopping_ = false;
        boost::system::error_code ignored;
        out_.close(ignored);
        log("sim: stopped; no process of its group is left");
        changed();
        return;
    }
    if (leader_done_ && !stopping_) {
        // scripts/sim.sh ended by itself and left processes behind.
        ::kill(-pid_, SIGTERM);
        stopping_ = true;
        kill_at_ = now + kKillAfter;
        log("sim: scripts/sim.sh left processes behind: SIGTERM to process group " + std::to_string(pid_));
        changed();
    }
    if (stopping_ && now >= kill_at_) {
        ::kill(-pid_, SIGKILL);
        log("sim: process group " + std::to_string(pid_) + " still alive 5 s after SIGTERM: SIGKILL");
        kill_at_ = now + 1s;
    }
    timer_.expires_after(kPoll);
    timer_.async_wait([this](const boost::system::error_code& ec) {
        if (!ec) poll();
    });
}

// One output line to the clients (and stdout), at most kLogPerSecond a second; an empty line only rolls the window.
void Launcher::log(const std::string& line) {
    const auto emit = [this](const std::string& l) {
        std::printf("%s\n", l.c_str());
        std::fflush(stdout);
        broadcast_(json::serialize(json::object{{"type", "sim_log"}, {"line", l}}), false);
    };
    const auto now = Clock::now();
    if (now - window_ >= 1s) {
        window_ = now;
        window_lines_ = 0;
        if (suppressed_ > 0) {
            emit("sim: " + std::to_string(suppressed_) + " output lines held back (at most " +
                 std::to_string(kLogPerSecond) + " a second go to the page)");
            suppressed_ = 0;
            ++window_lines_;
        }
    }
    if (line.empty()) return;
    if (window_lines_ >= kLogPerSecond) {
        ++suppressed_;
        return;
    }
    ++window_lines_;
    emit(line);
}

void Launcher::changed() { broadcast_(status_message(), false); }

}  // namespace marv::gcs
