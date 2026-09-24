#include "resources.hpp"

#include <dirent.h>
#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <climits>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <functional>
#include <set>
#include <sstream>

#include <boost/json.hpp>

namespace marv::gcs {
namespace json = boost::json;

namespace {

std::string slurp(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    std::ostringstream s;
    s << f.rdbuf();
    return s.str();
}

std::string link_target(const std::string& path) {
    char b[PATH_MAX];
    const ssize_t n = ::readlink(path.c_str(), b, sizeof(b) - 1);
    return n > 0 ? std::string(b, static_cast<std::size_t>(n)) : std::string();
}

std::string resolve(const std::string& path) {
    char b[PATH_MAX];
    return ::realpath(path.c_str(), b) ? std::string(b) : path;
}

// The real uid in /proc/PID/status; -1 when unreadable.
long status_uid(const std::string& dir) {
    std::istringstream s(slurp(dir + "/status"));
    std::string line;
    while (std::getline(s, line))
        if (line.rfind("Uid:", 0) == 0) return std::strtol(line.c_str() + 4, nullptr, 10);
    return -1;
}

double boot_time(const std::string& proc) {
    std::istringstream s(slurp(proc + "/stat"));
    std::string line;
    while (std::getline(s, line))
        if (line.rfind("btime ", 0) == 0) return std::strtod(line.c_str() + 6, nullptr);
    return 0;
}

struct Stat {
    char state = 0;
    long pgid = 0;
    double started = -1;  // unix seconds
};

// /proc/PID/stat: the fields after the command name's closing parenthesis, from field 3 (state) on.
Stat read_stat(const std::string& proc, const std::string& dir) {
    Stat st;
    const std::string s = slurp(dir + "/stat");
    const std::size_t close = s.rfind(')');
    if (close == std::string::npos) return st;
    std::istringstream f(s.substr(close + 1));
    std::vector<std::string> v;
    for (std::string w; f >> w;) v.push_back(w);
    if (v.size() < 20) return st;
    st.state = v[0][0];
    st.pgid = std::strtol(v[2].c_str(), nullptr, 10);
    const long ticks = ::sysconf(_SC_CLK_TCK);
    st.started = boot_time(proc) + std::strtod(v[19].c_str(), nullptr) / static_cast<double>(ticks > 0 ? ticks : 100);
    return st;
}

std::vector<std::string> argv_of(const std::string& dir) {
    const std::string raw = slurp(dir + "/cmdline");
    std::vector<std::string> a;
    std::size_t at = 0;
    while (at < raw.size()) {
        const std::size_t end = std::min(raw.find('\0', at), raw.size());
        a.push_back(raw.substr(at, end - at));
        at = end + 1;
    }
    return a;
}

bool is_gz_sim(const std::vector<std::string>& a) {
    for (std::size_t i = 0; i + 1 < a.size(); ++i) {
        const std::size_t slash = a[i].rfind('/');
        if ((slash == std::string::npos ? a[i] : a[i].substr(slash + 1)) == "gz" && a[i + 1] == "sim") return true;
    }
    return false;
}

// The socket inodes bound to port in /proc/net/<table> (tcp: listening only).
void inodes(const std::string& proc, const char* table, unsigned port, bool listen, std::set<std::string>& out) {
    std::istringstream s(slurp(proc + "/net/" + table));
    std::string line;
    std::getline(s, line);  // the column header
    while (std::getline(s, line)) {
        std::istringstream f(line);
        std::vector<std::string> v;
        for (std::string w; f >> w;) v.push_back(w);
        if (v.size() < 10) continue;
        const std::size_t colon = v[1].rfind(':');
        if (colon == std::string::npos || std::strtoul(v[1].c_str() + colon + 1, nullptr, 16) != port) continue;
        if (listen && v[3] != "0A") continue;
        out.insert("socket:[" + v[9] + "]");
    }
}

// Calls fn(pid, directory) for every numeric entry of proc owned by uid.
void each_process(const std::string& proc, unsigned uid, const std::function<void(long, const std::string&)>& fn) {
    DIR* d = ::opendir(proc.c_str());
    if (!d) return;
    std::vector<long> pids;
    while (const dirent* e = ::readdir(d)) {
        char* end = nullptr;
        const long pid = std::strtol(e->d_name, &end, 10);
        if (pid > 0 && end && *end == '\0') pids.push_back(pid);
    }
    ::closedir(d);
    std::sort(pids.begin(), pids.end());
    for (long pid : pids) {
        const std::string dir = proc + "/" + std::to_string(pid);
        if (status_uid(dir) == static_cast<long>(uid)) fn(pid, dir);
    }
}

// The targets of /proc/PID/fd/*.
std::vector<std::string> fd_targets(const std::string& dir) {
    std::vector<std::string> out;
    DIR* d = ::opendir((dir + "/fd").c_str());
    if (!d) return out;
    while (const dirent* e = ::readdir(d)) {
        if (e->d_name[0] == '.') continue;
        std::string t = link_target(dir + "/fd/" + e->d_name);
        if (!t.empty()) out.push_back(std::move(t));
    }
    ::closedir(d);
    return out;
}

Holder make_holder(const std::string& proc, long pid, const std::string& dir, const std::vector<std::string>& argv,
                   const ResourceTargets& t) {
    Holder h;
    h.pid = pid;
    h.name = slurp(dir + "/comm");
    while (!h.name.empty() && (h.name.back() == '\n' || h.name.back() == '\r')) h.name.pop_back();
    for (const auto& a : argv) h.cmdline += (h.cmdline.empty() ? "" : " ") + a;
    if (h.cmdline.size() > 300) h.cmdline = h.cmdline.substr(0, 297) + "...";
    const Stat st = read_stat(proc, dir);
    h.started = st.started;
    h.pgid = st.pgid;
    h.self = pid == t.self;
    h.child_of_launcher = t.launcher_group > 0 && st.pgid == t.launcher_group;
    return h;
}

void error_text(std::string& err, const std::string& what, const std::string& path) {
    err = what + " " + path + ": " + std::strerror(errno);
}

}  // namespace

std::vector<Resource> scan_resources(const ResourceTargets& t) {
    std::vector<Resource> r = {
        {"fc_usb", "Flight controller USB", t.fc_tty, {}},
        {"rig_lock", "Rig lock", t.rig_lock, {}},
        {"sim", "Gazebo sim server", "gz sim", {}},
        {"ground_udp", "Sim bridge ground port", "udp/" + std::to_string(t.ground_udp), {}},
        {"gcs", "Ground control (marv_gcs, HTTP port)", "tcp/" + std::to_string(t.http), {}},
    };
    if (!t.fc_by_id.empty()) r[0].label += " (" + t.fc_by_id.substr(t.fc_by_id.rfind('/') + 1) + ")";
    std::set<std::string> udp, tcp;
    inodes(t.proc, "udp", t.ground_udp, false, udp);
    inodes(t.proc, "udp6", t.ground_udp, false, udp);
    inodes(t.proc, "tcp", t.http, true, tcp);
    inodes(t.proc, "tcp6", t.http, true, tcp);

    each_process(t.proc, t.uid, [&](long pid, const std::string& dir) {
        bool in[5] = {};
        const std::vector<std::string> argv = argv_of(dir);
        in[2] = is_gz_sim(argv);
        std::string comm = slurp(dir + "/comm");
        in[4] = comm == "marv_gcs\n" || comm == "marv_gcs";
        for (const std::string& f : fd_targets(dir)) {
            if (!t.fc_tty.empty() && f == t.fc_tty) in[0] = true;
            if (f == t.rig_lock) in[1] = true;
            if (udp.count(f)) in[3] = true;
            if (tcp.count(f)) in[4] = true;
        }
        if (!(in[0] || in[1] || in[2] || in[3] || in[4])) return;
        const Holder h = make_holder(t.proc, pid, dir, argv, t);
        for (int i = 0; i < 5; ++i)
            if (in[i]) r[static_cast<std::size_t>(i)].holders.push_back(h);
    });
    return r;
}

json::array resources_json(const std::vector<Resource>& r) {
    json::array out;
    for (const Resource& x : r) {
        json::array hs;
        for (const Holder& h : x.holders)
            hs.push_back(json::object{{"pid", h.pid},
                                      {"name", h.name},
                                      {"cmdline", h.cmdline},
                                      {"started", h.started},
                                      {"self", h.self},
                                      {"child_of_launcher", h.child_of_launcher}});
        out.push_back(json::object{{"id", x.id},
                                   {"label", x.label},
                                   {"path", x.path.empty() ? json::value(nullptr) : json::value(x.path)},
                                   {"holders", std::move(hs)}});
    }
    return out;
}

const Holder* find_holder(const std::vector<Resource>& r, long pid) {
    for (const Resource& x : r)
        for (const Holder& h : x.holders)
            if (h.pid == pid) return &h;
    return nullptr;
}

std::string terminate_refusal(const std::vector<Resource>& r, const ResourceTargets& t, long pid) {
    if (pid <= 0) return "want {pid: a holder's process id}";
    if (pid == t.self) return "pid " + std::to_string(pid) + " is this marv_gcs: stop it from its terminal (Ctrl-C)";
    const long uid = status_uid(t.proc + "/" + std::to_string(pid));
    if (uid < 0) return "no process " + std::to_string(pid);
    if (uid != static_cast<long>(t.uid)) return "pid " + std::to_string(pid) + " belongs to another user: not terminated";
    if (!find_holder(r, pid)) return "pid " + std::to_string(pid) + " holds none of the rig's resources: not terminated";
    return {};
}

double process_started(const std::string& proc, long pid) {
    const Stat st = read_stat(proc, proc + "/" + std::to_string(pid));
    return st.state == 0 || st.state == 'Z' || st.state == 'X' ? -1 : st.started;
}

Holder device_holder(const std::string& proc, const std::string& device, long self, unsigned uid) {
    const std::string dev = resolve(device);
    Holder found;
    each_process(proc, uid, [&](long pid, const std::string& dir) {
        if (found.pid != 0 || pid == self) return;
        for (const std::string& f : fd_targets(dir)) {
            if (f != dev && f != device) continue;
            ResourceTargets t;
            t.self = self;
            found = make_holder(proc, pid, dir, argv_of(dir), t);
            return;
        }
    });
    return found;
}

std::string instance_lock_path() {
    if (const char* p = std::getenv("MARV_GCS_LOCK"); p && *p) return p;
    if (const char* d = std::getenv("XDG_RUNTIME_DIR"); d && *d) return std::string(d) + "/marv-gcs.lock";
    return "/tmp/marv-gcs-" + std::to_string(::getuid()) + ".lock";
}

int lock_instance(const std::string& path, std::string& held, std::string& err) {
    const int fd = ::open(path.c_str(), O_RDWR | O_CREAT | O_CLOEXEC, 0600);
    if (fd < 0) {
        error_text(err, "open", path);
        return -1;
    }
    if (::flock(fd, LOCK_EX | LOCK_NB) != 0) {
        if (errno == EWOULDBLOCK) {
            held = slurp(path);
            while (!held.empty() && (held.back() == '\n' || held.back() == ' ')) held.pop_back();
        } else {
            error_text(err, "flock", path);
        }
        ::close(fd);
        return -1;
    }
    if (::ftruncate(fd, 0) != 0) {
        error_text(err, "truncate", path);
        ::close(fd);
        return -1;
    }
    return fd;
}

void write_instance(int fd, const std::string& text) {
    if (::ftruncate(fd, 0) != 0) return;
    const std::string line = text + "\n";
    if (::pwrite(fd, line.data(), line.size(), 0) < 0) return;
}

}  // namespace marv::gcs
