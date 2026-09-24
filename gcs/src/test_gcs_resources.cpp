// The resource scanner (resources.hpp) on a fixture /proc tree: fd symlinks, net tables, other users' processes and a
// zombie; terminate validation; the single-instance lock; and the automatic link reporting a flight controller another
// process holds as busy instead of opening it.
#include <sys/stat.h>
#include <unistd.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include <boost/asio/io_context.hpp>
#include <boost/json.hpp>

#include "link.hpp"
#include "resources.hpp"

namespace {

int g_fails = 0;
#define CHECK(c)                                                                  \
    do {                                                                          \
        if (!(c)) {                                                               \
            std::fprintf(stderr, "%s:%d: CHECK(%s) failed\n", __FILE__, __LINE__, #c); \
            ++g_fails;                                                            \
        }                                                                         \
    } while (0)

namespace fs = std::filesystem;
using namespace marv::gcs;

constexpr unsigned kUid = 4242;

void put(const fs::path& p, const std::string& text) {
    fs::create_directories(p.parent_path());
    std::ofstream(p, std::ios::binary) << text;
}

void process(const fs::path& proc, long pid, unsigned uid, const std::string& comm, const std::vector<std::string>& argv,
             long pgid, char state, long start_ticks, const std::vector<std::string>& fds) {
    const fs::path d = proc / std::to_string(pid);
    put(d / "status", "Name:\t" + comm + "\nUmask:\t0022\nUid:\t" + std::to_string(uid) + "\t" + std::to_string(uid) +
                          "\t" + std::to_string(uid) + "\t" + std::to_string(uid) + "\n");
    put(d / "comm", comm + "\n");
    std::string cmd;
    for (const auto& a : argv) cmd += a + std::string(1, '\0');
    put(d / "cmdline", cmd);
    put(d / "stat", std::to_string(pid) + " (" + comm + ") " + state + " 1 " + std::to_string(pgid) + " " +
                        std::to_string(pgid) + " 0 -1 4194560 0 0 0 0 0 0 0 0 20 0 1 0 " + std::to_string(start_ticks) +
                        " 1000 100 18446744073709551615\n");
    fs::create_directories(d / "fd");
    for (std::size_t i = 0; i < fds.size(); ++i) fs::create_symlink(fds[i], d / "fd" / std::to_string(i + 3));
}

const Resource& by_id(const std::vector<Resource>& r, const std::string& id) {
    for (const auto& x : r)
        if (x.id == id) return x;
    std::fprintf(stderr, "no resource %s\n", id.c_str());
    std::abort();
}

std::vector<long> pids(const Resource& r) {
    std::vector<long> p;
    for (const auto& h : r.holders) p.push_back(h.pid);
    return p;
}

void test_scan(const fs::path& root) {
    const fs::path proc = root / "proc";
    const std::string rig = (root / "rig.lock").string();
    put(proc / "stat", "cpu  1 2 3\nbtime 1000000000\nprocesses 5\n");
    // 14650 = 0x393A, 8765 = 0x223D. Only the listening tcp socket (0A) is the HTTP port's holder.
    const std::string head = "  sl  local_address rem_address   st tx_queue rx_queue tr tm->when retrnsmt   uid  timeout inode\n";
    put(proc / "net" / "udp", head + "   1: 0100007F:393A 00000000:0000 07 00000000:00000000 00:00000000 00000000  4242        0 5001 2 0 0\n" +
                                  "   2: 0100007F:9C40 0100007F:393A 01 00000000:00000000 00:00000000 00000000  4242        0 5003 2 0 0\n");
    put(proc / "net" / "udp6", head);
    put(proc / "net" / "tcp", head + "   0: 0100007F:223D 00000000:0000 0A 00000000:00000000 00:00000000 00000000  4242        0 6001 1 0 100\n" +
                                  "   1: 0100007F:223D 0100007F:D000 01 00000000:00000000 00:00000000 00000000  4242        0 6002 1 0 20\n");
    put(proc / "net" / "tcp6", head);
    process(proc, 100, kUid, "marv_bridge", {"build/native/bridge/marv_bridge", "--port", "/dev/ttyACM1"}, 150, 'S', 500,
            {"/dev/ttyACM1", "socket:[5001]", rig, "/dev/null"});
    process(proc, 200, kUid, "ruby", {"ruby", "/usr/bin/gz", "sim", "-s", "-r", "world.sdf"}, 150, 'S', 600, {"/dev/null"});
    process(proc, 300, kUid, "marv_gcs", {"marv_gcs", "--web", "gcs/web/build"}, 300, 'S', 700,
            {"socket:[6001]", "socket:[6002]", "socket:[5003]"});
    process(proc, 400, 0, "minicom", {"minicom", "-D", "/dev/ttyACM1"}, 400, 'S', 800, {"/dev/ttyACM1", rig});
    process(proc, 500, kUid, "bash", {"bash"}, 500, 'S', 900, {"/dev/pts/1"});
    process(proc, 600, kUid, "flock", {"flock", rig, "sleep", "300"}, 600, 'Z', 1000, {});
    process(proc, 700, kUid, "marv_gcs", {"marv_gcs", "--http", "9000"}, 700, 'S', 1100, {"socket:[7777]"});
    fs::create_directories(proc / "self");  // not a pid

    ResourceTargets t;
    t.proc = proc.string();
    t.fc_by_id = "/dev/serial/by-id/usb-MARV_MARV_flight_controller_123-if00";
    t.fc_tty = "/dev/ttyACM1";
    t.rig_lock = rig;
    t.self = 300;
    t.uid = kUid;
    t.launcher_group = 150;
    const auto r = scan_resources(t);
    CHECK(r.size() == 5);
    CHECK(pids(by_id(r, "fc_usb")) == std::vector<long>{100});  // not 400: another user
    CHECK(by_id(r, "fc_usb").path == "/dev/ttyACM1");
    CHECK(by_id(r, "fc_usb").label.find("usb-MARV_MARV_flight_controller_123-if00") != std::string::npos);
    CHECK(pids(by_id(r, "rig_lock")) == std::vector<long>{100});
    CHECK(pids(by_id(r, "sim")) == std::vector<long>{200});
    CHECK(pids(by_id(r, "ground_udp")) == std::vector<long>{100});  // not 300: its socket only talks to the port
    CHECK((pids(by_id(r, "gcs")) == std::vector<long>{300, 700}));
    const Holder& gz = by_id(r, "sim").holders.at(0);
    CHECK(gz.name == "ruby");
    CHECK(gz.cmdline == "ruby /usr/bin/gz sim -s -r world.sdf");
    CHECK(gz.child_of_launcher && gz.pgid == 150 && !gz.self);
    const long hz = ::sysconf(_SC_CLK_TCK);
    CHECK(std::fabs(gz.started - (1e9 + 600.0 / static_cast<double>(hz))) < 1e-6);
    CHECK(by_id(r, "gcs").holders.at(0).self && !by_id(r, "gcs").holders.at(1).self);
    CHECK(!by_id(r, "gcs").holders.at(0).child_of_launcher);

    const auto j = resources_json(r);
    CHECK(j.size() == 5 && j[0].as_object().at("id") == "fc_usb");
    CHECK(j[4].as_object().at("holders").as_array()[0].as_object().at("self") == true);
    CHECK(j[2].as_object().at("holders").as_array()[0].as_object().at("child_of_launcher") == true);

    // No flight controller plugged in: its resource has no path and no holders.
    ResourceTargets none = t;
    none.fc_by_id.clear();
    none.fc_tty.clear();
    const auto r2 = scan_resources(none);
    CHECK(by_id(r2, "fc_usb").holders.empty() && by_id(r2, "fc_usb").path.empty());
    CHECK(resources_json(r2)[0].as_object().at("path").is_null());

    // Terminate: holders only, never marv_gcs itself, never another user's process.
    CHECK(terminate_refusal(r, t, 100).empty());
    CHECK(terminate_refusal(r, t, 200).empty());
    CHECK(terminate_refusal(r, t, 700).empty());  // another marv_gcs
    CHECK(terminate_refusal(r, t, 300).find("this marv_gcs") != std::string::npos);
    CHECK(terminate_refusal(r, t, 400).find("another user") != std::string::npos);
    CHECK(terminate_refusal(r, t, 500).find("holds none") != std::string::npos);
    CHECK(terminate_refusal(r, t, 99999).find("no process") != std::string::npos);
    CHECK(!terminate_refusal(r, t, 0).empty() && !terminate_refusal(r, t, -1).empty());
    CHECK(find_holder(r, 700) && !find_holder(r, 500));

    CHECK(process_started(t.proc, 100) > 0);
    CHECK(process_started(t.proc, 600) < 0);  // a zombie has gone
    CHECK(process_started(t.proc, 99999) < 0);

    CHECK(device_holder(t.proc, "/dev/ttyACM1", 300, kUid).pid == 100);
    CHECK(device_holder(t.proc, "/dev/ttyACM1", 300, kUid).name == "marv_bridge");
    CHECK(device_holder(t.proc, "/dev/ttyACM1", 100, kUid).pid == 0);  // itself, and 400 is another user's
    CHECK(device_holder(t.proc, "/dev/ttyACM9", 300, kUid).pid == 0);
}

void test_lock(const fs::path& root) {
    ::setenv("MARV_GCS_LOCK", (root / "gcs.lock").c_str(), 1);
    const std::string path = instance_lock_path();
    CHECK(path == (root / "gcs.lock").string());
    std::string held, err;
    const int a = lock_instance(path, held, err);
    CHECK(a >= 0 && err.empty());
    write_instance(a, "123 http://127.0.0.1:8765/");
    const int b = lock_instance(path, held, err);
    CHECK(b < 0 && err.empty());
    CHECK(held == "123 http://127.0.0.1:8765/");
    ::close(a);
    held.clear();
    const int c = lock_instance(path, held, err);
    CHECK(c >= 0 && held.empty());
    ::close(c);
    CHECK(lock_instance((root / "no" / "dir" / "x.lock").string(), held, err) < 0 && !err.empty());
    ::unsetenv("MARV_GCS_LOCK");
    CHECK(instance_lock_path() != path);
}

void test_busy_link() {
    boost::asio::io_context io;
    LinkConfig cfg;
    int opened = 0;
    cfg.scan = [] { return std::string("/dev/serial/by-id/usb-MARV_MARV_flight_controller_1-if00"); };
    cfg.holder = [](const std::string&) {
        Holder h;
        h.pid = 4321;
        h.name = "marv_bridge";
        return h;
    };
    cfg.open_serial = [&opened](const char*) -> std::unique_ptr<marv::ground::Transport> {
        ++opened;
        return nullptr;
    };
    std::vector<std::string> sent;
    Link link(io, cfg, [&sent](const std::string& text, bool) { sent.push_back(text); });
    CHECK(link.start());
    CHECK(opened == 0);
    const auto s = link.state();
    CHECK(s.at("via") == "busy");
    CHECK(s.at("target") == "/dev/serial/by-id/usb-MARV_MARV_flight_controller_1-if00");
    CHECK(s.at("holder").as_object().at("name") == "marv_bridge" && s.at("holder").as_object().at("pid") == 4321);
    CHECK(!sent.empty() && sent.back().find("\"busy\"") != std::string::npos);
}

}  // namespace

int main() {
    const fs::path root = fs::temp_directory_path() / ("marv-test-resources-" + std::to_string(::getpid()));
    fs::remove_all(root);
    test_scan(root);
    test_lock(root);
    test_busy_link();
    fs::remove_all(root);
    if (g_fails) {
        std::fprintf(stderr, "test_gcs_resources: %d failed\n", g_fails);
        return 1;
    }
    std::printf("test_gcs_resources: ok\n");
    return 0;
}
