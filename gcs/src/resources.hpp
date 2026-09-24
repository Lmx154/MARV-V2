// Who holds the rig's resources, from /proc (a root path, so tests scan a fixture tree): the flight controller's tty,
// the rig lock, the Gazebo sim server, the bridge's ground UDP port, the GCS's HTTP port and marv_gcs processes. Only
// the processes of one user are looked at. And the single-instance lock of marv_gcs.
#pragma once

#include <string>
#include <vector>

#include <boost/json/array.hpp>

namespace marv::gcs {

struct Holder {
    long pid = 0;
    std::string name;             // /proc/PID/comm
    std::string cmdline;          // argv, space-separated
    double started = 0;           // unix seconds
    long pgid = 0;
    bool self = false;            // marv_gcs itself
    bool child_of_launcher = false;  // a member of the launcher's sim process group
};

struct Resource {
    std::string id;  // fc_usb | rig_lock | sim | ground_udp | gcs
    std::string label, path;
    std::vector<Holder> holders;
};

struct ResourceTargets {
    std::string proc = "/proc";
    std::string fc_by_id;  // the flight controller's /dev/serial/by-id link, empty when none
    std::string fc_tty;    // ... resolved (/dev/ttyACM1)
    std::string rig_lock = "/tmp/marv-rig.lock";
    unsigned ground_udp = 14650;
    unsigned http = 8765;
    long self = 0;
    unsigned uid = 0;
    long launcher_group = -1;  // the launcher's sim process group, -1 when none
};

// fc_usb, rig_lock, sim, ground_udp, gcs, each with its holders among the processes of t.uid (by pid).
std::vector<Resource> scan_resources(const ResourceTargets& t);
boost::json::array resources_json(const std::vector<Resource>& r);
// The holder pid in r, or nullptr.
const Holder* find_holder(const std::vector<Resource>& r, long pid);
// Empty when pid may be terminated from the Development tab: a holder in r (a fresh scan), not marv_gcs itself, and a
// process of t.uid; else why not.
std::string terminate_refusal(const std::vector<Resource>& r, const ResourceTargets& t, long pid);
// The process's start time in unix seconds; negative once it has gone (or is a zombie).
double process_started(const std::string& proc, long pid);
// The first process of uid other than self with device (followed through symlinks) open; pid 0 when none.
Holder device_holder(const std::string& proc, const std::string& device, long self, unsigned uid);

// MARV_GCS_LOCK (for tests only), else $XDG_RUNTIME_DIR/marv-gcs.lock, else /tmp/marv-gcs-UID.lock.
std::string instance_lock_path();
// An fd holding an exclusive flock on path (close-on-exec; released when the process dies). -1 when another process
// holds it (held: the file's content, "PID URL") or on an error (err).
int lock_instance(const std::string& path, std::string& held, std::string& err);
// Replaces the lock file's content with text.
void write_instance(int fd, const std::string& text);

}  // namespace marv::gcs
