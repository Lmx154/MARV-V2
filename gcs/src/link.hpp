// The GCS's link to the flight controller, run on the io_context thread: link frames (protocol.hpp) over
// ground::Transport, one request in flight at a time. A request's one reply is its acknowledgement; none within
// 500 ms sends it once more, none again reports "no reply". Replies and telemetry go to the WebSocket clients as JSON.
// Mission commands go to the executor (mission.hpp); its MissionCommand frames go out at 20 Hz beside the request queue.
#pragma once

#include <sys/types.h>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <boost/asio/io_context.hpp>
#include <boost/asio/posix/stream_descriptor.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/asio/streambuf.hpp>
#include <boost/json/object.hpp>

#include <marv/link/protocol.hpp>

#include "mission.hpp"
#include "transport.hpp"

namespace marv::gcs {

// Sends one text frame to one client; a no-op once that client has gone.
using Reply = std::function<void(const std::string&)>;
// Sends one text frame to every client. A client that is behind skips droppable frames (telemetry).
using Broadcast = std::function<void(const std::string&, bool droppable)>;

// The first /dev/serial/by-id/usb-MARV_MARV_flight_controller_* (the firmware's own USB name), empty when none.
std::string first_fc();

struct LinkConfig {
    // kAuto: the flight controller on USB (scanned for every 1 s) while no sim runs, the sim's bridge while one does.
    enum class Mode : std::uint8_t { kAuto, kUdp, kSerial };
    using Opener = std::function<std::unique_ptr<ground::Transport>(const char*)>;
    Mode mode = Mode::kAuto;
    std::string target = "127.0.0.1:14650";  // HOST:PORT of the bridge's --ground (kUdp, kAuto), or the serial device
    std::function<std::string()> scan = first_fc;  // kAuto: the device to open, empty when none
    Opener open_serial = ground::open_serial;
    Opener open_udp = ground::open_udp;
};

class Link {
public:
    Link(boost::asio::io_context& io, LinkConfig cfg, Broadcast broadcast);
    ~Link();
    Link(const Link&) = delete;
    Link& operator=(const Link&) = delete;

    // Opens the link and starts polling it. False when a UDP address is unusable.
    bool start();
    // One client message: request_setup | set_param | set_kind | load_factory | save | reset | reboot | flash, or a
    // mission command: arm | disarm | climb | mission_start | rth | land.
    void handle(const boost::json::object& msg, const Reply& reply);
    // kAuto: a sim is about to start its bridge (on: the serial port is closed before this returns and the link goes to
    // the bridge's --ground) or has ended (off: back to scanning USB). A no-op in the fixed modes.
    void sim(bool on);
    // GET /api/link: {mode, via, target, connected, schema_ok, schema_hash, header, error?, flashing}.
    boost::json::object state() const;
    // state() as a {type: "link"} message.
    std::string link_message() const;
    // The current mission_state, for a client that has just connected (the broadcast only goes out on a change).
    std::string mission_message() const;

private:
    // Broadcasts the link state and re-sends mission_state on the next tick: a page drops its mission state whenever
    // the link goes down (a USB <-> sim switch), and an idle executor would otherwise never broadcast again.
    void broadcast_link();

public:

private:
    using Clock = std::chrono::steady_clock;
    enum class Kind : std::uint8_t { kSetup, kSetParam, kSetKind, kSave, kReset };
    struct Request {
        Kind kind;
        std::string name;
        std::vector<std::uint8_t> frame;
        std::uint16_t index;  // kSetParam
        Reply reply;          // empty for the link's own requests
        link::SetKind set{};  // kSetKind: refused when the header does not hold it
    };

    template <class T> void enqueue(Kind kind, const std::string& name, const T& msg, const Reply& reply,
                                    std::uint16_t index = 0);
    void issue();
    void transmit();
    void complete();
    void fail(const std::string& why);
    void fail_all(const std::string& why);
    void tick();
    void poll();
    void receive(const link::Packet& p);
    void on_header(const link::SetupHeader& h);
    void on_value(const link::ParamValue& v);
    bool send(const std::uint8_t* p, std::size_t n);
    bool open();
    void close(const char* why);
    void set_connected(bool connected);
    std::string edit_refusal() const;
    std::string setup_message(bool with_values) const;
    std::string telemetry_message() const;
    void error(const Reply& reply, const std::string& request, const std::string& what) const;
    std::string kind_refusal(const link::SetKind& k, const link::SetupHeader& h) const;
    void mission_request(const std::string& type, const boost::json::object& m, const Reply& reply);
    void mission_tick(Clock::time_point now);
    void flash(const Reply& reply);
    void read_flash();
    void flash_log(const std::string& line) const;

    LinkConfig cfg_;
    Broadcast broadcast_;
    boost::asio::steady_timer timer_;
    std::unique_ptr<ground::Transport> tx_;
    bool serial_ = false;                 // tx_ is a serial device ...
    std::string at_;                      // ... or HOST:PORT: the path or address it was opened on
    bool sim_ = false;                    // kAuto: a sim's bridge holds the flight controller
    link::Decoder dec_;

    std::deque<Request> queue_;
    int tries_ = 0;
    Clock::time_point deadline_{};
    bool rx_header_ = false;              // the head request's header arrived
    std::vector<float> rx_values_;        // its values, by index
    std::size_t rx_count_ = 0;

    bool connected_ = false;              // the last request was answered
    bool have_header_ = false;
    link::SetupHeader header_{};
    std::vector<float> values_;           // staged values as last reported
    Telemetry tlm_{};
    bool tlm_pending_ = false;
    bool armed_ = false;
    bool have_armed_ = false;
    float motor_[kMotorCount] = {};       // the last ActuatorCommand

    Mission mission_;
    Clock::time_point next_mission_{};    // the next 20 Hz period
    Clock::time_point last_state_{};      // the last mission_state broadcast
    std::string state_key_;               // ... and its content without dist_m

    Clock::time_point last_rx_{};
    Clock::time_point last_tx_{};
    Clock::time_point last_tlm_{};
    Clock::time_point next_probe_{};
    Clock::time_point reopen_at_{};

    pid_t flash_pid_ = -1;
    bool verify_flash_ = false;
    boost::asio::posix::stream_descriptor flash_out_;
    boost::asio::streambuf flash_buf_;
};

}  // namespace marv::gcs
