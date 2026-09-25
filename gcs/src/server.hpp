// HTTP and WebSocket on one port: GET /api/schema, GET /api/link, GET /api/sim/airframes, GET /api/sim/status,
// GET /api/resources (resources.hpp: who holds the rig's resources), GET /api/radio/devices and GET|PUT|DELETE
// /api/radio/config (radio.hpp: the pilot's joysticks and their configs), WS /ws,
// and the files of the web build (index.html for any other path, so the single-page app routes). Runs on the io_context
// thread with the link.
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <boost/asio/io_context.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/json/array.hpp>
#include <boost/json/object.hpp>

#include "link.hpp"
#include "resources.hpp"

namespace marv::gcs {

class Launcher;
class Link;
class Radio;
class WsSession;

class Server {
public:
    // Listens on address:port (port 0: any free port). Throws when it cannot.
    Server(boost::asio::io_context& io, const std::string& address, unsigned short port, std::string web_root);
    void set_link(Link* link) { link_ = link; }
    void set_sim(Launcher* sim) { sim_ = sim; }
    void set_radio(Radio* radio) { radio_ = radio; }
    // What the resource scan looks for; fc_by_id, fc_tty and launcher_group are filled in on every scan.
    void set_resources(const ResourceTargets& t) { targets_ = t; }
    unsigned short port() const;

    // Sends text to every WebSocket client.
    void broadcast(const std::string& text, bool droppable);

    // Session side.
    Link& link() { return *link_; }
    Launcher* sim() { return sim_; }
    Radio* radio() { return radio_; }
    const std::string& web_root() const { return web_root_; }
    void add(const std::shared_ptr<WsSession>& s);
    void message(const std::string& text, const std::weak_ptr<WsSession>& from);
    // A fresh scan, as JSON; {type: "resources", resources: [...]}.
    boost::json::array resources();
    std::string resources_message();

private:
    void accept();
    // {type: "terminate", pid}: SIGTERM to a holder of a fresh scan (not marv_gcs itself), SIGKILL 5 s later if it is
    // still alive; the launcher's own sim group is stopped as sim_stop does.
    void terminate(const boost::json::object& m, const Reply& reply);
    void resources_tick();
    // {type: "radio_subscribe", device: name | null}: the device's radio messages to this client, or none.
    void radio_subscribe(const boost::json::object& m, const std::weak_ptr<WsSession>& from, const Reply& reply);
    void radio_tick();

    boost::asio::ip::tcp::acceptor acceptor_;
    std::string web_root_;
    Link* link_ = nullptr;
    Launcher* sim_ = nullptr;
    Radio* radio_ = nullptr;
    std::vector<std::weak_ptr<WsSession>> clients_;
    ResourceTargets targets_;
    boost::asio::steady_timer resources_timer_;  // every 2 s: a scan to the clients with resources_subscribe on
    boost::asio::steady_timer radio_timer_;      // 20 Hz: radio messages to the clients with radio_subscribe on
};

}  // namespace marv::gcs
