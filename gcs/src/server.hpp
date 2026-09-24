// HTTP and WebSocket on one port: GET /api/schema, GET /api/link, GET /api/sim/airframes, GET /api/sim/status, WS /ws,
// and the files of the web build (index.html for any other path, so the single-page app routes). Runs on the io_context
// thread with the link.
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <boost/asio/io_context.hpp>
#include <boost/asio/ip/tcp.hpp>

namespace marv::gcs {

class Launcher;
class Link;
class WsSession;

class Server {
public:
    // Listens on address:port (port 0: any free port). Throws when it cannot.
    Server(boost::asio::io_context& io, const std::string& address, unsigned short port, std::string web_root);
    void set_link(Link* link) { link_ = link; }
    void set_sim(Launcher* sim) { sim_ = sim; }
    unsigned short port() const;

    // Sends text to every WebSocket client.
    void broadcast(const std::string& text, bool droppable);

    // Session side.
    Link& link() { return *link_; }
    Launcher* sim() { return sim_; }
    const std::string& web_root() const { return web_root_; }
    void add(const std::shared_ptr<WsSession>& s);
    void message(const std::string& text, const std::weak_ptr<WsSession>& from);

private:
    void accept();

    boost::asio::ip::tcp::acceptor acceptor_;
    std::string web_root_;
    Link* link_ = nullptr;
    Launcher* sim_ = nullptr;
    std::vector<std::weak_ptr<WsSession>> clients_;
};

}  // namespace marv::gcs
