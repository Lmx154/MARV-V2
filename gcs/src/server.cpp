#include "server.hpp"

#include <signal.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <climits>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <deque>
#include <functional>
#include <filesystem>
#include <utility>

#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/json.hpp>

#include "launcher.hpp"
#include "link.hpp"
#include "schema.hpp"

namespace marv::gcs {
namespace asio = boost::asio;
namespace beast = boost::beast;
namespace http = beast::http;
namespace websocket = beast::websocket;
namespace json = boost::json;
using tcp = asio::ip::tcp;

// One WebSocket client: a read loop handing messages to the link, and a write queue.
class WsSession : public std::enable_shared_from_this<WsSession> {
public:
    WsSession(tcp::socket socket, Server& server) : ws_(std::move(socket)), server_(server) {}

    void run(http::request<http::string_body> req) {
        beast::get_lowest_layer(ws_).expires_never();
        ws_.set_option(websocket::stream_base::timeout::suggested(beast::role_type::server));
        ws_.text(true);
        req_ = std::move(req);
        ws_.async_accept(req_, [self = shared_from_this()](beast::error_code ec) {
            if (ec) return;
            self->server_.add(self);
            self->send(self->server_.link().link_message(), false);
            // Without a mission_state the page enables no mission control; a client joining while disarmed would
            // otherwise wait for the next change.
            self->send(self->server_.link().mission_message(), false);
            if (self->server_.sim()) self->send(self->server_.sim()->status_message(), false);
            self->read();
        });
    }

    // Queues text. A client more than a few frames behind skips droppable ones; one far behind is dropped.
    void send(const std::string& text, bool droppable) {
        if (closed_) return;
        if (droppable && out_.size() >= 4) return;
        if (out_.size() >= 4096) {
            closed_ = true;
            beast::get_lowest_layer(ws_).close();
            return;
        }
        out_.push_back(text);
        if (out_.size() == 1) write();
    }

    bool closed() const { return closed_; }
    bool resources = false;  // resources_subscribe: a scan every 2 s (the Development tab is open)

private:
    void read() {
        ws_.async_read(in_, [self = shared_from_this()](beast::error_code ec, std::size_t) {
            if (ec) {
                self->closed_ = true;
                return;
            }
            const std::string text = beast::buffers_to_string(self->in_.data());
            self->in_.consume(self->in_.size());
            self->server_.message(text, self);
            self->read();
        });
    }

    void write() {
        ws_.async_write(asio::buffer(out_.front()), [self = shared_from_this()](beast::error_code ec, std::size_t) {
            if (ec) {
                self->closed_ = true;
                self->out_.clear();
                return;
            }
            self->out_.pop_front();
            if (!self->out_.empty()) self->write();
        });
    }

    websocket::stream<beast::tcp_stream> ws_;
    Server& server_;
    http::request<http::string_body> req_;
    beast::flat_buffer in_;
    std::deque<std::string> out_;
    bool closed_ = false;
};

namespace {

const char* mime(const std::string& path) {
    const auto ext = std::filesystem::path(path).extension().string();
    if (ext == ".html") return "text/html; charset=utf-8";
    if (ext == ".js" || ext == ".mjs") return "text/javascript";
    if (ext == ".css") return "text/css";
    if (ext == ".json") return "application/json";
    if (ext == ".svg") return "image/svg+xml";
    if (ext == ".png") return "image/png";
    if (ext == ".ico") return "image/x-icon";
    if (ext == ".woff2") return "font/woff2";
    if (ext == ".txt") return "text/plain; charset=utf-8";
    return "application/octet-stream";
}

// One HTTP connection: requests until it closes, or its upgrade to a WebSocket on /ws.
class HttpSession : public std::enable_shared_from_this<HttpSession> {
public:
    HttpSession(tcp::socket socket, Server& server) : stream_(std::move(socket)), server_(server) {}

    void read() {
        req_ = {};
        stream_.expires_after(std::chrono::seconds(30));
        http::async_read(stream_, buf_, req_, [self = shared_from_this()](beast::error_code ec, std::size_t) {
            if (ec) {
                beast::error_code ignored;
                self->stream_.socket().shutdown(tcp::socket::shutdown_send, ignored);
                return;
            }
            self->respond();
        });
    }

private:
    void respond() {
        if (websocket::is_upgrade(req_)) {
            if (req_.target() == "/ws") std::make_shared<WsSession>(stream_.release_socket(), server_)->run(std::move(req_));
            return;
        }
        if (req_.method() != http::verb::get) return text(http::status::method_not_allowed, "text/plain", "GET only\n");
        std::string path(req_.target());
        path = path.substr(0, path.find('?'));
        if (path == "/api/schema") return text(http::status::ok, "application/json", schema_text());
        if (path == "/api/link") return text(http::status::ok, "application/json", json::serialize(server_.link().state()));
        if (path == "/api/resources") return text(http::status::ok, "application/json", json::serialize(server_.resources()));
        if (path == "/api/sim/airframes" && server_.sim())
            return text(http::status::ok, "application/json", json::serialize(server_.sim()->airframes()));
        if (path == "/api/sim/status" && server_.sim())
            return text(http::status::ok, "application/json", json::serialize(server_.sim()->status()));
        if (path.rfind("/api/", 0) == 0) return text(http::status::not_found, "text/plain", "no such api\n");
        if (path.find("..") != std::string::npos) return text(http::status::bad_request, "text/plain", "bad path\n");

        namespace fs = std::filesystem;
        std::string file = server_.web_root() + path;
        std::error_code fe;
        if (!fs::is_regular_file(file, fe)) file = server_.web_root() + "/index.html";  // the SPA routes the rest
        http::file_body::value_type body;
        beast::error_code ec;
        if (fs::is_regular_file(file, fe)) body.open(file.c_str(), beast::file_mode::scan, ec);
        else ec = beast::errc::make_error_code(beast::errc::no_such_file_or_directory);
        if (ec)
            return text(http::status::not_found, "text/plain",
                        "no web build at " + server_.web_root() + ": run scripts/gcs.sh (npm ci && npm run build)\n");
        http::response<http::file_body> res{std::piecewise_construct, std::make_tuple(std::move(body)),
                                            std::make_tuple(http::status::ok, req_.version())};
        res.set(http::field::content_type, mime(file));
        res.keep_alive(req_.keep_alive());
        res.prepare_payload();
        send(std::move(res));
    }

    void text(http::status status, const char* type, const std::string& body) {
        http::response<http::string_body> res{status, req_.version()};
        res.set(http::field::content_type, type);
        res.set(http::field::cache_control, "no-store");
        res.keep_alive(req_.keep_alive());
        res.body() = body;
        res.prepare_payload();
        send(std::move(res));
    }

    template <class Body> void send(http::response<Body>&& res) {
        auto sp = std::make_shared<http::response<Body>>(std::move(res));
        http::async_write(stream_, *sp, [self = shared_from_this(), sp](beast::error_code ec, std::size_t) {
            if (ec || sp->need_eof()) {
                beast::error_code ignored;
                self->stream_.socket().shutdown(tcp::socket::shutdown_send, ignored);
                return;
            }
            self->read();
        });
    }

    beast::tcp_stream stream_;
    Server& server_;
    beast::flat_buffer buf_;
    http::request<http::string_body> req_;
};

}  // namespace

Server::Server(asio::io_context& io, const std::string& address, unsigned short port, std::string web_root)
    : acceptor_(io, tcp::endpoint(asio::ip::make_address(address), port)),
      web_root_(std::move(web_root)),
      resources_timer_(io) {
    targets_.self = ::getpid();
    targets_.uid = ::getuid();
    targets_.http = this->port();
    accept();
    resources_tick();
}

unsigned short Server::port() const { return acceptor_.local_endpoint().port(); }

void Server::accept() {
    acceptor_.async_accept([this](beast::error_code ec, tcp::socket socket) {
        if (ec == asio::error::operation_aborted) return;
        if (!ec) std::make_shared<HttpSession>(std::move(socket), *this)->read();
        accept();
    });
}

void Server::add(const std::shared_ptr<WsSession>& s) { clients_.push_back(s); }

void Server::broadcast(const std::string& text, bool droppable) {
    clients_.erase(std::remove_if(clients_.begin(), clients_.end(),
                                  [](const std::weak_ptr<WsSession>& w) {
                                      const auto s = w.lock();
                                      return !s || s->closed();
                                  }),
                   clients_.end());
    for (const auto& w : clients_)
        if (const auto s = w.lock()) s->send(text, droppable);
}

void Server::message(const std::string& text, const std::weak_ptr<WsSession>& from) {
    const Reply reply = [from](const std::string& t) {
        if (const auto s = from.lock()) s->send(t, false);
    };
    boost::system::error_code ec;
    const json::value v = json::parse(text, ec);
    if (ec || !v.is_object()) {
        reply(json::serialize(json::object{{"type", "error"}, {"request", ""}, {"error", "not a JSON object"}}));
        return;
    }
    const json::object& m = v.get_object();
    const json::value* t = m.if_contains("type");
    if (t && t->is_string() && t->get_string() == "resources_request") return reply(resources_message());
    if (t && t->is_string() && t->get_string() == "resources_subscribe") {
        const json::value* on = m.if_contains("on");
        if (const auto s = from.lock()) s->resources = !on || !on->is_bool() || on->get_bool();
        return reply(resources_message());
    }
    if (t && t->is_string() && t->get_string() == "terminate") return terminate(m, reply);
    if (sim_ && sim_->handle(m, reply)) return;
    link_->handle(m, reply);
}

json::array Server::resources() {
    ResourceTargets t = targets_;
    t.fc_by_id = first_fc();
    t.fc_tty.clear();
    if (!t.fc_by_id.empty()) {
        char b[PATH_MAX];
        if (::realpath(t.fc_by_id.c_str(), b)) t.fc_tty = b;
    }
    if (sim_) {
        const json::object st = sim_->status();
        if (const json::value* pid = st.if_contains("pid"); pid && pid->is_int64()) t.launcher_group = pid->get_int64();
    }
    targets_.fc_by_id = t.fc_by_id;
    targets_.fc_tty = t.fc_tty;
    targets_.launcher_group = t.launcher_group;
    return resources_json(scan_resources(t));
}

std::string Server::resources_message() {
    return json::serialize(json::object{{"type", "resources"}, {"resources", resources()}});
}

void Server::resources_tick() {
    resources_timer_.expires_after(std::chrono::seconds(2));
    resources_timer_.async_wait([this](beast::error_code ec) {
        if (ec) return;
        std::string text;
        for (const auto& w : clients_) {
            const auto s = w.lock();
            if (!s || s->closed() || !s->resources) continue;
            if (text.empty()) text = resources_message();
            s->send(text, true);
        }
        resources_tick();
    });
}

void Server::terminate(const json::object& m, const Reply& reply) {
    const auto refuse = [&reply](const std::string& why) {
        reply(json::serialize(json::object{{"type", "error"}, {"request", "terminate"}, {"error", why}}));
    };
    const json::value* p = m.if_contains("pid");
    const long pid = p && p->is_int64() ? static_cast<long>(p->get_int64()) : p && p->is_uint64() ? static_cast<long>(p->get_uint64()) : 0;
    resources();  // the targets of a fresh scan
    const std::vector<Resource> scan = scan_resources(targets_);
    const std::string why = terminate_refusal(scan, targets_, pid);
    if (!why.empty()) return refuse(why);
    const Holder h = *find_holder(scan, pid);
    if (h.child_of_launcher && sim_) {
        std::printf("marv_gcs: terminate %ld (%s): a process of the launched sim: sim_stop\n", pid, h.name.c_str());
        sim_->handle(json::object{{"type", "sim_stop"}}, reply);
        return reply(resources_message());
    }
    if (::kill(static_cast<pid_t>(pid), SIGTERM) != 0) return refuse("kill " + std::to_string(pid) + ": " + std::strerror(errno));
    std::printf("marv_gcs: terminate %ld (%s): SIGTERM\n", pid, h.name.c_str());
    std::fflush(stdout);
    // Watch it every 100 ms: the scan goes to the client once it has gone; SIGKILL if it is still there after 5 s.
    struct Watch {
        asio::steady_timer timer;
        int polls = 0;
    };
    auto w = std::make_shared<Watch>(Watch{asio::steady_timer(resources_timer_.get_executor()), 0});
    const std::string proc = targets_.proc;
    const double started = h.started;
    const auto step = std::make_shared<std::function<void()>>();
    *step = [this, w, step, proc, pid, started, reply] {
        w->timer.expires_after(std::chrono::milliseconds(100));
        w->timer.async_wait([this, w, step, proc, pid, started, reply](beast::error_code ec) {
            if (ec) return;
            const double now = process_started(proc, pid);
            if (now < 0 || std::abs(now - started) > 0.5) {
                *step = nullptr;
                return reply(resources_message());
            }
            if (++w->polls == 50) {
                ::kill(static_cast<pid_t>(pid), SIGKILL);
                std::printf("marv_gcs: terminate %ld: still alive 5 s after SIGTERM: SIGKILL\n", pid);
                std::fflush(stdout);
            }
            if (w->polls > 70) {
                *step = nullptr;
                return reply(resources_message());
            }
            (*step)();
        });
    };
    (*step)();
}

}  // namespace marv::gcs
