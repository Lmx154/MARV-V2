// The ground's byte path to the flight controller. Every send is complete link frames; recv returns what
// arrived. UDP to the bridge today; a serial radio can stand behind the same two calls later.
#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>

namespace marv::ground {

class Transport {
public:
    virtual ~Transport() = default;
    // Sends n bytes of complete frames. Returns false on an I/O error.
    virtual bool send(const std::uint8_t* p, std::size_t n) = 0;
    // Waits at most timeout_ms for data. Returns the byte count, 0 on timeout, -1 on an I/O error.
    virtual long recv(std::uint8_t* p, std::size_t cap, int timeout_ms) = 0;
};

// host:port, e.g. 127.0.0.1:14650. Returns null, with a message on stderr, on failure.
std::unique_ptr<Transport> open_udp(const char* host_port);

// A serial device (the flight controller's USB CDC port), raw 8N1. Returns null, with a message on stderr, on failure.
std::unique_ptr<Transport> open_serial(const char* path);

}  // namespace marv::ground
