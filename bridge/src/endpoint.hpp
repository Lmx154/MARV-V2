// Where the flight software runs: on the Pico behind a serial port, or in this process. Either way the
// bridge only exchanges wire-protocol bytes with it, so the lockstep loop cannot tell which it has.
#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>

namespace marv::bridge {

class Endpoint {
public:
    virtual ~Endpoint() = default;
    // Writes all n bytes. Returns false on an I/O error.
    virtual bool write(const std::uint8_t* p, std::size_t n) = 0;
    // Reads up to cap bytes, waiting at most timeout_ms for the first. Returns the count, 0 on timeout,
    // -1 on an I/O error.
    virtual long read(std::uint8_t* p, std::size_t cap, int timeout_ms) = 0;
};

// Opens a serial device raw (8N1, no flow control). Returns null, with a message on stderr, on failure.
std::unique_ptr<Endpoint> open_serial(const char* path);

// Runs marv::Fsw(motor_cmd) in this process behind the same encode / Decoder byte path as the Pico.
std::unique_ptr<Endpoint> make_sitl(float motor_cmd);

}  // namespace marv::bridge
