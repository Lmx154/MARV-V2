// Flight-controller entry point: the USB CDC port is a raw byte pipe carrying only link frames.
// Each kSensors frame runs one flight-software tick and is answered at once with a kActuators frame.
#include <cstddef>
#include <cstdint>

#include "tusb.h"

#include <marv/fsw/fsw.hpp>
#include <marv/link/protocol.hpp>

namespace {

// Fixed motor fraction for the pipeline test; the bridge's --sitl default uses the same value.
constexpr float kPassThroughCmd = 0.85f;

// Writes a whole frame and flushes it. Gives up as soon as the host is gone, so it never blocks forever.
void send(const std::uint8_t* p, std::size_t n) {
    while (n > 0 && tud_cdc_connected()) {
        const std::uint32_t w = tud_cdc_write(p, static_cast<std::uint32_t>(n));
        p += w;
        n -= w;
        tud_cdc_write_flush();
        if (n > 0) tud_task();
    }
}

}  // namespace

int main() {
    tud_init(0);

    static marv::Fsw fsw(kPassThroughCmd);
    static marv::link::Decoder decoder;
    static std::uint8_t rx[64];
    static std::uint8_t tx[marv::link::kMaxFrame];

    for (;;) {
        tud_task();
        if (!tud_cdc_available()) continue;
        const std::uint32_t n = tud_cdc_read(rx, sizeof(rx));
        for (std::uint32_t i = 0; i < n; ++i) {
            if (!decoder.push(rx[i])) continue;
            marv::SensorBus bus;
            if (!decoder.packet().as(bus)) continue;
            const marv::ActuatorCommand cmd = fsw.step(bus);
            send(tx, marv::link::encode(cmd, tx));
        }
    }
}
