// Flight-controller entry point: the USB CDC port is a raw byte pipe carrying only link frames.
// kReset rebuilds the flight software in place (a new run); kMission and kTruth frames are handed to the flight software; each kSensors frame runs one tick and is
// answered at once with a kTelemetry frame, then a kActuators frame (always last).
#include <cstddef>
#include <cstdint>
#include <new>

#include "tusb.h"

#include <marv/fsw/fsw.hpp>
#include <marv/link/protocol.hpp>

namespace {

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

    static marv::Fsw fsw;
    static marv::link::Decoder decoder;
    static std::uint8_t rx[64];
    static std::uint8_t tx[marv::link::kMaxFrame];

    for (;;) {
        tud_task();
        if (!tud_cdc_available()) continue;
        const std::uint32_t n = tud_cdc_read(rx, sizeof(rx));
        for (std::uint32_t i = 0; i < n; ++i) {
            if (!decoder.push(rx[i])) continue;
            const marv::link::Packet& pkt = decoder.packet();
            marv::MissionCommand mission;
            marv::State truth;
            marv::SensorBus bus;
            marv::link::Reset reset;
            if (pkt.as(reset)) {
                // In place: a temporary Fsw would not fit on the stack.
                fsw.~Fsw();
                new (&fsw) marv::Fsw();
            } else if (pkt.as(mission)) {
                fsw.on_mission(mission);
            } else if (pkt.as(truth)) {
                fsw.on_truth(truth);
            } else if (pkt.as(bus)) {
                const marv::Tick tick = fsw.step(bus);
                send(tx, marv::link::encode(tick.tlm, tx));
                send(tx, marv::link::encode(tick.act, tx));
            }
        }
    }
}
