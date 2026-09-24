// The flight controller's packet dispatch, shared by the firmware (main.cpp) and the bridge's SITL endpoint
// (bridge/src/endpoint.cpp), so both answer the wire protocol the same way. Header-only; the platform is a template
// parameter (no virtual calls) providing
//
//   void send(const std::uint8_t* p, std::size_t n);  // one whole frame to the PC
//   std::uint8_t load_preset();                        // the stored preset id (0 when none is stored)
//   void store_preset(std::uint8_t id);                // persists it (flash on the Pico, a file in SITL)
//   void reboot();                                     // restarts the controller; returns on a platform that cannot
//
// kReset and kReboot (where reboot returns) rebuild the flight software in place with the stored preset: a new run.
// kSetPreset stores the id only; it takes effect at the next boot, kReset or kReboot.
#pragma once

#include <cstddef>
#include <cstdint>
#include <new>

#include <marv/fsw/fsw.hpp>
#include <marv/link/protocol.hpp>

namespace marv::fw {

template <class Platform> void dispatch(const link::Packet& pkt, Fsw& fsw, Platform& platform) {
    MissionCommand mission;
    State truth;
    SensorBus bus;
    link::Reset reset;
    link::SetPreset set;
    link::Reboot reboot;
    if (pkt.as(bus)) {
        const Tick tick = fsw.step(bus);
        std::uint8_t tx[link::kMaxFrame];
        platform.send(tx, link::encode(tick.tlm, tx));
        platform.send(tx, link::encode(tick.act, tx));
    } else if (pkt.as(truth)) {
        fsw.on_truth(truth);
    } else if (pkt.as(mission)) {
        fsw.on_mission(mission);
    } else if (pkt.as(set)) {
        platform.store_preset(set.id);
    } else if (pkt.as(reset) || pkt.as(reboot)) {
        if (pkt.id == link::kReboot) platform.reboot();
        // In place: a temporary Fsw would not fit on the Pico's stack.
        fsw.~Fsw();
        new (&fsw) Fsw(platform.load_preset());
    }
}

}  // namespace marv::fw
