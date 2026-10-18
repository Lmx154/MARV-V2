#pragma once

// The composite USB device of the FC3v2 flight image (ground-segment unit 12a, decision 0053; descriptors, reset gate and
// link transport in src/usb.cpp). Three functions: CDC 0 (interface 0, "MARV console") is the stdio console, exactly the
// Pico SDK's pico_stdio_usb CDC, its bytes and connection handling unchanged; CDC 1 (interface 2, "MARV link") is the byte
// transport for the ground link; the Raspberry Pi reset interface (interface 4) lets picotool find the board. The device
// identity (vendor, product, serial) is the SDK's, so the host path of the console does not change. The HIL images keep the
// SDK's single-CDC device and do not compile this file.
//
// The SDK services the device from a user interrupt at PICO_DEFAULT_IRQ_PRIORITY (below the IMU and receiver interrupts), so
// no USB work runs at or above the IMU interrupt. The transport calls below run in the background loop and mask that
// priority level and below (BASEPRI) for the few microseconds a call takes, which keeps the SDK's service from interleaving
// with them; the IMU interrupt is never masked.
#include <cstddef>
#include <cstdint>
#include <span>

namespace marv::hal_target {

// The reset gate. A USB-initiated reset (the reset interface's BOOTSEL or flash-boot request, or the 1200-baud line coding
// on either CDC) is honoured only when the gate returns true, which main wires to composition::maintenance_granted; with no
// gate set, or a gate that returns false, the request is refused and counted. Honoured requests use the SDK's own paths
// (rom_reset_usb_boot_extra, watchdog_reboot). Call once at start-up, before the host can enumerate the device.
using ResetGate = bool (*)() noexcept;
void usb_set_reset_gate(ResetGate gate) noexcept;

// USB-initiated resets refused since boot. Any context.
[[nodiscard]] std::uint32_t usb_reset_refused() noexcept;

// The link transport, a byte stream shaped like hal_sim's UDP link (marv/hal_sim/link_udp.hpp). Background loop only,
// never blocking.
// True while a host has the link port open (DTR).
[[nodiscard]] bool link_usb_connected() noexcept;
// Moves up to out.size() received bytes into out; returns the count (0 if none).
[[nodiscard]] std::size_t link_usb_receive(std::span<std::uint8_t> out) noexcept;
// Queues as many of the bytes as fit and starts the transfer; returns the count queued (0 when the port is not open or the
// queue is full). The caller keeps the rest.
[[nodiscard]] std::size_t link_usb_send(std::span<const std::uint8_t> bytes) noexcept;

#if defined(MARV_FC3V2_USB_BENCH)
// The bench echo (MARV_FC3V2_USB_BENCH, bench only, default OFF): sends every byte received on the link port back on it. One
// bounded chunk per call, from the background loop.
void usb_bench_echo_step() noexcept;
#endif

}  // namespace marv::hal_target
