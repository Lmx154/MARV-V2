// Flight-controller entry point: the USB CDC port is a raw byte pipe carrying only link frames, dispatched by
// dispatch.hpp (the same code the bridge's SITL endpoint runs). kReset rebuilds the flight software in place (a new
// run) with the stored preset; each kSensors frame runs one tick and is answered at once with a kTelemetry frame, then
// a kActuators frame (always last). kSetPreset stores the preset id in the last flash sector; kReboot restarts through
// the watchdog.
#include <cstddef>
#include <cstdint>

#include "hardware/flash.h"
#include "hardware/watchdog.h"
#include "pico/flash.h"
#include "tusb.h"

#include "dispatch.hpp"

namespace {

// The preset record: the first words of the last 4 KB flash sector. Erased flash (all ones) has no magic: preset 0.
constexpr std::uint32_t kPresetOffset = PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE;
constexpr std::uint32_t kPresetMagic = 0x5056524Du;  // "MRVP"

struct PresetRecord {
    std::uint32_t magic;
    std::uint32_t id;
    std::uint32_t check;  // ~id
};

void program_preset(void* page) {
    flash_range_erase(kPresetOffset, FLASH_SECTOR_SIZE);
    flash_range_program(kPresetOffset, static_cast<const std::uint8_t*>(page), FLASH_PAGE_SIZE);
}

struct Pico {
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

    std::uint8_t load_preset() {
        const auto* r = reinterpret_cast<const PresetRecord*>(XIP_BASE + kPresetOffset);
        if (r->magic != kPresetMagic || r->check != ~r->id || r->id > 0xFFu) return 0;
        return static_cast<std::uint8_t>(r->id);
    }

    void store_preset(std::uint8_t id) {
        const auto* r = reinterpret_cast<const PresetRecord*>(XIP_BASE + kPresetOffset);
        if (r->magic == kPresetMagic && r->id == id && r->check == ~static_cast<std::uint32_t>(id)) return;
        static std::uint8_t page[FLASH_PAGE_SIZE];
        for (std::uint8_t& b : page) b = 0xFF;
        const PresetRecord rec{kPresetMagic, id, ~static_cast<std::uint32_t>(id)};
        const auto* src = reinterpret_cast<const std::uint8_t*>(&rec);
        for (std::size_t i = 0; i < sizeof(rec); ++i) page[i] = src[i];
        // Interrupts off (USB included) for the erase and program; a single core runs, so nothing else reads flash.
        flash_safe_execute(program_preset, page, 100);
    }

    [[noreturn]] void reboot() {
        watchdog_reboot(0, 0, 1);
        for (;;) tight_loop_contents();
    }
};

}  // namespace

int main() {
    tud_init(0);

    static Pico pico;
    static marv::Fsw fsw{pico.load_preset()};
    static marv::link::Decoder decoder;
    static std::uint8_t rx[64];

    for (;;) {
        tud_task();
        if (!tud_cdc_available()) continue;
        const std::uint32_t n = tud_cdc_read(rx, sizeof(rx));
        for (std::uint32_t i = 0; i < n; ++i)
            if (decoder.push(rx[i])) marv::fw::dispatch(decoder.packet(), fsw, pico);
    }
}
