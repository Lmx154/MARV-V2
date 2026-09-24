// Flight-controller entry point: the USB CDC port is a raw byte pipe carrying only link frames, dispatched by
// dispatch.hpp (the same code the bridge's SITL endpoint runs). Power-on runs the stored setup; kReset rebuilds the
// flight software in place (a new run) with the staged one; each kSensors frame runs one tick and is answered at once
// with a kTelemetry frame, then a kActuators frame (always last). The setup record lives in the last flash sector;
// kReboot restarts through the watchdog.
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "hardware/flash.h"
#include "hardware/watchdog.h"
#include "pico/flash.h"
#include "tusb.h"

#include "dispatch.hpp"

namespace {

// The setup record (dispatch.hpp): the start of the last 4 KB flash sector, programmed in whole pages padded with
// erased bytes. Erased flash (all ones) has no magic: factory 0.
constexpr std::uint32_t kSetupOffset = PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE;
constexpr std::size_t kSetupPages = (marv::fw::kRecordBytes + FLASH_PAGE_SIZE - 1) / FLASH_PAGE_SIZE;
static_assert(kSetupPages * FLASH_PAGE_SIZE <= FLASH_SECTOR_SIZE, "the record fits its sector");

const std::uint8_t* setup_flash() { return reinterpret_cast<const std::uint8_t*>(XIP_BASE + kSetupOffset); }

void program_setup(void* pages) {
    flash_range_erase(kSetupOffset, FLASH_SECTOR_SIZE);
    flash_range_program(kSetupOffset, static_cast<const std::uint8_t*>(pages), kSetupPages * FLASH_PAGE_SIZE);
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

    std::size_t read_record(std::uint8_t* p, std::size_t cap) {
        const std::size_t n = cap < FLASH_SECTOR_SIZE ? cap : FLASH_SECTOR_SIZE;
        std::memcpy(p, setup_flash(), n);
        return n;
    }

    void write_record(const std::uint8_t* p, std::size_t n) {
        static std::uint8_t pages[kSetupPages * FLASH_PAGE_SIZE];
        for (std::size_t i = 0; i < sizeof(pages); ++i) pages[i] = i < n ? p[i] : 0xFF;
        if (std::memcmp(pages, setup_flash(), sizeof(pages)) == 0) return;  // already there: no erase
        // Interrupts off (USB included) for the erase and program; a single core runs, so nothing else reads flash.
        flash_safe_execute(program_setup, pages, 100);
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
    static marv::fw::Node<Pico> node{pico};
    static marv::link::Decoder decoder;
    static std::uint8_t rx[64];

    for (;;) {
        tud_task();
        if (!tud_cdc_available()) continue;
        const std::uint32_t n = tud_cdc_read(rx, sizeof(rx));
        for (std::uint32_t i = 0; i < n; ++i)
            if (decoder.push(rx[i])) node.dispatch(decoder.packet());
    }
}
