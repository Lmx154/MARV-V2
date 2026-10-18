// The composite USB device of the FC3v2 flight image (marv/hal_target/usb.hpp): descriptors, the reset gate and the link
// transport. The console CDC itself is the Pico SDK's pico_stdio_usb (stdio_usb.c), compiled with its default descriptors
// switched off (fw/hal/target/CMakeLists.txt) so the callbacks below supply them; this file adds no console behaviour.
#include "marv/hal_target/usb.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <climits>
#include <cstddef>
#include <cstdint>
#include <string_view>

#include <tusb.h>

#include <device/usbd_pvt.h>
#include <hardware/irq.h>
#include <pico/bootrom.h>
#include <pico/unique_id.h>
#include <pico/usb_reset.h>

#include "marv/prim/constants.hpp"

namespace marv::hal_target {
namespace {

namespace prim = marv::prim;

// Interface numbers in descriptor order: each CDC takes two (control, data). The console stays at interface 0 so the host's
// ...-if00 path is unchanged; the link port is then -if02.
enum Interface : std::uint8_t { kItfConsole, kItfConsoleData, kItfLink, kItfLinkData, kItfReset, kItfCount };
static_assert(kItfReset == PICO_USB_RESET_MS_OS_20_DESCRIPTOR_ITF,
              "the reset interface number the Microsoft OS 2.0 descriptor names (usb_reset.c) is the one in the configuration");

enum StringIndex : std::uint8_t { kStrLanguage, kStrManufacturer, kStrProduct, kStrSerial, kStrConsole, kStrLink, kStrReset, kStrCount };

// The SDK's identity strings (stdio_usb_descriptors.c) with the interface strings fixed for this image. Index kStrSerial is the
// board's unique ID, read when first asked for, as the SDK does.
constexpr std::array<std::string_view, kStrCount> kStrings{"",         "Raspberry Pi", "Pico",  "",
                                                           "MARV console", "MARV link", "Reset"};
constexpr std::size_t kSerialChars = PICO_UNIQUE_BOARD_ID_SIZE_BYTES * 2;

constexpr std::size_t longest_string() noexcept {
  std::size_t n = kSerialChars;
  for (const std::string_view s : kStrings) {
    n = std::max(n, s.size());
  }
  return n;
}
constexpr std::size_t kStringCharsMax = longest_string();

constexpr std::size_t kConfigBytes = TUD_CONFIG_DESC_LEN + 2 * TUD_CDC_DESC_LEN + TUD_RPI_RESET_DESC_LEN;

const tusb_desc_device_t kDeviceDescriptor = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = prim::kUsbBcdUsb210,
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = prim::kUsbVendorIdRaspberryPi,
    .idProduct = prim::kUsbProductIdPicoSdkCdc,
    .bcdDevice = prim::kUsbBcdDevice100,
    .iManufacturer = kStrManufacturer,
    .iProduct = kStrProduct,
    .iSerialNumber = kStrSerial,
    .bNumConfigurations = 1,
};

const std::uint8_t kConfigDescriptor[] = {
    TUD_CONFIG_DESCRIPTOR(1, kItfCount, kStrLanguage, kConfigBytes, 0, prim::kUsbConfigMaxPowerMa),
    TUD_CDC_DESCRIPTOR(kItfConsole, kStrConsole, prim::kUsbEpConsoleNotifyIn, prim::kUsbCdcNotifyPacketBytes,
                       prim::kUsbEpConsoleDataOut, prim::kUsbEpConsoleDataIn, prim::kUsbFullSpeedBulkPacketBytes),
    TUD_CDC_DESCRIPTOR(kItfLink, kStrLink, prim::kUsbEpLinkNotifyIn, prim::kUsbCdcNotifyPacketBytes,
                       prim::kUsbEpLinkDataOut, prim::kUsbEpLinkDataIn, prim::kUsbFullSpeedBulkPacketBytes),
    TUD_RPI_RESET_DESCRIPTOR(kItfReset, kStrReset),
};
static_assert(sizeof(kConfigDescriptor) == kConfigBytes, "the configuration descriptor's total length");

char g_serial[kSerialChars + 1];

// The reset gate and its refusal counter. The gate is set once at start-up; requests arrive from the SDK's USB service
// (a user interrupt below the IMU's, or the background loop's own tud_task calls).
std::atomic<ResetGate> g_gate{nullptr};
std::atomic<std::uint32_t> g_refused{0};

[[nodiscard]] bool reset_granted() noexcept {
  const ResetGate gate = g_gate.load(std::memory_order_relaxed);
  return gate != nullptr && gate();
}

// A refused request is counted and not acted on.
[[nodiscard]] bool refuse() noexcept {
  g_refused.fetch_add(1, std::memory_order_relaxed);
  return false;
}

// The reset interface's control requests: the SDK's handler (usb_reset.c), behind the gate. Only its two reset requests are
// gated; anything else it receives is its own business.
bool reset_control_xfer(std::uint8_t rhport, std::uint8_t stage, const tusb_control_request_t* request) {
  const bool is_reset_request = stage == CONTROL_STAGE_SETUP && request->wIndex == kItfReset &&
                                (request->bRequest == RESET_REQUEST_BOOTSEL || request->bRequest == RESET_REQUEST_FLASH);
  if (is_reset_request && !reset_granted()) {
    return refuse();
  }
  return usb_reset_interface_control_xfer_cb(rhport, stage, request);
}

const usbd_class_driver_t kResetDriver = {
    .name = "MARV reset",
    .init = usb_reset_interface_init,
    .deinit = nullptr,
    .reset = usb_reset_interface_reset,
    .open = usb_reset_interface_open,
    .control_xfer_cb = reset_control_xfer,
    .xfer_cb = usb_reset_interface_xfer_cb,
    .sof = nullptr,
};

// The link transport's critical section: the SDK services the device from a user interrupt at PICO_DEFAULT_IRQ_PRIORITY and
// holds its own mutex around the console's calls; TinyUSB's CDC functions are not safe against that service running in the
// middle of them, and the SDK's mutex is private to stdio_usb.c. BASEPRI masks that level and every level below it (the USB
// interrupt and the service), and nothing above: the IMU and receiver interrupts keep running. Held for one FIFO move.
class UsbLevelMask {
 public:
  UsbLevelMask() noexcept {
    asm volatile("mrs %0, basepri" : "=r"(saved_));
    asm volatile("msr basepri, %0" : : "r"(static_cast<std::uint32_t>(PICO_DEFAULT_IRQ_PRIORITY)) : "memory");
  }
  ~UsbLevelMask() { asm volatile("msr basepri, %0" : : "r"(saved_) : "memory"); }
  UsbLevelMask(const UsbLevelMask&) = delete;
  UsbLevelMask& operator=(const UsbLevelMask&) = delete;

 private:
  std::uint32_t saved_ = 0;
};

constexpr std::uint8_t kLinkCdc = 1;

}  // namespace

void usb_set_reset_gate(ResetGate gate) noexcept { g_gate.store(gate, std::memory_order_relaxed); }

std::uint32_t usb_reset_refused() noexcept { return g_refused.load(std::memory_order_relaxed); }

bool link_usb_connected() noexcept { return tud_cdc_n_connected(kLinkCdc); }

std::size_t link_usb_receive(std::span<std::uint8_t> out) noexcept {
  if (out.empty()) {
    return 0;
  }
  const UsbLevelMask mask;
  if (tud_cdc_n_available(kLinkCdc) == 0) {
    return 0;
  }
  return tud_cdc_n_read(kLinkCdc, out.data(), static_cast<std::uint32_t>(out.size()));
}

std::size_t link_usb_send(std::span<const std::uint8_t> bytes) noexcept {
  if (bytes.empty() || !tud_cdc_n_connected(kLinkCdc)) {
    return 0;
  }
  const UsbLevelMask mask;
  const std::size_t n = std::min<std::size_t>(bytes.size(), tud_cdc_n_write_available(kLinkCdc));
  if (n == 0) {
    return 0;
  }
  const std::size_t queued = tud_cdc_n_write(kLinkCdc, bytes.data(), static_cast<std::uint32_t>(n));
  static_cast<void>(tud_cdc_n_write_flush(kLinkCdc));
  return queued;
}

#if defined(MARV_FC3V2_USB_BENCH)
void usb_bench_echo_step() noexcept {
  static std::array<std::uint8_t, CFG_TUD_CDC_RX_BUFSIZE> chunk{};
  static std::size_t held = 0;
  static std::size_t sent = 0;
  if (sent == held) {
    held = link_usb_receive(chunk);
    sent = 0;
  }
  if (sent < held) {
    sent += link_usb_send(std::span<const std::uint8_t>{chunk.data() + sent, held - sent});
  }
}
#endif

}  // namespace marv::hal_target

// TinyUSB's application callbacks (C linkage). Called from the SDK's USB service.
extern "C" {

const std::uint8_t* tud_descriptor_device_cb(void) { return reinterpret_cast<const std::uint8_t*>(&marv::hal_target::kDeviceDescriptor); }

const std::uint8_t* tud_descriptor_configuration_cb(std::uint8_t) { return marv::hal_target::kConfigDescriptor; }

const std::uint16_t* tud_descriptor_string_cb(std::uint8_t index, std::uint16_t) {
  using namespace marv::hal_target;
  static std::array<std::uint16_t, kStringCharsMax + 1> desc{};
  if (g_serial[0] == '\0') {
    pico_get_unique_board_id_string(g_serial, sizeof(g_serial));
  }
  std::size_t len = 0;
  if (index == kStrLanguage) {
    desc[1] = marv::prim::kUsbLangIdEnglishUs;
    len = 1;
  } else if (index < kStrCount) {
    const std::string_view text = index == kStrSerial ? std::string_view{g_serial} : kStrings[index];
    for (; len < text.size(); ++len) {
      desc[1 + len] = static_cast<std::uint8_t>(text[len]);
    }
  } else {
    return nullptr;
  }
  // First element: length in bytes including this header (low byte), descriptor type (high byte).
  desc[0] = static_cast<std::uint16_t>((TUSB_DESC_STRING << CHAR_BIT) | (2 * len + 2));
  return desc.data();
}

// The reset interface's driver, gated (the SDK's default registration is switched off in CMake).
const usbd_class_driver_t* usbd_app_driver_get_cb(std::uint8_t* driver_count) {
  *driver_count = 1;
  return &marv::hal_target::kResetDriver;
}

// The 1200-baud line coding reset of the SDK (usb_reset.c, switched off in CMake), behind the same gate, on either CDC.
void tud_cdc_line_coding_cb(std::uint8_t, const cdc_line_coding_t* line_coding) {
  if (line_coding->bit_rate != PICO_USB_RESET_MAGIC_BAUD_RATE) {
    return;
  }
  if (!marv::hal_target::reset_granted()) {
    static_cast<void>(marv::hal_target::refuse());
    return;
  }
  rom_reset_usb_boot_extra(-1, PICO_USB_RESET_BOOTSEL_INTERFACE_DISABLE_MASK, false);
}

}  // extern "C"
