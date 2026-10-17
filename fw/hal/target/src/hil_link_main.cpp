#include <cstdio>

#include <pico/stdio_usb.h>
#include <pico/stdlib.h>

#include "marv/hal_target/hil_link.hpp"

// The HIL S1 image (MARV_FC3V2_HIL_LINK, fw/hal/target/CMakeLists.txt): the USB link measurement and nothing else. It starts
// no IMU, composition tick, receiver or motor output: the DShot PIO program is never loaded and the motor pads are never
// driven, not even with stop frames (decision 0046 / architect D6). Each time a host opens the port the counters restart and
// one text line names the image; the records follow (hil_link_format.hpp).
int main() {
  stdio_init_all();
  bool was_connected = false;
  for (;;) {
    const bool connected = stdio_usb_connected();
    if (connected && !was_connected) {
      marv::hal_target::hil_link_reset();
      std::printf("marv_fc3v2 hil_link %s\n", MARV_FC3V2_VERSION);
    }
    was_connected = connected;
    if (connected) {
      marv::hal_target::hil_link_service();
    }
    tight_loop_contents();
  }
}
