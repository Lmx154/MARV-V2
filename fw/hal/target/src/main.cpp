#include <cstdio>

#include <pico/stdio_usb.h>
#include <pico/stdlib.h>

#include "marv/hal/hal.hpp"
#include "marv/hal/tick.hpp"
#include "marv/params/param.hpp"

namespace {

// How far boot got, printed at boot and again whenever a host opens the CDC port, so a host that attaches late still
// reads it (the lines printed at boot are lost when no host has the port open).
enum class Stage { kStart, kParamsOk, kInitOk };
Stage stage = Stage::kStart;

void print_progress() {
  std::printf("marv_fc3v2 %s\n", MARV_FC3V2_VERSION);
  if (stage >= Stage::kParamsOk) {
    std::printf("params ok\n");
  }
  if (stage >= Stage::kInitOk) {
    std::printf("init ok\n");
  }
}

}  // namespace

// The M1 image: stdio over USB CDC, a banner, the default parameters, the composition's init, then idle. There is no
// tick loop yet (later packets). Console output never runs in the tick (decision 0027 item 6).
int main() {
  stdio_init_all();
  print_progress();
  if (!marv::params_init(marv::param_defaults())) {
    marv::hal_panic("params_init rejected the default table");
  }
  stage = Stage::kParamsOk;
  std::printf("params ok\n");
  marv::composition::init();
  stage = Stage::kInitOk;
  std::printf("init ok\n");
  bool was_connected = false;
  for (;;) {
    const bool connected = stdio_usb_connected();
    if (connected && !was_connected) {
      print_progress();
    }
    was_connected = connected;
    tight_loop_contents();
  }
}
