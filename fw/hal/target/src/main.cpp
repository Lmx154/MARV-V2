#include <cstdio>

#include <pico/stdlib.h>

#include "marv/hal/hal.hpp"
#include "marv/hal/tick.hpp"
#include "marv/params/param.hpp"

// The M1 image: stdio over USB CDC, a banner, the default parameters, the composition's init, then idle. There is no
// tick loop yet (later packets). Console output never runs in the tick (decision 0027 item 6).
int main() {
  stdio_init_all();
  std::printf("marv_fc3v2 %s\n", MARV_FC3V2_VERSION);
  if (!marv::params_init(marv::param_defaults())) {
    marv::hal_panic("params_init rejected the default table");
  }
  marv::composition::init();
  for (;;) {
    tight_loop_contents();
  }
}
