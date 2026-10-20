// Negative control of invariant L-8b's post-link check (tools/ci/check_single_core.py): an image that starts core 1, so the
// check must find multicore_launch_core1 in it.
#include <pico/multicore.h>

namespace {

void core1_entry() {}

}  // namespace

int main() {
  multicore_launch_core1(&core1_entry);
  return 0;
}
