// Test-only composition: records, on every tick, what the HAL rotor-speed pull read returned. No actuator output (the SIL
// reports the latches' stop values).
#include <marv/composition.hpp>
#include <marv/hal/hal.hpp>
#include <marv/hal/tick.hpp>

namespace marv {
namespace {

std::array<rotor_probe::Seen, rotor_probe::kCapacity> g_seen{};
std::size_t g_count = 0;

}  // namespace

namespace composition {

void init() noexcept { rotor_probe::reset(); }

void tick(const ImuSample& s) noexcept {
  if (g_count >= rotor_probe::kCapacity) {
    hal_panic("rotor-speed probe composition: recording is full");
  }
  g_seen[g_count] = rotor_probe::Seen{hal_rotor_speed(), s.t_us, hal_time_us()};
  ++g_count;
}

}  // namespace composition

namespace rotor_probe {

std::size_t count() noexcept { return g_count; }
const Seen& at(std::size_t i) noexcept { return g_seen[i]; }
void reset() noexcept { g_count = 0; }

}  // namespace rotor_probe
}  // namespace marv
