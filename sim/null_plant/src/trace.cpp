#include "marv/null_plant/trace.hpp"

namespace marv::null_plant {

std::uint64_t reference_stamp_us(std::uint32_t num_us, std::uint32_t den, std::uint64_t n) noexcept {
  const unsigned __int128 exact = static_cast<unsigned __int128>(n) * num_us / den;
  return static_cast<std::uint64_t>(exact);
}

bool dshot_value_ok(std::uint16_t v) noexcept { return v == 0 || (v >= 48 && v <= 2047); }

}  // namespace marv::null_plant
