#include "marv/hal_target/rc_input.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>

#include "marv/prim/constants.hpp"

namespace marv::hal_target {
namespace {

[[nodiscard]] bool channel_ok(std::int32_t n) noexcept {
  return n >= 1 && static_cast<std::size_t>(n) <= prim::kCrsfRcChannels;
}

}  // namespace

bool rc_channel_map_valid(const RcChannelMap& map) noexcept {
  return channel_ok(map.roll) && channel_ok(map.pitch) && channel_ok(map.throttle) && channel_ok(map.yaw);
}

bool rc_channels_distinct(const RcChannelMap& map, std::int32_t arm, std::int32_t angle) noexcept {
  const std::array assigned{map.roll, map.pitch, map.throttle, map.yaw, arm, angle};
  const std::size_t used = angle == 0 ? assigned.size() - 1 : assigned.size();
  for (std::size_t i = 0; i < used; ++i) {
    for (std::size_t j = i + 1; j < used; ++j) {
      if (assigned[i] == assigned[j]) {
        return false;
      }
    }
  }
  return true;
}

float crsf_ticks_to_us(std::uint16_t ticks) noexcept {
  return static_cast<float>(static_cast<std::int32_t>(ticks) - prim::kCrsfTickCentre) * prim::kCrsfUsPerTickNum /
             prim::kCrsfUsPerTickDen +
         prim::kCrsfUsCentre;
}

float rc_normalised(float us) noexcept { return std::clamp((us - prim::kCrsfUsCentre) / prim::kRcUsHalfSpan, -1.0F, 1.0F); }

float rc_throttle(float us) noexcept {
  return std::clamp((us - prim::kRcUsThrottleMin) / prim::kRcUsThrottleSpan, 0.0F, 1.0F);
}

bool crsf_to_controls(const CrsfRcChannels& rc, const RcChannelMap& map, ManualControl& out) noexcept {
  if (!rc_channel_map_valid(map)) {
    return false;
  }
  std::array<float, prim::kCrsfRcChannels> us{};
  for (std::size_t i = 0; i < prim::kCrsfRcChannels; ++i) {
    us[i] = crsf_ticks_to_us(rc.raw[i]);
    out.channels[i] = rc_normalised(us[i]);
  }
  const auto index = [](std::int32_t n) { return static_cast<std::size_t>(n - 1); };
  out.roll = out.channels[index(map.roll)];
  out.pitch = 0.0F - out.channels[index(map.pitch)];
  out.yaw = out.channels[index(map.yaw)];
  out.throttle = rc_throttle(us[index(map.throttle)]);
  return true;
}

float rc_link_quality(std::uint8_t percent) noexcept {
  return std::clamp(static_cast<float>(percent) / prim::kPercentPerUnit, 0.0F, 1.0F);
}

}  // namespace marv::hal_target
