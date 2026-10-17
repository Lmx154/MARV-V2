// Shared between boot.cpp and staging.cpp; not part of the store's interface.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "marv/hal/storage.hpp"
#include "marv/store/store.hpp"

namespace marv::store::detail {

struct SlotView {
  bool committed = false;  // the commit record starts with its magic
  bool intact = false;     // and the file matches it (length, SHA-256, header sequence)
  std::uint32_t sequence = 0;
  std::span<const std::uint8_t> file;  // when intact
};
using SlotViews = std::array<SlotView, kConfigSlots>;

[[nodiscard]] SlotViews view_slots() noexcept;

// The index of the committed slot with the larger sequence (A on a tie); kConfigSlots if none is committed.
[[nodiscard]] std::size_t newest(const SlotViews& v) noexcept;

// The inputs boot was given; null before boot.
[[nodiscard]] const BootInputs* booted_inputs() noexcept;

// Value and provenance equal, bit for bit, source text compared.
[[nodiscard]] bool same_record(const ParamRecord& a, const ParamRecord& b) noexcept;

}  // namespace marv::store::detail
