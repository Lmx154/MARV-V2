#include "marv/store/arena.hpp"

#include <cstdint>
#include <span>

namespace marv::store {
namespace {

std::span<std::uint8_t> g_arena{};  // empty until arena_open
bool g_leased = false;
ArenaHolder g_holder{};

}  // namespace

void arena_open(std::span<std::uint8_t> arena) noexcept { g_arena = arena; }

std::span<std::uint8_t> arena_lease(const ArenaHolder& holder) noexcept {
  if (g_arena.empty() || g_leased || holder.yield == nullptr) {
    return {};
  }
  g_holder = holder;
  g_leased = true;
  return g_arena;
}

bool arena_leased() noexcept { return g_leased; }

bool arena_reclaim() noexcept {
  if (!g_leased) {
    return true;
  }
  if (!g_holder.yield(g_holder.ctx)) {
    return false;
  }
  g_leased = false;
  g_holder = ArenaHolder{};
  return true;
}

}  // namespace marv::store
