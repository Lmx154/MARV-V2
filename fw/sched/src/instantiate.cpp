#include <cstddef>
#include <cstdint>
#include <limits>
#include <type_traits>

#include "marv/sched/rate_groups.hpp"

namespace marv::sched {

constexpr std::size_t kMaxGroups = std::numeric_limits<std::uint32_t>::digits;

template class RateGroups<1>;
template class RateGroups<2>;
template class RateGroups<kMaxGroups>;

static_assert(std::is_trivially_copyable_v<RateGroups<1>> && std::is_trivially_copyable_v<RateGroups<kMaxGroups>>);

}  // namespace marv::sched
