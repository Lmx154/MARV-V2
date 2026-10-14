#include "marv/vertical/config.hpp"
#include "marv/vertical/vertical_law.hpp"

namespace marv::vertical {

template class VerticalLaw<float>;
template ConfigError validate<float>(const VerticalConfig<float>&) noexcept;

}  // namespace marv::vertical
