#include <type_traits>

#include "marv/types/actuator.hpp"
#include "marv/types/attitude_state.hpp"
#include "marv/types/imu_sample.hpp"
#include "marv/types/time.hpp"

namespace marv {

template struct ActuatorOutput<kQuadXMotors, 0>;
template struct AttitudeState<float>;

static_assert(std::is_trivially_copyable_v<ActuatorOutput<kQuadXMotors, 0>> &&
              std::is_standard_layout_v<ActuatorOutput<kQuadXMotors, 0>>);

}  // namespace marv
