#include <type_traits>

#include "marv/types/actuator.hpp"
#include "marv/types/attitude_state.hpp"
#include "marv/types/imu_sample.hpp"
#include "marv/types/manual_control.hpp"
#include "marv/types/rotor_speed_sample.hpp"
#include "marv/types/seqlock.hpp"
#include "marv/types/telemetry.hpp"
#include "marv/types/time.hpp"

namespace marv {

template struct ActuatorOutput<kQuadXMotors, 0>;
template struct AttitudeState<float>;
template class Seqlock<Telemetry>;

static_assert(std::is_trivially_copyable_v<ActuatorOutput<kQuadXMotors, 0>> &&
              std::is_standard_layout_v<ActuatorOutput<kQuadXMotors, 0>>);

}  // namespace marv
