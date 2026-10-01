// L6 stage (b), the plant's rotor-speed output has the layout of the firmware's RotorSpeedSample without t_us, and of the
// SIL's marv_rotor_speed_meas, field for field; its flag bits are those of marv::RotorSpeedFlag and of the SIL. A host
// adapter can then copy it with one memcpy. RotorSpeedSample is 8-byte aligned by t_us and has 4 bytes of tail padding that
// the C structs do not, so the sizes are compared through the end of flags. Test code only: fw/ does not see the plant.
#include <gtest/gtest.h>

#include <cstddef>
#include <type_traits>

#include "marv/types/rotor_speed_sample.hpp"
#include "marv_plant.h"
#include "marv_sil.h"

namespace {

using marv::RotorSpeedSample;

constexpr std::size_t kStamp = sizeof(marv::TimeUs);

static_assert(sizeof(marv_plant_rotor_speed_out) == offsetof(RotorSpeedSample, flags) + sizeof(RotorSpeedSample::flags) - kStamp);
static_assert(offsetof(marv_plant_rotor_speed_out, omega_rad_s) + kStamp == offsetof(RotorSpeedSample, omega_rad_s));
static_assert(offsetof(marv_plant_rotor_speed_out, flags) + kStamp == offsetof(RotorSpeedSample, flags));
static_assert(sizeof(marv_plant_rotor_speed_out::omega_rad_s) == sizeof(RotorSpeedSample::omega_rad_s));
static_assert(std::is_same_v<std::remove_all_extents_t<decltype(marv_plant_rotor_speed_out::omega_rad_s)>,
                             decltype(RotorSpeedSample::omega_rad_s)::value_type>);
static_assert(std::is_same_v<decltype(marv_plant_rotor_speed_out::flags), decltype(RotorSpeedSample::flags)>);
static_assert(std::is_standard_layout_v<marv_plant_rotor_speed_out> && std::is_trivially_copyable_v<marv_plant_rotor_speed_out>);

static_assert(sizeof(marv_plant_rotor_speed_out) == sizeof(marv_rotor_speed_meas));
static_assert(offsetof(marv_plant_rotor_speed_out, omega_rad_s) == offsetof(marv_rotor_speed_meas, omega_rad_s));
static_assert(offsetof(marv_plant_rotor_speed_out, flags) == offsetof(marv_rotor_speed_meas, flags));
static_assert(offsetof(marv_rotor_speed_meas, omega_rad_s) + kStamp == offsetof(RotorSpeedSample, omega_rad_s));
static_assert(offsetof(marv_rotor_speed_meas, flags) + kStamp == offsetof(RotorSpeedSample, flags));

static_assert(static_cast<std::size_t>(MARV_PLANT_N_MOTORS) == marv::kQuadXMotors);
static_assert(static_cast<std::size_t>(MARV_ROTOR_SPEED_MOTORS) == marv::kQuadXMotors);
static_assert(static_cast<int>(MARV_PLANT_ROTOR_SPEED_M1_VALID) == static_cast<int>(marv::RotorSpeedFlag::M1Valid));
static_assert(static_cast<int>(MARV_PLANT_ROTOR_SPEED_M2_VALID) == static_cast<int>(marv::RotorSpeedFlag::M2Valid));
static_assert(static_cast<int>(MARV_PLANT_ROTOR_SPEED_M3_VALID) == static_cast<int>(marv::RotorSpeedFlag::M3Valid));
static_assert(static_cast<int>(MARV_PLANT_ROTOR_SPEED_M4_VALID) == static_cast<int>(marv::RotorSpeedFlag::M4Valid));
static_assert(static_cast<int>(MARV_ROTOR_SPEED_M1_VALID) == static_cast<int>(marv::RotorSpeedFlag::M1Valid));
static_assert(static_cast<int>(MARV_ROTOR_SPEED_M2_VALID) == static_cast<int>(marv::RotorSpeedFlag::M2Valid));
static_assert(static_cast<int>(MARV_ROTOR_SPEED_M3_VALID) == static_cast<int>(marv::RotorSpeedFlag::M3Valid));
static_assert(static_cast<int>(MARV_ROTOR_SPEED_M4_VALID) == static_cast<int>(marv::RotorSpeedFlag::M4Valid));
static_assert(static_cast<int>(MARV_ROTOR_SPEED_FLAG_COUNT) == static_cast<int>(marv::RotorSpeedFlag::Count));
static_assert(marv::rotor_speed_valid_bit(0) == (1U << MARV_PLANT_ROTOR_SPEED_M1_VALID));
static_assert(marv::rotor_speed_valid_bit(3) == (1U << MARV_PLANT_ROTOR_SPEED_M4_VALID));

TEST(RotorSpeedLayout, MatchesTheFirmwareSampleAndTheSilMeasurementAtCompileTime) { SUCCEED(); }

}  // namespace
