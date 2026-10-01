// L6 stage (a): the IMU output of the plant has the layout of marv_imu_meas (fw/sil/include/marv_sil.h), field for
// field, and its flag bits are those of the SIL and of marv::ImuFlag. A host adapter can then copy it with one
// assignment; this test is where a divergence is caught. Test code only: fw/ does not see the plant.
#include <gtest/gtest.h>

#include <cstddef>
#include <type_traits>

#include "marv/types/imu_sample.hpp"
#include "marv_plant.h"
#include "marv_sil.h"

namespace {

static_assert(sizeof(marv_plant_vec3f) == sizeof(marv_vec3f));
static_assert(offsetof(marv_plant_vec3f, x) == offsetof(marv_vec3f, x));
static_assert(offsetof(marv_plant_vec3f, y) == offsetof(marv_vec3f, y));
static_assert(offsetof(marv_plant_vec3f, z) == offsetof(marv_vec3f, z));
static_assert(sizeof(marv_plant_imu_out) == sizeof(marv_imu_meas));
static_assert(alignof(marv_plant_imu_out) == alignof(marv_imu_meas));
static_assert(offsetof(marv_plant_imu_out, gyro_rad_s) == offsetof(marv_imu_meas, gyro_rad_s));
static_assert(offsetof(marv_plant_imu_out, accel_m_s2) == offsetof(marv_imu_meas, accel_m_s2));
static_assert(offsetof(marv_plant_imu_out, temp_k) == offsetof(marv_imu_meas, temp_k));
static_assert(offsetof(marv_plant_imu_out, flags) == offsetof(marv_imu_meas, flags));
static_assert(std::is_same_v<decltype(marv_plant_imu_out::temp_k), decltype(marv_imu_meas::temp_k)>);
static_assert(std::is_same_v<decltype(marv_plant_imu_out::flags), decltype(marv_imu_meas::flags)>);
static_assert(std::is_same_v<decltype(marv_plant_vec3f::x), decltype(marv_vec3f::x)>);
static_assert(std::is_standard_layout_v<marv_plant_imu_out> && std::is_trivially_copyable_v<marv_plant_imu_out>);

static_assert(static_cast<int>(MARV_PLANT_IMU_GYRO_SAT_X) == static_cast<int>(MARV_IMU_GYRO_SAT_X));
static_assert(static_cast<int>(MARV_PLANT_IMU_GYRO_SAT_Y) == static_cast<int>(MARV_IMU_GYRO_SAT_Y));
static_assert(static_cast<int>(MARV_PLANT_IMU_GYRO_SAT_Z) == static_cast<int>(MARV_IMU_GYRO_SAT_Z));
static_assert(static_cast<int>(MARV_PLANT_IMU_ACCEL_SAT_X) == static_cast<int>(MARV_IMU_ACCEL_SAT_X));
static_assert(static_cast<int>(MARV_PLANT_IMU_ACCEL_SAT_Y) == static_cast<int>(MARV_IMU_ACCEL_SAT_Y));
static_assert(static_cast<int>(MARV_PLANT_IMU_ACCEL_SAT_Z) == static_cast<int>(MARV_IMU_ACCEL_SAT_Z));
static_assert(static_cast<int>(MARV_PLANT_IMU_GYRO_VALID) == static_cast<int>(MARV_IMU_GYRO_VALID));
static_assert(static_cast<int>(MARV_PLANT_IMU_ACCEL_VALID) == static_cast<int>(MARV_IMU_ACCEL_VALID));
static_assert(static_cast<int>(MARV_PLANT_IMU_TEMP_VALID) == static_cast<int>(MARV_IMU_TEMP_VALID));
static_assert(static_cast<int>(MARV_PLANT_IMU_GYRO_SAT_X) == static_cast<int>(marv::ImuFlag::GyroSatX));
static_assert(static_cast<int>(MARV_PLANT_IMU_GYRO_VALID) == static_cast<int>(marv::ImuFlag::GyroValid));
static_assert(static_cast<int>(MARV_PLANT_IMU_ACCEL_VALID) == static_cast<int>(marv::ImuFlag::AccelValid));
static_assert(static_cast<int>(MARV_PLANT_IMU_TEMP_VALID) == static_cast<int>(marv::ImuFlag::TempValid));
static_assert(static_cast<int>(MARV_PLANT_IMU_TEMP_VALID) + 1 == static_cast<int>(MARV_IMU_FLAG_COUNT));

TEST(ImuLayout, MatchesTheSilMeasurementAtCompileTime) { SUCCEED(); }

}  // namespace
