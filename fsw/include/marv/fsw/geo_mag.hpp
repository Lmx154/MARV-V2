// The Earth's magnetic field at a GNSS position, from PX4's World Magnetic Model lookup (geo_mag.cpp: NOAA WMM grids
// dated 2024.41257, 10-degree nodes, bilinear). The estimators take it at the first fix unless the setup overrides it.
#pragma once

#include <cstdint>

#include <marv/fsw/contracts.hpp>

namespace marv {

// The field at (lat, lon), NED, microtesla: north and east from the declination, down from the inclination, the
// magnitude the total intensity.
Vec3 earth_field_ned_ut(std::int32_t lat_e7, std::int32_t lon_e7);

}  // namespace marv
