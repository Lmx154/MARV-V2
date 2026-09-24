// GPS setpoints about the home the flight controller reports: shared by marv_ground and marv_gcs's mission executor.
#pragma once

#include <cmath>
#include <cstdint>

#include <marv/fsw/geo.hpp>

namespace marv::ground {

inline GeoPoint geo(double lat_deg, double lon_deg, double alt_m) {
    return {static_cast<std::int32_t>(std::lround(lat_deg * 1e7)), static_cast<std::int32_t>(std::lround(lon_deg * 1e7)),
            static_cast<float>(alt_m)};
}

// A GPS setpoint: horizontal from lat/lon about home, height as metres above home. Height is not taken from the
// geodetic altitude: every estimator measures height from the barometer at the start point, while home's altitude is
// one GNSS fix (metres of noise), so an absolute altitude would land off by that fix's error.
inline Vec3 setpoint(const LocalFrame& frame, double lat_deg, double lon_deg, double alt_above_home_m) {
    Vec3 p = frame.to_ned(geo(lat_deg, lon_deg, frame.origin().alt_m));
    p.z = -static_cast<float>(alt_above_home_m);
    return p;
}

}  // namespace marv::ground
