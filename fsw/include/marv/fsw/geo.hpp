// Local NED about a geodetic origin: flat earth with the WGS84 meridian and prime-vertical radii at the
// origin's latitude. Shared by every estimator (origin = first GNSS fix) and by the ground software
// (GPS setpoints -> NED about the home the flight controller reports), so both use one frame.
#pragma once

#include <cmath>
#include <cstdint>

#include <marv/fsw/contracts.hpp>

namespace marv {

class LocalFrame {
public:
    void set(const GeoPoint& origin) {
        constexpr float kA = 6378137.f, kE2 = 6.69437999014e-3f;
        o_ = origin;
        const float lat = static_cast<float>(origin.lat_e7) * kRadPerE7;
        const float s = std::sin(lat);
        const float d = 1.f - kE2 * s * s;
        m_per_e7_n_ = (kA * (1.f - kE2) / (d * std::sqrt(d)) + origin.alt_m) * kRadPerE7;
        m_per_e7_e_ = (kA / std::sqrt(d) + origin.alt_m) * std::cos(lat) * kRadPerE7;
        valid_ = true;
    }
    bool valid() const { return valid_; }
    const GeoPoint& origin() const { return o_; }

    // Integer e7 differences first, so float keeps centimetres at any latitude.
    Vec3 to_ned(const GeoPoint& p) const {
        return {static_cast<float>(p.lat_e7 - o_.lat_e7) * m_per_e7_n_,
                static_cast<float>(p.lon_e7 - o_.lon_e7) * m_per_e7_e_, -(p.alt_m - o_.alt_m)};
    }
    GeoPoint to_geo(const Vec3& ned) const {
        return {o_.lat_e7 + static_cast<std::int32_t>(std::lround(ned.x / m_per_e7_n_)),
                o_.lon_e7 + static_cast<std::int32_t>(std::lround(ned.y / m_per_e7_e_)), o_.alt_m - ned.z};
    }

private:
    static constexpr float kRadPerE7 = 1e-7f * 3.14159265358979f / 180.f;
    GeoPoint o_{};
    float m_per_e7_n_ = 0.f, m_per_e7_e_ = 0.f;
    bool valid_ = false;
};

}  // namespace marv
