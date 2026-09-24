// earth_field_ned_ut against PX4's lookup test: NOAA WMM points with PX4's tolerances (model uncertainty plus the
// 10-degree grid's interpolation error), PX4-Autopilot @ af2e7b43 src/lib/world_magnetic_model/test_geo_lookup.cpp
// at the lines cited; and the Gazebo world's origin, Zurich.
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include <marv/fsw/geo_mag.hpp>

using namespace marv;

static int failures = 0;
#define NEAR(a, b, tol)                                                                              \
    do {                                                                                             \
        if (std::fabs((a) - (b)) > (tol)) {                                                          \
            std::printf("FAIL %s:%d  %s = %.6g, want %.6g\n", __FILE__, __LINE__, #a, (double)(a), (double)(b)); \
            ++failures;                                                                              \
        }                                                                                            \
    } while (0)

namespace {

struct Wmm {
    double decl_deg, incl_deg, f_nt;
};

Wmm wmm_of(std::int32_t lat_e7, std::int32_t lon_e7) {
    const Vec3 m = earth_field_ned_ut(lat_e7, lon_e7);
    const double n = m.x, e = m.y, d = m.z, h = std::hypot(n, e);
    constexpr double kDeg = 180.0 / 3.14159265358979323846;
    return {std::atan2(e, n) * kDeg, std::atan2(d, h) * kDeg, std::sqrt(h * h + d * d) * 1e3};
}

struct Point {
    int lat, lon;         // deg
    double decl, d_tol;   // deg
    double incl, i_tol;   // deg
    double f, f_tol;      // nT
    int line_d, line_i, line_f;
};

// EXPECT_NEAR(get_mag_*(lat, lon), expected, sigma + interpolation) of test_geo_lookup.cpp, at the lines given.
constexpr Point kPoints[] = {
    {-50, -180, 31.7, 0.40 + 1.0, -71.6, 0.21 + 1.2, 58366, 145 + 584, 43, 1726, 3409},
    {-50, 130, 2.5, 0.57 + 1.0, -80.3, 0.21 + 1.2, 65796, 145 + 658, 105, 1788, 3471},
    {-30, -60, -11.3, 0.41 + 1.0, -36.1, 0.21 + 1.2, 22227, 145 + 222, 359, 2042, 3725},
    {0, 0, -3.9, 0.33 + 1.0, -30.3, 0.21 + 1.2, 31945, 145 + 319, 809, 2492, 4175},
    {0, 90, -1.8, 0.29 + 1.0, -17.4, 0.21 + 1.2, 42694, 145 + 427, 827, 2510, 4193},
    {30, -120, 11.0, 0.34 + 1.0, 54.8, 0.21 + 1.2, 43575, 145 + 436, 1223, 2906, 4589},
    {45, 5, 2.6, 0.36 + 1.0, 60.9, 0.21 + 1.2, 47342, 145 + 473, 1467, 3150, 4833},
    {45, 10, 3.7, 0.36 + 1.0, 61.3, 0.21 + 1.2, 47694, 145 + 477, 1468, 3151, 4834},
    {50, 5, 2.6, 0.38 + 1.0, 65.5, 0.21 + 1.2, 48868, 145 + 489, 1540, 3223, 4906},
    {50, 10, 3.9, 0.38 + 1.0, 65.9, 0.21 + 1.2, 49154, 145 + 492, 1541, 3224, 4907},
    {50, 180, 2.4, 0.36 + 1.0, 61.9, 0.21 + 1.2, 48321, 145 + 483, 1575, 3258, 4941},
};

}  // namespace

int main() {
    for (const Point& p : kPoints) {
        const Wmm w = wmm_of(p.lat * 10000000, p.lon * 10000000);
        std::printf("(%4d, %4d) decl %8.3f (PX4 :%d %6.1f)  incl %8.3f (:%d %6.1f)  F %7.0f nT (:%d %6.0f)\n", p.lat, p.lon,
                    w.decl_deg, p.line_d, p.decl, w.incl_deg, p.line_i, p.incl, w.f_nt, p.line_f, p.f);
        NEAR(w.decl_deg, p.decl, p.d_tol);
        NEAR(w.incl_deg, p.incl, p.i_tol);
        NEAR(w.f_nt, p.f, p.f_tol);
    }
    // The Gazebo world's origin (sitl/gazebo/world.sdf.in), between the (45..50, 5..10) points above: the bilinear
    // value of the grid, to the table's resolution.
    const Wmm z = wmm_of(473763880, 85477780);
    std::printf("Zurich decl %.3f deg, incl %.3f deg, F %.0f nT\n", z.decl_deg, z.incl_deg, z.f_nt);
    NEAR(z.decl_deg, 3.405, 0.01);
    NEAR(z.incl_deg, 63.136, 0.01);
    NEAR(z.f_nt, 48232.0, 5.0);
    // Negative control: latitude and longitude swapped must miss Zurich's declination by more than PX4's tolerance.
    const Wmm swapped = wmm_of(85477780, 473763880);
    std::printf("negative control (lat, lon swapped): decl %.3f deg\n", swapped.decl_deg);
    if (!(std::fabs(swapped.decl_deg - 3.405) > 0.38 + 1.0)) {
        std::printf("FAIL negative control passed\n");
        ++failures;
    }
    std::printf("%s (%d failures)\n", failures ? "FAIL" : "PASS", failures);
    return failures ? EXIT_FAILURE : EXIT_SUCCESS;
}
