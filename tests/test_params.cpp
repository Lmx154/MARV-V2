// The parameter table: unique ids, defaults within their ranges, every (family, kind) of the GCS and airframe decisions
// in its place with the vehicle classes it serves, the four profiles and ADR-0012's profiled rows, and a schema hash that
// is stable and follows the ids.
#include <cstdio>
#include <cstdlib>
#include <cstring>

#define MARV_PARAMS_TEXT
#include <marv/fsw/params.hpp>

using namespace marv::param;

static int failures = 0;
#define CHECK(c)                                                         \
    do {                                                                 \
        if (!(c)) {                                                      \
            std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c);     \
            ++failures;                                                  \
        }                                                                \
    } while (0)

constexpr bool same(const char* a, const char* b) {
    for (; *a && *a == *b; ++a, ++b) {}
    return *a == *b;
}

// The hash of params.def recomputed here, with one profile or parameter id optionally renamed.
constexpr std::uint32_t hash_with(const char* from, const char* to) {
    std::uint32_t h = kFnvBasis;
#define MARV_PROFILE(id, label) h = fnv1a(fnv1a(h, "profile"), same(#id, from) ? to : #id);
#define MARV_KIND(f, k, name, label, summary, vehicles) h = hash_kind(h, #f, #k, name);
#define MARV_PARAM(f, k, id, label, unit, dflt, min, max, step, digits, note, source) \
    h = hash_param(h, #f, #k, same(#id, from) ? to : #id);
#define MARV_PROFILED(f, k, id, label, unit, d0, d1, d2, d3, min, max, step, digits, note, source) \
    h = fnv1a(hash_param(h, #f, #k, same(#id, from) ? to : #id), "profiled");
#include <marv/fsw/params.def>
    return h;
}

static_assert(schema_hash() == kSchemaHash, "stable");
static_assert(hash_with("", "") == kSchemaHash, "the same algorithm over the same table");
static_assert(hash_with("mass", "mass_kg") != kSchemaHash, "a renamed id changes the hash");
static_assert(hash_with("k_vel", "k_vel") == kSchemaHash, "renamed to itself: unchanged");
static_assert(hash_with("jerk", "jerk_max") != kSchemaHash, "a renamed profiled id changes the hash");
static_assert(hash_with("agile", "sport") != kSchemaHash, "a renamed profile changes the hash");

int main() {
    constexpr std::size_t kRows = sizeof(kSchema) / sizeof(kSchema[0]);

    // Every parameter row, in order, is the index its enum names, and no (family, kind, id, profile) repeats; a profiled
    // parameter is kProfileCount consecutive rows, one per profile in profile order.
    {
        std::uint16_t index = 0;
        for (std::size_t r = 0; r < kRows; ++r) {
            if (kSchema[r].type != RowType::kParam) continue;
            CHECK(kParamMeta[index].id == index);
            CHECK(kParamMeta[index].family == kSchema[r].family && kParamMeta[index].kind == kSchema[r].kind);
            const std::uint8_t pr = kSchema[r].profile;
            CHECK(pr == kShared || pr < kProfileCount);
            if (pr != kShared && pr > 0)
                CHECK(kSchema[r - 1].profile == pr - 1 && std::strcmp(kSchema[r - 1].id, kSchema[r].id) == 0);
            if (pr != kShared && pr + 1 < kProfileCount)
                CHECK(r + 1 < kRows && kSchema[r + 1].profile == pr + 1);
            for (std::size_t q = r + 1; q < kRows; ++q)
                if (kSchema[q].type == RowType::kParam && kSchema[q].family == kSchema[r].family &&
                    kSchema[q].kind == kSchema[r].kind)
                    CHECK(std::strcmp(kSchema[q].id, kSchema[r].id) != 0 || (pr != kShared && kSchema[q].profile != pr));
            ++index;
        }
        CHECK(index == kParamCount);
    }

    // The four profiles of ADR-0012, and its eight profiled rows: hold's default is the shared default they replace, each
    // profile's default at index + profile within the one range, and each profile's typed field filled from its own index.
    {
        const char* const ids[] = {"hold", "freestyle", "stabilized", "agile"};
        const char* const labels[] = {"Hold", "Freestyle", "Stabilized", "Agile"};
        CHECK(kProfileCount == 4 && k_profile_hold == 0 && k_profile_freestyle == 1 && k_profile_stabilized == 2 &&
              k_profile_agile == 3);
        for (std::uint8_t p = 0; p < kProfileCount; ++p)
            CHECK(std::strcmp(kProfileId[p], ids[p]) == 0 && std::strcmp(kProfileLabel[p], labels[p]) == 0);
        struct Row {
            std::uint16_t index;
            float dflt[4];
            float min, max;
        };
        const Row rows[] = {
            {k_guidance_trajectory_cruise_speed, {5.f, 10.f, 5.f, 5.f}, 0.5f, 20.f},
            {k_guidance_trajectory_acc_xy, {3.f, 5.f, 2.5f, 5.6f}, 2.f, 15.f},
            {k_guidance_trajectory_acc_up, {4.f, 4.f, 2.f, 5.6f}, 2.f, 15.f},
            {k_guidance_trajectory_acc_dn, {3.f, 3.f, 2.f, 5.5f}, 2.f, 15.f},
            {k_guidance_trajectory_jerk, {4.f, 8.f, 1.f, 8.9f}, 1.f, 80.f},
            {k_guidance_trajectory_yaw_rate_auto, {1.047f, 1.5f, 0.524f, 1.5f}, 0.087f, 6.283f},
            {k_controller_cascaded_pid_tilt_max_deg, {35.f, 45.f, 35.f, 45.f}, 5.f, 60.f},
            {k_controller_cascaded_pid_input_tc, {0.1f, 0.05f, 0.2f, 0.1f}, 0.01f, 1.f},
        };
        std::size_t profiled = 0;
        for (std::size_t r = 0; r < kRows; ++r) profiled += kSchema[r].type == RowType::kParam && kSchema[r].profile == 0;
        CHECK(profiled == sizeof rows / sizeof rows[0]);
        Setup s{};
        for (std::uint16_t i = 0; i < kParamCount; ++i) s.values[i] = static_cast<float>(i);
        const TrajectoryParams t = guidance_trajectory(s);
        const ControllerParams c = controller_cascaded_pid(s);
        const float* const fields[] = {t.cruise_speed, t.acc_xy, t.acc_up, t.acc_dn, t.jerk, t.yaw_rate_auto, c.tilt_max_deg,
                                       c.input_tc};
        const TrajectoryParams td{};
        const ControllerParams cd{};
        const float* const defaults[] = {td.cruise_speed, td.acc_xy, td.acc_up, td.acc_dn, td.jerk, td.yaw_rate_auto,
                                         cd.tilt_max_deg, cd.input_tc};
        for (std::size_t r = 0; r < sizeof rows / sizeof rows[0]; ++r)
            for (std::uint8_t p = 0; p < kProfileCount; ++p) {
                const std::uint16_t i = static_cast<std::uint16_t>(rows[r].index + p);
                const ParamMeta& m = kParamMeta[i];
                CHECK(m.dflt == rows[r].dflt[p] && m.min == rows[r].min && m.max == rows[r].max);
                CHECK(defaults[r][p] == rows[r].dflt[p]);
                if (fields[r] != t.acc_xy) CHECK(fields[r][p] == static_cast<float>(i));
            }
        CHECK(k_guidance_trajectory_cruise_speed_last == k_guidance_trajectory_cruise_speed + 3 &&
              k_guidance_trajectory_xy_vel_max == k_guidance_trajectory_cruise_speed_last + 1);
    }

    // spin_arm: appended after spin_max in the rotor-speed-fraction actuators' spin part, MOT_SPIN_ARM's default.
    {
        constexpr std::uint16_t i = k_actuators_rotor_speed_fraction_spin_arm;
        CHECK(i == k_actuators_rotor_speed_fraction_spin_max + 1);
        CHECK(kParamMeta[i].family == k_actuators && kParamMeta[i].kind == k_actuators_rotor_speed_fraction);
        CHECK(kParamMeta[i].dflt == 0.10f && kParamMeta[i].min == 0.f && kParamMeta[i].max == 0.3f);
        bool found = false;
        for (std::size_t r = 0; r < kRows; ++r) {
            if (kSchema[r].type != RowType::kParam || std::strcmp(kSchema[r].id, "spin_arm") != 0) continue;
            found = true;
            CHECK(std::strcmp(kSchema[r].label, "Spin when armed") == 0 && kSchema[r].step == 0.01f &&
                  kSchema[r].digits == 2 && kSchema[r].dflt == 0.10f);
        }
        CHECK(found);
    }

    // The trajectory guidance, the first guidance kind: its twelve rows follow the estimators', in order (a profiled row
    // taking four indices), with ADR-0011's defaults (a profiled row's hold) and ranges; the controller's velocity limit
    // reaches 20 m/s, its default PX4's MPC_XY_VEL_MAX 12 m/s.
    {
        struct Row {
            std::uint16_t index;
            float dflt, min, max;
        };
        const Row rows[] = {
            {k_guidance_trajectory_cruise_speed, 5.f, 0.5f, 20.f},
            {k_guidance_trajectory_xy_vel_max, 12.f, 0.5f, 20.f},
            {k_guidance_trajectory_z_vel_up, 3.f, 0.5f, 8.f},
            {k_guidance_trajectory_z_vel_dn, 1.5f, 0.5f, 4.f},
            {k_guidance_trajectory_acc_xy, 3.f, 2.f, 15.f},
            {k_guidance_trajectory_acc_up, 4.f, 2.f, 15.f},
            {k_guidance_trajectory_acc_dn, 3.f, 2.f, 15.f},
            {k_guidance_trajectory_jerk, 4.f, 1.f, 80.f},
            {k_guidance_trajectory_err_xy_max, 2.f, 0.f, 10.f},
            {k_guidance_trajectory_err_z_max, 1.f, 0.f, 10.f},
            {k_guidance_trajectory_yaw_rate_auto, 1.047f, 0.087f, 6.283f},
            {k_guidance_trajectory_heading_min_speed, 0.3f, 0.f, 2.f},
        };
        std::uint16_t next = k_estimator_complementary_k_baro + 1;
        for (std::size_t r = 0; r < sizeof rows / sizeof rows[0]; ++r) {
            const ParamMeta& m = kParamMeta[rows[r].index];
            CHECK(rows[r].index == next);
            next = static_cast<std::uint16_t>(next + (r == 0 || (r >= 4 && r <= 7) || r == 10 ? kProfileCount : 1));
            CHECK(m.family == k_guidance && m.kind == k_guidance_trajectory);
            CHECK(m.dflt == rows[r].dflt && m.min == rows[r].min && m.max == rows[r].max);
        }
        const ParamMeta& v = kParamMeta[k_controller_cascaded_pid_vel_max];
        CHECK(v.dflt == 12.f && v.min == 0.5f && v.max == 20.f);
        CHECK(first_compatible(k_guidance, k_vehicle_uav) == k_guidance_trajectory);
        CHECK(compatible(k_guidance, k_guidance_trajectory, k_vehicle_uav) &&
              !compatible(k_guidance, k_guidance_trajectory, k_vehicle_rocket));
    }

    // Defaults lie within [min, max]; the range is not empty; the table's default is the flight-side one.
    {
        std::uint16_t index = 0;
        for (std::size_t r = 0; r < kRows; ++r) {
            if (kSchema[r].type != RowType::kParam) continue;
            const ParamMeta& m = kParamMeta[index++];
            CHECK(m.min < m.max);
            CHECK(m.dflt >= m.min && m.dflt <= m.max);
            CHECK(m.dflt == kSchema[r].dflt && m.min == kSchema[r].min && m.max == kSchema[r].max);
            CHECK(kSchema[r].step > 0.f);
            if (!(m.dflt >= m.min && m.dflt <= m.max)) std::printf("  out of range: %s\n", kSchema[r].id);
        }
    }

    // The families and kinds of the decisions, in order, with their parameter counts and vehicle classes.
    {
        const char* const families[] = {"vehicle", "sensors", "estimator", "guidance", "controller", "allocation", "actuators"};
        struct Kind {
            Family family;
            std::uint8_t index;
            const char* name;
            std::uint16_t params;
            std::uint8_t vehicles;
        };
        constexpr std::uint8_t kBoth = kClassUav | kClassRocket;
        const Kind kinds[] = {
            {k_vehicle, k_vehicle_uav, "uav", 1, kClassUav},
            {k_vehicle, k_vehicle_rocket, "rocket", 0, kClassRocket},
            {k_sensors, k_sensors_suite, "suite", 18, kBoth},
            {k_estimator, k_estimator_eskf, "eskf", 4, kBoth},
            {k_estimator, k_estimator_ekf, "ekf", 4, kBoth},
            {k_estimator, k_estimator_mahony, "mahony", 6, kBoth},
            {k_estimator, k_estimator_complementary, "complementary", 6, kBoth},
            {k_guidance, k_guidance_trajectory, "trajectory", 12 + 6 * 3, kClassUav},
            {k_guidance, k_guidance_passthrough, "passthrough", 0, kClassUav},
            {k_guidance, k_guidance_apogee_predictor, "apogee-predictor", 6, kClassRocket},
            {k_controller, k_controller_cascaded_pid, "cascaded-pid", 33 + 2 * 3, kClassUav},
            {k_controller, k_controller_apogee_pid, "apogee-pid", 3, kClassRocket},
            {k_allocation, k_allocation_quad_x, "quad-x", 0, kClassUav},
            {k_allocation, k_allocation_rocket_brake, "rocket-brake", 0, kClassRocket},
            {k_actuators, k_actuators_rotor_speed_fraction, "rotor-speed-fraction", 4, kClassUav},
            {k_actuators, k_actuators_brake_servo, "brake-servo", 0, kClassRocket},
        };
        std::size_t nf = 0, nk = 0;
        for (std::size_t r = 0; r < kRows; ++r) {
            if (kSchema[r].type == RowType::kFamily) {
                CHECK(nf < kFamilyCount && kSchema[r].family == nf && std::strcmp(kSchema[r].id, families[nf]) == 0);
                ++nf;
            } else if (kSchema[r].type == RowType::kKind) {
                CHECK(nk < sizeof kinds / sizeof kinds[0]);
                const Kind& k = kinds[nk];
                CHECK(kSchema[r].family == k.family && kSchema[r].kind == k.index && std::strcmp(kSchema[r].id, k.name) == 0);
                CHECK(std::strcmp(kKindName[nk], k.name) == 0);
                CHECK(param_count(k.family, k.index) == k.params);
                CHECK(kind_vehicles(k.family, k.index) == k.vehicles);
                ++nk;
            }
        }
        CHECK(nf == kFamilyCount && nk == sizeof kinds / sizeof kinds[0]);
        CHECK(k_estimator_eskf == 0 && k_estimator_ekf == 1 && k_estimator_mahony == 2 && k_estimator_complementary == 3);
        for (std::uint8_t f = 0; f < kFamilyCount; ++f)
            CHECK(kind_count(f) == (f == k_estimator ? 4 : f == k_sensors ? 1 : f == k_guidance ? 3 : 2));
    }

    // Class consistency: each vehicle's first kinds, a kind of the other class refused, an out-of-range kind never.
    {
        const std::uint8_t uav[kFamilyCount] = {k_vehicle_uav, k_sensors_suite, k_estimator_eskf, k_guidance_trajectory,
                                                k_controller_cascaded_pid, k_allocation_quad_x,
                                                k_actuators_rotor_speed_fraction};
        const std::uint8_t rocket[kFamilyCount] = {k_vehicle_rocket, k_sensors_suite, k_estimator_eskf,
                                                   k_guidance_apogee_predictor, k_controller_apogee_pid,
                                                   k_allocation_rocket_brake, k_actuators_brake_servo};
        for (std::uint8_t f = 0; f < kFamilyCount; ++f) {
            CHECK(first_compatible(f, k_vehicle_uav) == uav[f]);
            CHECK(first_compatible(f, k_vehicle_rocket) == rocket[f]);
            CHECK(!compatible(f, kind_count(f), k_vehicle_uav) && !compatible(f, kind_count(f), k_vehicle_rocket));
        }
        CHECK(compatible(k_estimator, k_estimator_mahony, k_vehicle_rocket));
        CHECK(!compatible(k_controller, k_controller_cascaded_pid, k_vehicle_rocket));
        CHECK(!compatible(k_actuators, k_actuators_brake_servo, k_vehicle_uav));
        Setup s{};
        for (std::uint8_t f = 0; f < kFamilyCount; ++f) s.kind[f] = rocket[f];
        CHECK(consistent(s));
        s.kind[k_allocation] = k_allocation_quad_x;
        CHECK(!consistent(s));
        s.kind[k_allocation] = k_allocation_rocket_brake;
        s.kind[k_vehicle] = k_vehicle_uav;
        CHECK(!consistent(s));
    }

    // The hash, computed at run time over the same table, and a renamed id.
    {
        volatile std::uint32_t a = hash_with("", "");
        volatile std::uint32_t b = hash_with("sigma_p0", "sigma_pos0");
        CHECK(a == kSchemaHash);
        CHECK(b != kSchemaHash);
    }

    std::printf("kParamCount %u  sizeof(Setup) %zu  kSchemaHash 0x%08X\n", static_cast<unsigned>(kParamCount), sizeof(Setup),
                static_cast<unsigned>(kSchemaHash));
    std::printf("%s (%d failures)\n", failures ? "FAIL" : "PASS", failures);
    return failures ? EXIT_FAILURE : EXIT_SUCCESS;
}
