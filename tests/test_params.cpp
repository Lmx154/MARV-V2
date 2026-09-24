// The parameter table: unique ids, defaults within their ranges, every (family, kind) of the GCS and airframe decisions
// in its place with the vehicle classes it serves, and a schema hash that is stable and follows the ids.
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

// The hash of params.def recomputed here, with one parameter id optionally renamed.
constexpr std::uint32_t hash_with(const char* from, const char* to) {
    std::uint32_t h = kFnvBasis;
#define MARV_KIND(f, k, name, label, summary, vehicles) h = hash_kind(h, #f, #k, name);
#define MARV_PARAM(f, k, id, label, unit, dflt, min, max, step, digits, note, source) \
    h = hash_param(h, #f, #k, same(#id, from) ? to : #id);
#include <marv/fsw/params.def>
    return h;
}

static_assert(schema_hash() == kSchemaHash, "stable");
static_assert(hash_with("", "") == kSchemaHash, "the same algorithm over the same table");
static_assert(hash_with("mass", "mass_kg") != kSchemaHash, "a renamed id changes the hash");
static_assert(hash_with("k_vel", "k_vel") == kSchemaHash, "renamed to itself: unchanged");

int main() {
    constexpr std::size_t kRows = sizeof(kSchema) / sizeof(kSchema[0]);

    // Every parameter row, in order, is the index its enum names, and no (family, kind, id) repeats.
    {
        std::uint16_t index = 0;
        for (std::size_t r = 0; r < kRows; ++r) {
            if (kSchema[r].type != RowType::kParam) continue;
            CHECK(kParamMeta[index].id == index);
            CHECK(kParamMeta[index].family == kSchema[r].family && kParamMeta[index].kind == kSchema[r].kind);
            for (std::size_t q = r + 1; q < kRows; ++q)
                if (kSchema[q].type == RowType::kParam && kSchema[q].family == kSchema[r].family &&
                    kSchema[q].kind == kSchema[r].kind)
                    CHECK(std::strcmp(kSchema[q].id, kSchema[r].id) != 0);
            ++index;
        }
        CHECK(index == kParamCount);
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
            {k_guidance, k_guidance_passthrough, "passthrough", 0, kClassUav},
            {k_guidance, k_guidance_apogee_predictor, "apogee-predictor", 6, kClassRocket},
            {k_controller, k_controller_cascaded_pid, "cascaded-pid", 33, kClassUav},
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
            CHECK(kind_count(f) == (f == k_estimator ? 4 : f == k_sensors ? 1 : 2));
    }

    // Class consistency: each vehicle's first kinds, a kind of the other class refused, an out-of-range kind never.
    {
        const std::uint8_t uav[kFamilyCount] = {k_vehicle_uav, k_sensors_suite, k_estimator_eskf, k_guidance_passthrough,
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
