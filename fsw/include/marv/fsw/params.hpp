// Generated from params.def: parameter indices, the Setup that holds a value for every parameter of every kind, the
// typed parameter struct of each (family, kind) filled from a Setup, the flight-side metadata table and the schema hash.
// The flight-side expansion carries identifiers and numbers only. A GCS build defines MARV_PARAMS_TEXT before the
// first include to also get kSchema, every row with every column. float only, no heap.
#pragma once

#include <cmath>
#include <cstdint>
#include <cstring>

namespace marv::param {

// ---- families and kinds ------------------------------------------------------------------------------------------------

// Vehicle classes: the vehicles column of a kind is the set it serves; a vehicle kind serves its own class.
enum VehicleClass : std::uint8_t {
    kClassUav = 1u << 0,
    kClassRocket = 1u << 1,
};

// k_<family>: index into Setup::kind.
enum Family : std::uint8_t {
#define MARV_FAMILY(f, label) k_##f,
#include <marv/fsw/params.def>
    kFamilyCount
};
static_assert(kFamilyCount == 7, "seven families");

// k_profile_<id>: a flight profile, the column of every profiled parameter (MissionCommand::profile).
enum Profile : std::uint8_t {
#define MARV_PROFILE(id, label) k_profile_##id,
#include <marv/fsw/params.def>
    kProfileCount
};
static_assert(kProfileCount == 4 && k_profile_hold == 0, "four profiles, hold first");

namespace detail {
enum KindRow : std::uint8_t {
#define MARV_KIND(f, k, name, label, summary, vehicles) kKindRow_##f##_##k,
#include <marv/fsw/params.def>
    kKindRowCount
};
inline constexpr Family kKindRowFamily[kKindRowCount] = {
#define MARV_KIND(f, k, name, label, summary, vehicles) k_##f,
#include <marv/fsw/params.def>
};
inline constexpr std::uint8_t kKindRowVehicles[kKindRowCount] = {
#define MARV_KIND(f, k, name, label, summary, vehicles) static_cast<std::uint8_t>(vehicles),
#include <marv/fsw/params.def>
};
constexpr std::uint8_t kind_index(std::uint8_t row) {
    std::uint8_t n = 0;
    for (std::uint8_t r = 0; r < row; ++r)
        if (kKindRowFamily[r] == kKindRowFamily[row]) ++n;
    return n;
}
}  // namespace detail

// k_<family>_<kind>: the kind's index within its family, the value of Setup::kind[k_<family>].
#define MARV_KIND(f, k, name, label, summary, vehicles) \
    inline constexpr std::uint8_t k_##f##_##k = detail::kind_index(detail::kKindRow_##f##_##k);
#include <marv/fsw/params.def>

// Number of kinds of a family.
constexpr std::uint8_t kind_count(std::uint8_t family) {
    std::uint8_t n = 0;
    for (std::uint8_t r = 0; r < detail::kKindRowCount; ++r)
        if (detail::kKindRowFamily[r] == family) ++n;
    return n;
}

// The vehicle classes a kind of a family serves; 0 for a kind out of range.
constexpr std::uint8_t kind_vehicles(std::uint8_t family, std::uint8_t kind) {
    for (std::uint8_t r = 0; r < detail::kKindRowCount; ++r)
        if (detail::kKindRowFamily[r] == family && detail::kind_index(r) == kind) return detail::kKindRowVehicles[r];
    return 0;
}

// Whether a kind of a family serves the class of the vehicle kind.
constexpr bool compatible(std::uint8_t family, std::uint8_t kind, std::uint8_t vehicle) {
    return (kind_vehicles(family, kind) & kind_vehicles(k_vehicle, vehicle)) != 0;
}

// The first kind of a family that serves the vehicle kind's class (kind_count(family) if none).
constexpr std::uint8_t first_compatible(std::uint8_t family, std::uint8_t vehicle) {
    std::uint8_t k = 0;
    while (k < kind_count(family) && !compatible(family, k, vehicle)) ++k;
    return k;
}

// Every family has a kind for every vehicle kind, and each vehicle kind is exactly one class.
constexpr bool every_class_served() {
    for (std::uint8_t v = 0; v < kind_count(k_vehicle); ++v) {
        const std::uint8_t c = kind_vehicles(k_vehicle, v);
        if (c == 0 || (c & (c - 1)) != 0) return false;
        for (std::uint8_t f = 0; f < kFamilyCount; ++f)
            if (first_compatible(f, v) >= kind_count(f)) return false;
    }
    return true;
}
static_assert(every_class_served(), "every family has at least one kind per vehicle class");

// ---- parameters --------------------------------------------------------------------------------------------------------

// k_<family>_<kind>_<id>: the parameter's index into Setup::values; a profiled parameter's value for profile p is at
// k_<family>_<kind>_<id> + p, up to k_<family>_<kind>_<id>_last.
enum ParamIndex : std::uint16_t {
#define MARV_PARAM(f, k, id, label, unit, dflt, min, max, step, digits, note, source) k_##f##_##k##_##id,
#define MARV_PROFILED(f, k, id, label, unit, d0, d1, d2, d3, min, max, step, digits, note, source) \
    k_##f##_##k##_##id, k_##f##_##k##_##id##_last = k_##f##_##k##_##id + kProfileCount - 1,
#include <marv/fsw/params.def>
    kParamCount
};

struct ParamMeta {
    std::uint16_t id;  // ParamIndex, equal to the position in kParamMeta
    std::uint8_t family;
    std::uint8_t kind;
    float dflt;
    float min;
    float max;
};

inline constexpr ParamMeta kParamMeta[kParamCount] = {
#define MARV_PARAM(f, k, id, label, unit, dflt, min, max, step, digits, note, source) \
    {k_##f##_##k##_##id, k_##f, k_##f##_##k, dflt, min, max},
#define MARV_PROFILED(f, k, id, label, unit, d0, d1, d2, d3, min, max, step, digits, note, source) \
    {k_##f##_##k##_##id, k_##f, k_##f##_##k, d0, min, max},                                   \
    {k_##f##_##k##_##id + 1, k_##f, k_##f##_##k, d1, min, max},                               \
    {k_##f##_##k##_##id + 2, k_##f, k_##f##_##k, d2, min, max},                               \
    {k_##f##_##k##_##id + 3, k_##f, k_##f##_##k, d3, min, max},
#include <marv/fsw/params.def>
};

// Number of parameters of a (family, kind).
constexpr std::uint16_t param_count(std::uint8_t family, std::uint8_t kind) {
    std::uint16_t n = 0;
    for (std::uint16_t i = 0; i < kParamCount; ++i)
        if (kParamMeta[i].family == family && kParamMeta[i].kind == kind) ++n;
    return n;
}

// ---- setup -------------------------------------------------------------------------------------------------------------

// The kind of each family and a value for every parameter of every kind, so no index moves with the kinds chosen.
struct Setup {
    std::uint8_t kind[kFamilyCount];
    float values[kParamCount];
};

// CRC-32 (ISO-HDLC: reflected 0x04C11DB7, init and xorout 0xFFFFFFFF) over kind[] and then each value's bits,
// little-endian: independent of the struct's padding.
inline std::uint32_t setup_crc(const Setup& s) {
    std::uint32_t crc = 0xFFFFFFFFu;
    auto byte = [&crc](std::uint8_t b) {
        crc ^= b;
        for (int k = 0; k < 8; ++k) crc = (crc & 1u) ? (crc >> 1) ^ 0xEDB88320u : crc >> 1;
    };
    for (std::uint8_t k : s.kind) byte(k);
    for (float v : s.values) {
        std::uint32_t b;
        std::memcpy(&b, &v, 4);
        for (int i = 0; i < 4; ++i) byte(static_cast<std::uint8_t>(b >> (8 * i)));
    }
    return ~crc;
}

// Every family's kind serves the class of the setup's vehicle kind.
constexpr bool consistent(const Setup& s) {
    for (std::uint8_t f = 0; f < kFamilyCount; ++f)
        if (!compatible(f, s.kind[f], s.kind[k_vehicle])) return false;
    return true;
}

// ---- schema hash -------------------------------------------------------------------------------------------------------

// FNV-1a (32 bit) over each string and its terminating NUL.
inline constexpr std::uint32_t kFnvBasis = 2166136261u;
constexpr std::uint32_t fnv1a(std::uint32_t h, const char* s) {
    for (;; ++s) {
        h ^= static_cast<std::uint8_t>(*s);
        h *= 16777619u;
        if (*s == '\0') return h;
    }
}
constexpr std::uint32_t hash_kind(std::uint32_t h, const char* family, const char* kind, const char* name) {
    return fnv1a(fnv1a(fnv1a(h, family), kind), name);
}
constexpr std::uint32_t hash_param(std::uint32_t h, const char* family, const char* kind, const char* id) {
    return fnv1a(fnv1a(fnv1a(h, family), kind), id);
}

// Every profile (id), kind (family, kind, wire name) and parameter (family, kind, id; a profiled one marked), in table order.
constexpr std::uint32_t schema_hash() {
    std::uint32_t h = kFnvBasis;
#define MARV_PROFILE(id, label) h = fnv1a(fnv1a(h, "profile"), #id);
#define MARV_KIND(f, k, name, label, summary, vehicles) h = hash_kind(h, #f, #k, name);
#define MARV_PARAM(f, k, id, label, unit, dflt, min, max, step, digits, note, source) h = hash_param(h, #f, #k, #id);
#define MARV_PROFILED(f, k, id, label, unit, d0, d1, d2, d3, min, max, step, digits, note, source) \
    h = fnv1a(hash_param(h, #f, #k, #id), "profiled");
#include <marv/fsw/params.def>
    return h;
}
inline constexpr std::uint32_t kSchemaHash = schema_hash();

// ---- typed parameters --------------------------------------------------------------------------------------------------

// One struct per (family, kind), a float per parameter initialised to its default (a profiled one: float id[kProfileCount],
// by profile), and <family>_<kind>(setup), which fills it from a Setup. A pass selects its rows by defining
// MARV_PARAMS_SEL_<family>_<kind> as "~, 1".
#define MARV_PARAMS_CAT_(a, b) a##b
#define MARV_PARAMS_CAT(a, b) MARV_PARAMS_CAT_(a, b)
#define MARV_PARAMS_SECOND_(a, b, ...) b
#define MARV_PARAMS_SECOND(...) MARV_PARAMS_SECOND_(__VA_ARGS__, 0, 0)
#define MARV_PARAMS_IF_0(...)
#define MARV_PARAMS_IF_1(...) __VA_ARGS__
#define MARV_PARAMS_WHEN(f, k) MARV_PARAMS_CAT(MARV_PARAMS_IF_, MARV_PARAMS_SECOND(MARV_PARAMS_SEL_##f##_##k))
#define MARV_PARAMS_FIELD(f, k, id, label, unit, dflt, ...) MARV_PARAMS_WHEN(f, k)(float id = dflt;)
#define MARV_PARAMS_FILL(f, k, id, ...) MARV_PARAMS_WHEN(f, k)(p.id = s.values[k_##f##_##k##_##id];)
#define MARV_PARAMS_FIELD_PROFILED(f, k, id, label, unit, d0, d1, d2, d3, ...) \
    MARV_PARAMS_WHEN(f, k)(float id[kProfileCount] = {d0, d1, d2, d3};)
#define MARV_PARAMS_FILL_PROFILED(f, k, id, ...) \
    MARV_PARAMS_WHEN(f, k)(for (std::uint8_t i = 0; i < kProfileCount; ++i) p.id[i] = s.values[k_##f##_##k##_##id + i];)

#define MARV_PARAMS_SEL_vehicle_uav ~, 1
struct UavParams {
#define MARV_PARAM MARV_PARAMS_FIELD
#define MARV_PROFILED MARV_PARAMS_FIELD_PROFILED
#include <marv/fsw/params.def>
};
inline UavParams vehicle_uav(const Setup& s) {
    UavParams p;
#define MARV_PARAM MARV_PARAMS_FILL
#define MARV_PROFILED MARV_PARAMS_FILL_PROFILED
#include <marv/fsw/params.def>
    return p;
}
#undef MARV_PARAMS_SEL_vehicle_uav

#define MARV_PARAMS_SEL_sensors_suite ~, 1
struct SensorParams {
#define MARV_PARAM MARV_PARAMS_FIELD
#define MARV_PROFILED MARV_PARAMS_FIELD_PROFILED
#include <marv/fsw/params.def>
};
inline SensorParams sensors_suite(const Setup& s) {
    SensorParams p;
#define MARV_PARAM MARV_PARAMS_FILL
#define MARV_PROFILED MARV_PARAMS_FILL_PROFILED
#include <marv/fsw/params.def>
    return p;
}
#undef MARV_PARAMS_SEL_sensors_suite

// The ESKF and the EKF share their priors: the ekf rows name the same ids, in the same number.
#define MARV_PARAMS_SEL_estimator_eskf ~, 1
struct EskfPriors {
#define MARV_PARAM MARV_PARAMS_FIELD
#define MARV_PROFILED MARV_PARAMS_FIELD_PROFILED
#include <marv/fsw/params.def>
};
inline EskfPriors estimator_eskf(const Setup& s) {
    EskfPriors p;
#define MARV_PARAM MARV_PARAMS_FILL
#define MARV_PROFILED MARV_PARAMS_FILL_PROFILED
#include <marv/fsw/params.def>
    return p;
}
#undef MARV_PARAMS_SEL_estimator_eskf
#define MARV_PARAMS_SEL_estimator_ekf ~, 1
inline EskfPriors estimator_ekf(const Setup& s) {
    EskfPriors p;
#define MARV_PARAM MARV_PARAMS_FILL
#define MARV_PROFILED MARV_PARAMS_FILL_PROFILED
#include <marv/fsw/params.def>
    return p;
}
#undef MARV_PARAMS_SEL_estimator_ekf
static_assert(param_count(k_estimator, k_estimator_ekf) == param_count(k_estimator, k_estimator_eskf),
              "ekf and eskf rows name the same priors");

#define MARV_PARAMS_SEL_estimator_mahony ~, 1
struct MahonyParams {
#define MARV_PARAM MARV_PARAMS_FIELD
#define MARV_PROFILED MARV_PARAMS_FIELD_PROFILED
#include <marv/fsw/params.def>
};
inline MahonyParams estimator_mahony(const Setup& s) {
    MahonyParams p;
#define MARV_PARAM MARV_PARAMS_FILL
#define MARV_PROFILED MARV_PARAMS_FILL_PROFILED
#include <marv/fsw/params.def>
    return p;
}
#undef MARV_PARAMS_SEL_estimator_mahony

#define MARV_PARAMS_SEL_estimator_complementary ~, 1
struct ComplementaryParams {
#define MARV_PARAM MARV_PARAMS_FIELD
#define MARV_PROFILED MARV_PARAMS_FIELD_PROFILED
#include <marv/fsw/params.def>
};
inline ComplementaryParams estimator_complementary(const Setup& s) {
    ComplementaryParams p;
#define MARV_PARAM MARV_PARAMS_FILL
#define MARV_PROFILED MARV_PARAMS_FILL_PROFILED
#include <marv/fsw/params.def>
    return p;
}
#undef MARV_PARAMS_SEL_estimator_complementary

#define MARV_PARAMS_SEL_guidance_apogee_predictor ~, 1
struct ApogeePredictorParams {
#define MARV_PARAM MARV_PARAMS_FIELD
#define MARV_PROFILED MARV_PARAMS_FIELD_PROFILED
#include <marv/fsw/params.def>
};
inline ApogeePredictorParams guidance_apogee_predictor(const Setup& s) {
    ApogeePredictorParams p;
#define MARV_PARAM MARV_PARAMS_FILL
#define MARV_PROFILED MARV_PARAMS_FILL_PROFILED
#include <marv/fsw/params.def>
    return p;
}
#undef MARV_PARAMS_SEL_guidance_apogee_predictor

#define MARV_PARAMS_SEL_guidance_trajectory ~, 1
struct TrajectoryParams {
#define MARV_PARAM MARV_PARAMS_FIELD
#define MARV_PROFILED MARV_PARAMS_FIELD_PROFILED
#include <marv/fsw/params.def>
};
// Each profile's acc_xy within what its tilt limit holds with a third of the tilt to spare for correction:
// g tan(2/3 tilt_max) (ADR-0012; ArduPilot@0deeede043 libraries/AC_WPNav/AC_Loiter.cpp:237), g the sensors' gravity.
inline float acc_xy_limit(const Setup& s, std::uint8_t profile) {
    constexpr float kTwoThirdsRadPerDeg = 2.f / 3.f * 3.14159265358979f / 180.f;
    return s.values[k_sensors_suite_gravity] *
           std::tan(s.values[k_controller_cascaded_pid_tilt_max_deg + profile] * kTwoThirdsRadPerDeg);
}
inline TrajectoryParams guidance_trajectory(const Setup& s) {
    TrajectoryParams p;
#define MARV_PARAM MARV_PARAMS_FILL
#define MARV_PROFILED MARV_PARAMS_FILL_PROFILED
#include <marv/fsw/params.def>
    for (std::uint8_t i = 0; i < kProfileCount; ++i) p.acc_xy[i] = std::fmin(p.acc_xy[i], acc_xy_limit(s, i));
    return p;
}
#undef MARV_PARAMS_SEL_guidance_trajectory

#define MARV_PARAMS_SEL_controller_cascaded_pid ~, 1
struct ControllerParams {
#define MARV_PARAM MARV_PARAMS_FIELD
#define MARV_PROFILED MARV_PARAMS_FIELD_PROFILED
#include <marv/fsw/params.def>
};
inline ControllerParams controller_cascaded_pid(const Setup& s) {
    ControllerParams p;
#define MARV_PARAM MARV_PARAMS_FILL
#define MARV_PROFILED MARV_PARAMS_FILL_PROFILED
#include <marv/fsw/params.def>
    return p;
}
#undef MARV_PARAMS_SEL_controller_cascaded_pid

#define MARV_PARAMS_SEL_controller_apogee_pid ~, 1
struct ApogeePidParams {
#define MARV_PARAM MARV_PARAMS_FIELD
#define MARV_PROFILED MARV_PARAMS_FIELD_PROFILED
#include <marv/fsw/params.def>
};
inline ApogeePidParams controller_apogee_pid(const Setup& s) {
    ApogeePidParams p;
#define MARV_PARAM MARV_PARAMS_FILL
#define MARV_PROFILED MARV_PARAMS_FILL_PROFILED
#include <marv/fsw/params.def>
    return p;
}
#undef MARV_PARAMS_SEL_controller_apogee_pid

#define MARV_PARAMS_SEL_actuators_rotor_speed_fraction ~, 1
struct ActuatorParams {
#define MARV_PARAM MARV_PARAMS_FIELD
#define MARV_PROFILED MARV_PARAMS_FIELD_PROFILED
#include <marv/fsw/params.def>
};
inline ActuatorParams actuators_rotor_speed_fraction(const Setup& s) {
    ActuatorParams p;
#define MARV_PARAM MARV_PARAMS_FILL
#define MARV_PROFILED MARV_PARAMS_FILL_PROFILED
#include <marv/fsw/params.def>
    return p;
}
#undef MARV_PARAMS_SEL_actuators_rotor_speed_fraction

#undef MARV_PARAMS_FILL_PROFILED
#undef MARV_PARAMS_FIELD_PROFILED
#undef MARV_PARAMS_FILL
#undef MARV_PARAMS_FIELD
#undef MARV_PARAMS_WHEN
#undef MARV_PARAMS_IF_1
#undef MARV_PARAMS_IF_0
#undef MARV_PARAMS_SECOND
#undef MARV_PARAMS_SECOND_
#undef MARV_PARAMS_CAT
#undef MARV_PARAMS_CAT_

// ---- GCS: every column -------------------------------------------------------------------------------------------------

#ifdef MARV_PARAMS_TEXT
enum class RowType : std::uint8_t { kFamily, kKind, kPart, kParam };

// params.def as data, in file order. A kind row's id is its wire name and its note the summary; a part covers the
// parameter rows up to the next part or kind. A profiled parameter is four rows, one per profile, profile order.
inline constexpr std::uint8_t kShared = 0xFF;  // SchemaRow::profile of a parameter every profile shares
struct SchemaRow {
    RowType type;
    std::uint8_t family;
    std::uint8_t kind;
    std::uint8_t profile;  // Profile, or kShared
    const char* id;
    const char* label;
    const char* unit;
    float dflt;
    float min;
    float max;
    float step;
    std::uint8_t digits;
    const char* note;
    const char* source;
};

inline constexpr SchemaRow kSchema[] = {
#define MARV_FAMILY(f, label) {RowType::kFamily, k_##f, 0, kShared, #f, label, "", 0.f, 0.f, 0.f, 0.f, 0, "", ""},
#define MARV_KIND(f, k, name, label, summary, vehicles) {RowType::kKind, k_##f, k_##f##_##k, kShared, name, label, "", 0.f, 0.f, 0.f, 0.f, 0, summary, ""},
#define MARV_PART(f, k, id, label) {RowType::kPart, k_##f, k_##f##_##k, kShared, #id, label, "", 0.f, 0.f, 0.f, 0.f, 0, "", ""},
#define MARV_PARAM(f, k, id, label, unit, dflt, min, max, step, digits, note, source) \
    {RowType::kParam, k_##f, k_##f##_##k, kShared, #id, label, unit, dflt, min, max, step, digits, note, source},
#define MARV_PROFILED(f, k, id, label, unit, d0, d1, d2, d3, min, max, step, digits, note, source)      \
    {RowType::kParam, k_##f, k_##f##_##k, 0, #id, label, unit, d0, min, max, step, digits, note, source}, \
    {RowType::kParam, k_##f, k_##f##_##k, 1, #id, label, unit, d1, min, max, step, digits, note, source}, \
    {RowType::kParam, k_##f, k_##f##_##k, 2, #id, label, unit, d2, min, max, step, digits, note, source}, \
    {RowType::kParam, k_##f, k_##f##_##k, 3, #id, label, unit, d3, min, max, step, digits, note, source},
#include <marv/fsw/params.def>
};

// The profiles' ids (the last part of a profiled parameter's key) and labels, in profile order.
inline constexpr const char* kProfileId[kProfileCount] = {
#define MARV_PROFILE(id, label) #id,
#include <marv/fsw/params.def>
};
inline constexpr const char* kProfileLabel[kProfileCount] = {
#define MARV_PROFILE(id, label) label,
#include <marv/fsw/params.def>
};

// Wire names of the kinds, in table order (kind_count(f) consecutive entries per family).
inline constexpr const char* kKindName[detail::kKindRowCount] = {
#define MARV_KIND(f, k, name, label, summary, vehicles) name,
#include <marv/fsw/params.def>
};
#endif

}  // namespace marv::param
