// Generated from params.def: parameter indices, the Setup that holds a value for every parameter of every kind, the
// typed parameter struct of each (family, kind) filled from a Setup, the flight-side metadata table and the schema hash.
// The flight-side expansion carries identifiers and numbers only. A GCS build defines MARV_PARAMS_TEXT before the
// first include to also get kSchema, every row with every column. float only, no heap.
#pragma once

#include <cstdint>
#include <cstring>

namespace marv::param {

// ---- families and kinds ------------------------------------------------------------------------------------------------

// k_<family>: index into Setup::kind.
enum Family : std::uint8_t {
#define MARV_FAMILY(f, label) k_##f,
#include <marv/fsw/params.def>
    kFamilyCount
};
static_assert(kFamilyCount == 7, "seven families");

namespace detail {
enum KindRow : std::uint8_t {
#define MARV_KIND(f, k, name, label, summary) kKindRow_##f##_##k,
#include <marv/fsw/params.def>
    kKindRowCount
};
inline constexpr Family kKindRowFamily[kKindRowCount] = {
#define MARV_KIND(f, k, name, label, summary) k_##f,
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
#define MARV_KIND(f, k, name, label, summary) \
    inline constexpr std::uint8_t k_##f##_##k = detail::kind_index(detail::kKindRow_##f##_##k);
#include <marv/fsw/params.def>

// Number of kinds of a family.
constexpr std::uint8_t kind_count(std::uint8_t family) {
    std::uint8_t n = 0;
    for (std::uint8_t r = 0; r < detail::kKindRowCount; ++r)
        if (detail::kKindRowFamily[r] == family) ++n;
    return n;
}

// ---- parameters --------------------------------------------------------------------------------------------------------

// k_<family>_<kind>_<id>: the parameter's index into Setup::values.
enum ParamIndex : std::uint16_t {
#define MARV_PARAM(f, k, id, label, unit, dflt, min, max, step, digits, note, source) k_##f##_##k##_##id,
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

// Every kind (family, kind, wire name) and every parameter (family, kind, id), in table order.
constexpr std::uint32_t schema_hash() {
    std::uint32_t h = kFnvBasis;
#define MARV_KIND(f, k, name, label, summary) h = hash_kind(h, #f, #k, name);
#define MARV_PARAM(f, k, id, label, unit, dflt, min, max, step, digits, note, source) h = hash_param(h, #f, #k, #id);
#include <marv/fsw/params.def>
    return h;
}
inline constexpr std::uint32_t kSchemaHash = schema_hash();

// ---- typed parameters --------------------------------------------------------------------------------------------------

// One struct per (family, kind), a float per parameter initialised to its default, and <family>_<kind>(setup), which
// fills it from a Setup. A pass selects its rows by defining MARV_PARAMS_SEL_<family>_<kind> as "~, 1".
#define MARV_PARAMS_CAT_(a, b) a##b
#define MARV_PARAMS_CAT(a, b) MARV_PARAMS_CAT_(a, b)
#define MARV_PARAMS_SECOND_(a, b, ...) b
#define MARV_PARAMS_SECOND(...) MARV_PARAMS_SECOND_(__VA_ARGS__, 0, 0)
#define MARV_PARAMS_IF_0(...)
#define MARV_PARAMS_IF_1(...) __VA_ARGS__
#define MARV_PARAMS_WHEN(f, k) MARV_PARAMS_CAT(MARV_PARAMS_IF_, MARV_PARAMS_SECOND(MARV_PARAMS_SEL_##f##_##k))
#define MARV_PARAMS_FIELD(f, k, id, label, unit, dflt, ...) MARV_PARAMS_WHEN(f, k)(float id = dflt;)
#define MARV_PARAMS_FILL(f, k, id, ...) MARV_PARAMS_WHEN(f, k)(p.id = s.values[k_##f##_##k##_##id];)

#define MARV_PARAMS_SEL_vehicle_quad_x3 ~, 1
struct VehicleParams {
#define MARV_PARAM MARV_PARAMS_FIELD
#include <marv/fsw/params.def>
};
inline VehicleParams vehicle_quad_x3(const Setup& s) {
    VehicleParams p;
#define MARV_PARAM MARV_PARAMS_FILL
#include <marv/fsw/params.def>
    return p;
}
#undef MARV_PARAMS_SEL_vehicle_quad_x3

#define MARV_PARAMS_SEL_sensors_suite ~, 1
struct SensorParams {
#define MARV_PARAM MARV_PARAMS_FIELD
#include <marv/fsw/params.def>
};
inline SensorParams sensors_suite(const Setup& s) {
    SensorParams p;
#define MARV_PARAM MARV_PARAMS_FILL
#include <marv/fsw/params.def>
    return p;
}
#undef MARV_PARAMS_SEL_sensors_suite

// The ESKF and the EKF share their priors: the ekf rows name the same ids, in the same number.
#define MARV_PARAMS_SEL_estimator_eskf ~, 1
struct EskfPriors {
#define MARV_PARAM MARV_PARAMS_FIELD
#include <marv/fsw/params.def>
};
inline EskfPriors estimator_eskf(const Setup& s) {
    EskfPriors p;
#define MARV_PARAM MARV_PARAMS_FILL
#include <marv/fsw/params.def>
    return p;
}
#undef MARV_PARAMS_SEL_estimator_eskf
#define MARV_PARAMS_SEL_estimator_ekf ~, 1
inline EskfPriors estimator_ekf(const Setup& s) {
    EskfPriors p;
#define MARV_PARAM MARV_PARAMS_FILL
#include <marv/fsw/params.def>
    return p;
}
#undef MARV_PARAMS_SEL_estimator_ekf
static_assert(param_count(k_estimator, k_estimator_ekf) == param_count(k_estimator, k_estimator_eskf),
              "ekf and eskf rows name the same priors");

#define MARV_PARAMS_SEL_estimator_mahony ~, 1
struct MahonyParams {
#define MARV_PARAM MARV_PARAMS_FIELD
#include <marv/fsw/params.def>
};
inline MahonyParams estimator_mahony(const Setup& s) {
    MahonyParams p;
#define MARV_PARAM MARV_PARAMS_FILL
#include <marv/fsw/params.def>
    return p;
}
#undef MARV_PARAMS_SEL_estimator_mahony

#define MARV_PARAMS_SEL_estimator_complementary ~, 1
struct ComplementaryParams {
#define MARV_PARAM MARV_PARAMS_FIELD
#include <marv/fsw/params.def>
};
inline ComplementaryParams estimator_complementary(const Setup& s) {
    ComplementaryParams p;
#define MARV_PARAM MARV_PARAMS_FILL
#include <marv/fsw/params.def>
    return p;
}
#undef MARV_PARAMS_SEL_estimator_complementary

#define MARV_PARAMS_SEL_controller_cascaded_pid ~, 1
struct ControllerParams {
#define MARV_PARAM MARV_PARAMS_FIELD
#include <marv/fsw/params.def>
};
inline ControllerParams controller_cascaded_pid(const Setup& s) {
    ControllerParams p;
#define MARV_PARAM MARV_PARAMS_FILL
#include <marv/fsw/params.def>
    return p;
}
#undef MARV_PARAMS_SEL_controller_cascaded_pid

#define MARV_PARAMS_SEL_actuators_rotor_speed_fraction ~, 1
struct ActuatorParams {
#define MARV_PARAM MARV_PARAMS_FIELD
#include <marv/fsw/params.def>
};
inline ActuatorParams actuators_rotor_speed_fraction(const Setup& s) {
    ActuatorParams p;
#define MARV_PARAM MARV_PARAMS_FILL
#include <marv/fsw/params.def>
    return p;
}
#undef MARV_PARAMS_SEL_actuators_rotor_speed_fraction

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
// parameter rows up to the next part or kind.
struct SchemaRow {
    RowType type;
    std::uint8_t family;
    std::uint8_t kind;
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
#define MARV_FAMILY(f, label) {RowType::kFamily, k_##f, 0, #f, label, "", 0.f, 0.f, 0.f, 0.f, 0, "", ""},
#define MARV_KIND(f, k, name, label, summary) {RowType::kKind, k_##f, k_##f##_##k, name, label, "", 0.f, 0.f, 0.f, 0.f, 0, summary, ""},
#define MARV_PART(f, k, id, label) {RowType::kPart, k_##f, k_##f##_##k, #id, label, "", 0.f, 0.f, 0.f, 0.f, 0, "", ""},
#define MARV_PARAM(f, k, id, label, unit, dflt, min, max, step, digits, note, source) \
    {RowType::kParam, k_##f, k_##f##_##k, #id, label, unit, dflt, min, max, step, digits, note, source},
#include <marv/fsw/params.def>
};

// Wire names of the kinds, in table order (kind_count(f) consecutive entries per family).
inline constexpr const char* kKindName[detail::kKindRowCount] = {
#define MARV_KIND(f, k, name, label, summary) name,
#include <marv/fsw/params.def>
};
#endif

}  // namespace marv::param
