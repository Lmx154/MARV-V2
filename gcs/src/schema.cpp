#include "schema.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>

#include <boost/json.hpp>

#include <marv/fsw/params.hpp>
#include <marv/fsw/presets.hpp>

namespace marv::gcs {
namespace json = boost::json;

namespace {

using param::RowType;
using param::SchemaRow;
using param::kSchema;

constexpr std::size_t kRows = sizeof(kSchema) / sizeof(kSchema[0]);

json::object param_spec(const SchemaRow& r, std::uint16_t index) {
    json::object p;
    p["id"] = r.id;
    p["label"] = r.label;
    if (r.unit[0] != '\0') p["unit"] = r.unit;
    p["default"] = fnum(r.dflt);
    p["min"] = fnum(r.min);
    p["max"] = fnum(r.max);
    p["step"] = fnum(r.step);
    p["digits"] = r.digits;
    p["note"] = r.note;
    p["source"] = r.source;
    p["index"] = index;
    return p;
}

void pretty(std::string& out, const json::value& v, std::size_t depth) {
    if (v.is_object() && !v.get_object().empty()) {
        out += "{\n";
        bool first = true;
        for (const auto& kv : v.get_object()) {
            if (!first) out += ",\n";
            first = false;
            out.append(depth + 1, '\t');
            out += json::serialize(json::value(kv.key()));
            out += ": ";
            pretty(out, kv.value(), depth + 1);
        }
        out += '\n';
        out.append(depth, '\t');
        out += '}';
    } else if (v.is_array() && !v.get_array().empty()) {
        out += "[\n";
        bool first = true;
        for (const json::value& e : v.get_array()) {
            if (!first) out += ",\n";
            first = false;
            out.append(depth + 1, '\t');
            pretty(out, e, depth + 1);
        }
        out += '\n';
        out.append(depth, '\t');
        out += ']';
    } else if (v.is_double()) {
        char b[32];  // the shortest %g that reads back as the double, rather than the serializer's 1.52E0
        for (int p = 1; p <= 17; ++p) {
            std::snprintf(b, sizeof(b), "%.*g", p, v.get_double());
            if (std::strtod(b, nullptr) == v.get_double()) break;
        }
        out += b;
    } else {
        out += json::serialize(v);
    }
}

}  // namespace

json::value fnum(float f) {
    if (!std::isfinite(f)) return nullptr;
    char b[32];
    for (int p = 1; p <= 9; ++p) {
        std::snprintf(b, sizeof(b), "%.*g", p, static_cast<double>(f));
        if (std::strtof(b, nullptr) == f) break;
    }
    return std::strtod(b, nullptr);
}

json::value schema() {
    // A parameter's index is its order among the parameter rows.
    std::uint16_t index[kRows] = {};
    std::uint16_t n = 0;
    for (std::size_t i = 0; i < kRows; ++i)
        if (kSchema[i].type == RowType::kParam) index[i] = n++;

    json::array families;
    for (const SchemaRow& fr : kSchema) {
        if (fr.type != RowType::kFamily) continue;
        json::array kinds;
        for (std::size_t ki = 0; ki < kRows; ++ki) {
            const SchemaRow& kr = kSchema[ki];
            if (kr.type != RowType::kKind || kr.family != fr.family) continue;
            json::array params, parts;
            // The kind's parts and parameters are the rows up to the next kind.
            for (std::size_t i = ki + 1; i < kRows && kSchema[i].type != RowType::kKind; ++i) {
                const SchemaRow& r = kSchema[i];
                if (r.family != kr.family || r.kind != kr.kind) throw std::logic_error("params.def: row outside its kind");
                if (r.type == RowType::kPart) {
                    parts.push_back(json::object{{"id", r.id}, {"label", r.label}, {"params", json::array{}}});
                    continue;
                }
                params.push_back(param_spec(r, index[i]));
                if (!parts.empty()) parts.back().as_object()["params"].as_array().push_back(json::string(r.id));
            }
            // The vehicle kinds this kind serves, by wire name.
            json::array vehicles;
            for (std::size_t vi = 0, v = 0; vi < kRows; ++vi) {
                const SchemaRow& vr = kSchema[vi];
                if (vr.type != RowType::kKind || vr.family != param::k_vehicle) continue;
                if (param::compatible(kr.family, kr.kind, static_cast<std::uint8_t>(v++))) vehicles.push_back(json::string(vr.id));
            }
            json::object kind{{"id", kr.id}, {"label", kr.label}, {"summary", kr.note},
                              {"vehicles", std::move(vehicles)}, {"params", std::move(params)}};
            if (!parts.empty()) kind["parts"] = std::move(parts);
            kinds.push_back(std::move(kind));
        }
        families.push_back(json::object{{"id", fr.id}, {"label", fr.label}, {"kinds", std::move(kinds)}});
    }

    json::array factory;
    for (std::uint8_t i = 0; i < kPresetCount; ++i) {
        json::array kinds, values;
        for (std::uint8_t k : kFactory[i].kind) kinds.push_back(k);
        for (float v : kFactory[i].values) values.push_back(fnum(v));
        factory.push_back(json::object{{"id", kPresets[i].id}, {"label", kPresets[i].name}, {"kinds", std::move(kinds)},
                                       {"values", std::move(values)}});
    }
    return json::object{{"schema_hash", param::kSchemaHash}, {"families", std::move(families)},
                        {"factory", std::move(factory)}};
}

const std::string& schema_text() {
    static const std::string text = json::serialize(schema());
    return text;
}

std::string pretty(const json::value& v) {
    std::string out;
    pretty(out, v, 0);
    out += '\n';
    return out;
}

}  // namespace marv::gcs
