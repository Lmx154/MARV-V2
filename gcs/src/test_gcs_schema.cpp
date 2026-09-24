// /api/schema against params.hpp and presets.hpp: every parameter once, at its index, with the table's id, default and
// range; the families and kinds in wire order, each kind with the vehicle kinds it serves; the factory setups value
// for value.
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include <boost/json.hpp>

#include <marv/fsw/params.hpp>
#include <marv/fsw/presets.hpp>

#include "schema.hpp"

namespace {

int g_fails = 0;
#define CHECK(c)                                                                  \
    do {                                                                          \
        if (!(c)) {                                                               \
            std::fprintf(stderr, "%s:%d: CHECK(%s) failed\n", __FILE__, __LINE__, #c); \
            ++g_fails;                                                            \
        }                                                                         \
    } while (0)

namespace json = boost::json;

float f32(const json::value& v) { return static_cast<float>(v.to_number<double>()); }

}  // namespace

int main() {
    using namespace marv;
    using namespace marv::param;

    // The parameter rows of the table, in order: row k is parameter k.
    std::vector<const SchemaRow*> rows;
    for (const SchemaRow& r : kSchema)
        if (r.type == RowType::kParam) rows.push_back(&r);
    CHECK(rows.size() == kParamCount);

    const json::value doc = json::parse(gcs::schema_text());
    const json::object& s = doc.as_object();
    CHECK(s.at("schema_hash").to_number<std::uint64_t>() == kSchemaHash);

    const json::array& families = s.at("families").as_array();
    CHECK(families.size() == kFamilyCount);
    std::vector<int> seen(kParamCount, 0);
    std::size_t count = 0, kind_row = 0;
    for (std::size_t f = 0; f < families.size(); ++f) {
        const json::object& fam = families[f].as_object();
        CHECK(fam.at("id").as_string() == kSchema[f].id);  // the family rows open the table
        const json::array& kinds = fam.at("kinds").as_array();
        CHECK(kinds.size() == kind_count(static_cast<std::uint8_t>(f)));
        for (std::size_t k = 0; k < kinds.size(); ++k, ++kind_row) {
            const json::object& kind = kinds[k].as_object();
            CHECK(kind.at("id").as_string() == kKindName[kind_row]);
            // vehicles: the vehicle kinds this kind serves, by id.
            const json::array& vehicles = kind.at("vehicles").as_array();
            const json::array& vkinds = families[k_vehicle].as_object().at("kinds").as_array();
            std::size_t served = 0;
            for (std::uint8_t v = 0; v < kind_count(k_vehicle); ++v) {
                bool listed = false;
                for (const json::value& id : vehicles) listed = listed || id == vkinds[v].as_object().at("id");
                CHECK(listed == compatible(static_cast<std::uint8_t>(f), static_cast<std::uint8_t>(k), v));
                served += listed;
            }
            CHECK(served == vehicles.size() && served > 0);
            std::vector<std::string> ids;
            for (const json::value& pv : kind.at("params").as_array()) {
                const json::object& p = pv.as_object();
                const std::uint64_t i = p.at("index").to_number<std::uint64_t>();
                CHECK(i < kParamCount);
                if (i >= kParamCount) continue;
                ++seen[i];
                ++count;
                ids.emplace_back(p.at("id").as_string());
                CHECK(kParamMeta[i].family == f && kParamMeta[i].kind == k);
                CHECK(p.at("id").as_string() == rows[i]->id);
                CHECK(f32(p.at("default")) == kParamMeta[i].dflt);
                CHECK(f32(p.at("min")) == kParamMeta[i].min);
                CHECK(f32(p.at("max")) == kParamMeta[i].max);
                CHECK(f32(p.at("step")) == rows[i]->step);
                CHECK(p.at("label").as_string() == rows[i]->label);
            }
            CHECK(ids.size() == param_count(static_cast<std::uint8_t>(f), static_cast<std::uint8_t>(k)));
            if (const json::value* parts = kind.if_contains("parts"))
                for (const json::value& part : parts->as_array())
                    for (const json::value& id : part.as_object().at("params").as_array()) {
                        bool found = false;
                        for (const std::string& x : ids) found = found || id.as_string() == x;
                        CHECK(found);
                    }
        }
    }
    CHECK(count == kParamCount);
    for (int n : seen) CHECK(n == 1);

    const json::array& factory = s.at("factory").as_array();
    CHECK(factory.size() == kPresetCount);
    for (std::size_t i = 0; i < factory.size() && i < kPresetCount; ++i) {
        const json::object& f = factory[i].as_object();
        CHECK(f.at("id").to_number<std::uint64_t>() == kPresets[i].id);
        CHECK(f.at("label").as_string() == kPresets[i].name);
        const json::array& kinds = f.at("kinds").as_array();
        CHECK(kinds.size() == kFamilyCount);
        for (std::size_t k = 0; k < kinds.size() && k < kFamilyCount; ++k)
            CHECK(kinds[k].to_number<std::uint64_t>() == kFactory[i].kind[k]);
        const json::array& values = f.at("values").as_array();
        CHECK(values.size() == kParamCount);
        for (std::size_t v = 0; v < values.size() && v < kParamCount; ++v) CHECK(f32(values[v]) == kFactory[i].values[v]);
    }

    if (g_fails) std::fprintf(stderr, "test_gcs_schema: %d failures\n", g_fails);
    else std::printf("test_gcs_schema: %u params, %u factory setups OK\n", static_cast<unsigned>(kParamCount),
                     static_cast<unsigned>(kPresetCount));
    return g_fails ? 1 : 0;
}
