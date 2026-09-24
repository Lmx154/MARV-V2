// GET /api/schema: params.def with every column, and the factory setups (presets.hpp kFactory).
#pragma once

#include <string>

#include <boost/json/value.hpp>

namespace marv::gcs {

// {schema_hash, families[{id, label, kinds[{id, label, summary, vehicles[vehicle kind id], params[ParamSpec + index],
//  parts?}]}], factory[{id, label, kinds[7], values[kParamCount]}]}
boost::json::value schema();
// schema(), serialized once.
const std::string& schema_text();
// Tab-indented JSON, for --dump-schema.
std::string pretty(const boost::json::value& v);
// The shortest decimal that reads back as f, or null when f is not finite.
boost::json::value fnum(float f);

}  // namespace marv::gcs
