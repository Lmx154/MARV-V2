// Parameter lookup by name in the generated manifest (params_manifest.json, written by tools/gen/params_gen.py).
#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

namespace marv::null_plant {

struct ManifestEntry {
  std::uint32_t id;
  bool is_f32;  // false: i32
};

// Finds the entry `"<name>": { "id": N, "type": "f32"|"i32", ... }` in the manifest text. Empty if the name is absent
// or the entry is malformed.
[[nodiscard]] std::optional<ManifestEntry> find_param(std::string_view manifest_text, std::string_view name);

[[nodiscard]] std::optional<std::string> read_file(const char* path);

}  // namespace marv::null_plant
