#include "marv/null_plant/manifest.hpp"

#include <cstdio>
#include <cstdlib>
#include <string>

namespace marv::null_plant {

std::optional<ManifestEntry> find_param(std::string_view text, std::string_view name) {
  const std::string key = "\"" + std::string{name} + "\": {";
  const std::size_t at = text.find(key);
  if (at == std::string_view::npos) {
    return std::nullopt;
  }
  const std::size_t end = text.find('}', at);
  if (end == std::string_view::npos) {
    return std::nullopt;
  }
  const std::string_view body = text.substr(at + key.size(), end - (at + key.size()));

  const std::size_t id_at = body.find("\"id\":");
  const std::size_t type_at = body.find("\"type\":");
  if (id_at == std::string_view::npos || type_at == std::string_view::npos) {
    return std::nullopt;
  }
  const std::string id_text{body.substr(id_at + 5)};
  char* id_end = nullptr;
  const unsigned long id = std::strtoul(id_text.c_str(), &id_end, 10);
  if (id_end == id_text.c_str()) {
    return std::nullopt;
  }
  const std::string_view type_rest = body.substr(type_at + 7);
  const bool is_f32 = type_rest.find("\"f32\"") < type_rest.find("\"i32\"");
  const bool is_i32 = type_rest.find("\"i32\"") < type_rest.find("\"f32\"");
  if (is_f32 == is_i32) {
    return std::nullopt;
  }
  return ManifestEntry{static_cast<std::uint32_t>(id), is_f32};
}

std::optional<std::string> read_file(const char* path) {
  std::FILE* f = std::fopen(path, "rb");
  if (f == nullptr) {
    return std::nullopt;
  }
  std::string out;
  char buf[4096];
  std::size_t n = 0;
  while ((n = std::fread(buf, 1, sizeof buf, f)) > 0) {
    out.append(buf, n);
  }
  std::fclose(f);
  return out;
}

}  // namespace marv::null_plant
