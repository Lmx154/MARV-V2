// Reading and writing the frame-vector files (vectors/*.txt): one line per frame, `name system component sequence hex`;
// lines starting with '#' are comments.
#pragma once

#include <cstdint>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

#include "marv/link/codec.hpp"

namespace marv::link::test {

struct VectorLine {
  std::string name;
  Envelope envelope;
  std::vector<std::uint8_t> frame;
};

inline std::string to_hex(const std::vector<std::uint8_t>& bytes) {
  std::ostringstream out;
  for (const std::uint8_t byte : bytes) {
    out << std::hex << std::setw(2) << std::setfill('0') << static_cast<unsigned>(byte);
  }
  return out.str();
}

inline std::vector<VectorLine> read_vectors(const std::string& path) {
  std::vector<VectorLine> lines;
  std::ifstream in(path);
  std::string line;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#') {
      continue;
    }
    std::istringstream fields(line);
    VectorLine v;
    unsigned system = 0;
    unsigned component = 0;
    unsigned sequence = 0;
    std::string hex;
    fields >> v.name >> system >> component >> sequence >> hex;
    v.envelope = {static_cast<std::uint8_t>(system), static_cast<std::uint8_t>(component),
                  static_cast<std::uint8_t>(sequence)};
    for (std::size_t i = 0; i + 1 < hex.size(); i += 2) {
      v.frame.push_back(static_cast<std::uint8_t>(std::stoul(hex.substr(i, 2), nullptr, 16)));
    }
    lines.push_back(std::move(v));
  }
  return lines;
}

}  // namespace marv::link::test
