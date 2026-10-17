// Writes vectors/cpp_frames.txt: the MAVLink frames the C++ codec produces for the sample messages (samples.hpp). The
// TypeScript suite decodes them (gcs, tests/regression/quad/L00/link/ts); codec_conformance_test.cpp fails when the committed
// file is not what this tool writes. Regenerate with the CMake target link_vectors_update.
#include <array>
#include <cstdio>
#include <fstream>
#include <vector>

#include "marv/link/codec_mavlink.hpp"
#include "samples.hpp"
#include "vectors.hpp"

int main(int argc, char** argv) {
  if (argc != 2) {
    std::fprintf(stderr, "usage: link_vectors_tool <out.txt>\n");
    return 2;
  }
  std::ofstream out(argv[1]);
  out << "# MAVLink v2 frames produced by the C++ codec (fw/link, link_vectors_tool) for the samples of samples.hpp.\n"
      << "# name system component sequence frame-hex\n";
  const auto all = marv::link::test::samples();
  for (std::size_t i = 0; i < all.size(); ++i) {
    std::array<std::uint8_t, 300> buffer{};
    const marv::link::Envelope envelope = marv::link::test::envelope_for(i);
    const std::size_t length = marv::link::encode_mavlink(envelope, all[i].message, buffer);
    if (length == 0) {
      std::fprintf(stderr, "encode failed for %s\n", all[i].name.c_str());
      return 1;
    }
    const std::vector<std::uint8_t> frame(buffer.begin(), buffer.begin() + static_cast<std::ptrdiff_t>(length));
    out << all[i].name << ' ' << static_cast<unsigned>(envelope.system_id) << ' '
        << static_cast<unsigned>(envelope.component_id) << ' ' << static_cast<unsigned>(envelope.sequence) << ' '
        << marv::link::test::to_hex(frame) << '\n';
  }
  return 0;
}
