// marv_worldgen: writes the SITL world of a catalog airframe in calm air at the template's location (ETH Zurich), for
// scripts/sim.sh --world ID. The GCS launcher generates worlds with other environments through the same code.
//
//   marv_worldgen ID OUT.sdf
//
// On success prints, one per line, `max_rot_velocity=RAD_S` (the bridge's --max-rot-velocity) and
// `resource_path=DIR` (for GZ_SIM_RESOURCE_PATH; empty when the model needs none); notes go to stderr.
#include <cstdio>
#include <string>
#include <vector>

#include "worldgen.hpp"

int main(int argc, char** argv) {
    namespace wg = marv::gcs::worldgen;
    if (argc != 3) {
        std::fprintf(stderr, "usage: marv_worldgen ID OUT.sdf   (ID one of:");
        for (const auto& id : wg::airframe_ids(MARV_GCS_REPO_DIR)) std::fprintf(stderr, " %s", id.c_str());
        std::fprintf(stderr, ")\n");
        return 2;
    }
    wg::Airframe a;
    std::string err, resource_path;
    std::vector<std::string> notes;
    if (!wg::load_airframe(MARV_GCS_REPO_DIR, argv[1], a, err) ||
        !wg::generate(MARV_GCS_REPO_DIR, a, wg::Env{}, argv[2], resource_path, notes, err)) {
        std::fprintf(stderr, "marv_worldgen: %s\n", err.c_str());
        return 1;
    }
    for (const auto& n : notes) std::fprintf(stderr, "marv_worldgen: %s\n", n.c_str());
    std::printf("max_rot_velocity=%.17g\nresource_path=%s\n", a.specs.max_rot_velocity, resource_path.c_str());
    return 0;
}
