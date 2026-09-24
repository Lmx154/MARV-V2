// The world generator against the published airframes: the specs parsed from sitl/airframes/{x3,x500}/model.sdf equal
// the numbers of their sources (the X3 of Gazebo Fuel and gz-sim's multicopter_velocity_control.sdf; PX4's x500 @155f58e),
// the wind's compass convention, the environment's validation, and the generated worlds pass `gz sdf -k`.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <boost/json.hpp>

#include "worldgen.hpp"

namespace {

int g_fails = 0;
#define CHECK(c)                                                                  \
    do {                                                                          \
        if (!(c)) {                                                               \
            std::fprintf(stderr, "%s:%d: CHECK(%s) failed\n", __FILE__, __LINE__, #c); \
            ++g_fails;                                                            \
        }                                                                         \
    } while (0)

namespace wg = marv::gcs::worldgen;
namespace json = boost::json;

bool near(double a, double b, double rel = 1e-9) { return std::fabs(a - b) <= rel * std::fabs(b) + 1e-15; }

std::string slurp(const std::string& path) {
    std::ifstream f(path);
    std::ostringstream s;
    s << f.rdbuf();
    return s.str();
}

bool has(const std::string& text, const std::string& what) { return text.find(what) != std::string::npos; }

// `gz sdf -k path` exits 0. Without Gazebo installed the check is reported and skipped.
bool gz_valid(const std::string& path) {
#ifdef MARV_GZ
    const std::string cmd = std::string(MARV_GZ) + " sdf -k '" + path + "' >/dev/null 2>&1";
    return std::system(cmd.c_str()) == 0;
#else
    std::fprintf(stderr, "SKIP gz sdf -k %s: gz not installed\n", path.c_str());
    return true;
#endif
}

}  // namespace

int main() {
    const std::string repo = MARV_GCS_REPO_DIR;
    const std::string dir = std::string(MARV_GCS_BUILD_DIR) + "/sim/test_gcs_worldgen";
    std::filesystem::create_directories(dir);

    const auto ids = wg::airframe_ids(repo);
    CHECK(ids.size() >= 2 && ids[0] == "x3" && ids[1] == "x500");

    // x3: base_link 1.5 kg + 4 rotors 0.005 kg; motors 800 rad/s, 8.54858e-06 N/(rad/s)^2, 0.016 m, 0.0125/0.025 s.
    wg::Airframe x3;
    std::string err;
    CHECK(wg::load_airframe(repo, "x3", x3, err));
    const wg::Specs& a = x3.specs;
    CHECK(near(a.mass_kg, 1.52));
    CHECK(a.rotor_count == 4);
    CHECK(near(a.max_rot_velocity, 800) && near(a.motor_constant, 8.54858e-06) && near(a.moment_constant, 0.016));
    CHECK(near(a.time_constant_up, 0.0125) && near(a.time_constant_down, 0.025));
    CHECK(near(a.t_max_n, 8.54858e-06 * 800 * 800));                           // per rotor, 5.471 N
    CHECK(near(a.thrust_to_weight, 4 * 8.54858e-06 * 800 * 800 / (1.52 * 9.8066)));  // 1.468
    CHECK(near(a.hover_thrust_frac, 0.6811270117, 1e-8));
    // About the centre of mass (0, 0, 0.000303): the Fuel inertia plus the rotors' own and parallel-axis terms.
    CHECK(near(a.ixx, 0.03565464079, 1e-8) && near(a.iyy, 0.07051525719, 1e-8) && near(a.izz, 0.0990924164, 1e-8));
    CHECK(near(a.arm_m, 0.24703792781, 1e-8));
    // FRD: rotor_0 is at FLU (0.13, -0.22): front right.
    CHECK(a.rotors.size() == 4 && near(a.rotors[0][0], 0.13) && near(a.rotors[0][1], 0.22) &&
          near(a.rotors[1][0], -0.13) && near(a.rotors[1][1], -0.2));
    CHECK(near(x3.spawn_z, 0.055) && x3.mesh_path.empty() && x3.frame == "quad-x");

    // x500: base_link 2.0 kg + 4 rotors 0.016076923 kg at (+/-0.174, +/-0.174, 0.06); motors 1000 rad/s.
    wg::Airframe x5;
    CHECK(wg::load_airframe(repo, "x500", x5, err));
    const wg::Specs& b = x5.specs;
    CHECK(near(b.mass_kg, 2.0 + 4 * 0.016076923076923075));  // 2.064
    CHECK(near(b.max_rot_velocity, 1000) && near(b.t_max_n, 8.54858));
    CHECK(near(b.thrust_to_weight, 1.689122237, 1e-8) && near(b.hover_thrust_frac, 0.5920234652, 1e-8));
    CHECK(near(b.ixx, 0.02383948068, 1e-8) && near(b.iyy, 0.02394240549, 1e-8) && near(b.izz, 0.04399995371, 1e-8));
    CHECK(near(b.arm_m, 0.24607315985, 1e-8));
    CHECK(b.rotors.size() == 4 && near(b.rotors[0][0], 0.174) && near(b.rotors[0][1], 0.174));
    CHECK(near(x5.spawn_z, 0.227) && x5.mesh_path.rfind("/sim/PX4-gazebo-models/models") != std::string::npos);

    // The listing.
    const json::object j = wg::airframe_json(x5);
    CHECK(j.at("id").as_string() == "x500" && j.at("specs").as_object().at("rotors").as_array().size() == 4);
    CHECK(near(j.at("specs").as_object().at("mass_kg").to_number<double>(), b.mass_kg));

    // An unknown or malformed id is refused.
    wg::Airframe none;
    CHECK(!wg::load_airframe(repo, "nope", none, err) && has(err, "no airframe 'nope'"));
    CHECK(!wg::load_airframe(repo, "../x3", none, err));

    // Wind FROM a compass bearing blows TOWARD bearing + 180; ENU is (east, north).
    double e, n;
    wg::wind_enu(5, 270, e, n);  // from the west: toward the east
    CHECK(near(e, 5) && n == 0);
    wg::wind_enu(5, 0, e, n);  // from the north: toward the south
    CHECK(e == 0 && near(n, -5));
    wg::wind_enu(5, 90, e, n);
    CHECK(near(e, -5) && n == 0);
    wg::wind_enu(5, 180, e, n);
    CHECK(e == 0 && near(n, 5));
    wg::wind_enu(std::sqrt(2.0), 45, e, n);  // from the north-east: toward the south-west
    CHECK(near(e, -1, 1e-12) && near(n, -1, 1e-12));

    // The environment: defaults, ranges, null temperature/pressure.
    wg::Env env;
    CHECK(wg::env_from_json(json::object{}, env, err) && env.wind_speed_ms == 0 && near(env.lat, 47.376388) &&
          !env.temperature_c);
    CHECK(!wg::env_from_json(json::object{{"lat", 91}}, env, err) && has(err, "env.lat"));
    CHECK(!wg::env_from_json(json::object{{"wind_speed_ms", "5"}}, env, err));
    CHECK(!wg::env_from_json(json::object{{"wind_speed_ms", -1}}, env, err));
    CHECK(wg::env_from_json(json::object{{"temperature_c", nullptr}, {"pressure_pa", 95000}}, env, err) &&
          !env.temperature_c && env.pressure_pa && *env.pressure_pa == 95000);

    // Calm air at Zurich: the template's world, no wind.
    std::string res;
    std::vector<std::string> notes;
    const std::string calm = dir + "/x3-calm.sdf";
    CHECK(wg::generate(repo, x3, wg::Env{}, calm, res, notes, err));
    const std::string c = slurp(calm);
    CHECK(res.empty() && notes.empty());
    CHECK(has(c, "<pose>0 0 0.055 0 0 0</pose>") && has(c, "<link name=\"base_link\">") && has(c, "imu_sensor") &&
          has(c, "<latitude_deg>47.376388</latitude_deg>") && has(c, "<robot_base_frame>base_link</robot_base_frame>"));
    CHECK(!has(c, "gz-sim-wind-effects-system") && !has(c, "<linear_velocity>") && !has(c, "@"));
    CHECK(gz_valid(calm));

    // Wind 5 m/s from 270, gusts 0.5, elsewhere, 25 C: the x500, meshes resolved or dropped.
    wg::Env w;
    CHECK(wg::env_from_json(json::object{{"wind_speed_ms", 5}, {"wind_dir_deg", 270}, {"gust_sigma_ms", 0.5},
                                         {"lat", 40}, {"lon", -105.25}, {"elevation_m", 1650}, {"temperature_c", 25}},
                            w, err));
    notes.clear();
    const std::string windy = dir + "/x500-windy.sdf";
    CHECK(wg::generate(repo, x5, w, windy, res, notes, err));
    const std::string d = slurp(windy);
    CHECK(has(d, "<wind><linear_velocity>5 0 0</linear_velocity></wind>") &&
          has(d, "name=\"gz::sim::systems::WindEffects\"") && has(d, "<stddev>0.5</stddev>"));
    CHECK(has(d, "<latitude_deg>40</latitude_deg>") && has(d, "<longitude_deg>-105.25</longitude_deg>") &&
          has(d, "<elevation>1650</elevation>") && has(d, "<pose>0 0 0.227 0 0 0</pose>"));
    CHECK(!has(d, "<enable_wind>true"));
    CHECK(notes.size() == 2);  // temperature not simulated; the mag reference is Zurich's
    CHECK(res.empty() == !std::filesystem::is_directory(x5.mesh_path));
    CHECK(gz_valid(windy));

    // Without the PX4 checkout the model:// visuals go, and the world still validates.
    x5.mesh_path = dir + "/no-such-dir";
    notes.clear();
    const std::string bare = dir + "/x500-nomesh.sdf";
    CHECK(wg::generate(repo, x5, wg::Env{}, bare, res, notes, err));
    const std::string f = slurp(bare);
    CHECK(res.empty() && notes.size() == 1 && has(notes[0], "16 model:// visuals removed"));
    CHECK(!has(f, "model://") && has(f, "base_link_collision_4") && has(f, "rotor_3_collision"));
    CHECK(gz_valid(bare));

    // The validity check can fail: a world whose link has a non-numeric mass.
    const std::string broken = dir + "/broken.sdf";
    std::ofstream(broken) << "<?xml version=\"1.0\"?><sdf version=\"1.9\"><world name=\"w\"><model name=\"m\"><link "
                             "name=\"l\"><inertial><mass>heavy</mass></inertial></link></model></world></sdf>\n";
#ifdef MARV_GZ
    CHECK(!gz_valid(broken));
#endif

    if (g_fails) std::fprintf(stderr, "%d check(s) failed\n", g_fails);
    else std::printf("test_gcs_worldgen: all checks passed\n");
    return g_fails ? 1 : 0;
}
