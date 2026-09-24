#include "worldgen.hpp"

#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <map>
#include <sstream>

#include <boost/json.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

namespace marv::gcs::worldgen {
namespace json = boost::json;
namespace pt = boost::property_tree;
namespace fs = std::filesystem;

namespace {

constexpr double kPi = 3.14159265358979323846;

bool read_text(const std::string& path, std::string& out) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return false;
    std::ostringstream s;
    s << f.rdbuf();
    out = s.str();
    return true;
}

// The shortest fixed-point decimal (up to 12 places) that reads back as v, else 17 significant digits.
std::string num(double v) {
    char b[400];
    for (int p = 0; p <= 12; ++p) {
        std::snprintf(b, sizeof(b), "%.*f", p, v);
        if (std::strtod(b, nullptr) == v) return b;
    }
    std::snprintf(b, sizeof(b), "%.17g", v);
    return b;
}

// An SDF pose "x y z roll pitch yaw" at key under node; zeros when absent. False when present and not six numbers.
bool pose(const pt::ptree& node, const char* key, double (&p)[6]) {
    for (double& x : p) x = 0;
    const auto text = node.get_optional<std::string>(key);
    if (!text) return true;
    std::istringstream s(*text);
    for (double& x : p)
        if (!(s >> x)) return false;
    return true;
}

// g, m/s^2, from the template's <gravity>0 0 -g</gravity> (its comments mention the element too).
bool gravity(const std::string& tmpl, double& g) {
    for (auto a = tmpl.find("<gravity>"); a != std::string::npos; a = tmpl.find("<gravity>", a + 1)) {
        std::istringstream s(tmpl.substr(a + 9, 64));
        double x, y, z;
        std::string rest;
        if (s >> x >> y >> z >> rest && rest.rfind("</gravity>", 0) == 0 && x == 0 && y == 0 && z < 0) {
            g = -z;
            return true;
        }
    }
    return false;
}

// The specs of the model in model.sdf (see Specs), g the world's gravity.
bool parse_specs(const std::string& path, double g, Specs& s, std::string& err) {
    pt::ptree doc;
    try {
        pt::read_xml(path, doc, pt::xml_parser::no_comments | pt::xml_parser::trim_whitespace);
    } catch (const pt::xml_parser_error& e) {
        err = e.what();
        return false;
    }
    const auto model = doc.get_child_optional("sdf.model");
    if (!model) {
        err = path + ": no <sdf><model>";
        return false;
    }
    if (model->get<std::string>("<xmlattr>.name", "") != "marv_quad") {
        err = path + ": the model must be named marv_quad (the bridge's topics carry the name)";
        return false;
    }
    struct Link {
        double m, p[3], i[3];
    };
    std::map<std::string, Link> links;
    for (const auto& [key, link] : *model) {
        if (key != "link") continue;
        const std::string name = link.get<std::string>("<xmlattr>.name", "");
        double lp[6], ip[6];
        if (!pose(link, "pose", lp) || !pose(link, "inertial.pose", ip)) {
            err = path + ": link " + name + ": a pose is not six numbers";
            return false;
        }
        if (lp[3] != 0 || lp[4] != 0 || lp[5] != 0 || ip[3] != 0 || ip[4] != 0 || ip[5] != 0) {
            err = path + ": link " + name + ": a rotated link or inertial pose (not supported by the specs)";
            return false;
        }
        const auto m = link.get_optional<double>("inertial.mass");
        const auto ixx = link.get_optional<double>("inertial.inertia.ixx");
        const auto iyy = link.get_optional<double>("inertial.inertia.iyy");
        const auto izz = link.get_optional<double>("inertial.inertia.izz");
        if (!m || !ixx || !iyy || !izz || !(*m > 0)) {
            // SDF's defaults (mass 1, inertia 1) would be an invented airframe.
            err = path + ": link " + name + ": no inertial mass, ixx, iyy and izz";
            return false;
        }
        links[name] = {*m, {lp[0] + ip[0], lp[1] + ip[1], lp[2] + ip[2]}, {*ixx, *iyy, *izz}};
    }
    if (!links.count("base_link")) {
        err = path + ": no link named base_link (the bridge's sensor topics carry it)";
        return false;
    }

    // The whole vehicle: mass, centre of mass, inertia about it (parallel axes; every link frame is unrotated).
    double c[3] = {0, 0, 0};
    s.mass_kg = 0;
    for (const auto& [name, l] : links) {
        s.mass_kg += l.m;
        for (int k = 0; k < 3; ++k) c[k] += l.m * l.p[k];
    }
    for (double& x : c) x /= s.mass_kg;
    s.ixx = s.iyy = s.izz = 0;
    for (const auto& [name, l] : links) {
        const double r[3] = {l.p[0] - c[0], l.p[1] - c[1], l.p[2] - c[2]};
        s.ixx += l.i[0] + l.m * (r[1] * r[1] + r[2] * r[2]);
        s.iyy += l.i[1] + l.m * (r[0] * r[0] + r[2] * r[2]);
        s.izz += l.i[2] + l.m * (r[0] * r[0] + r[1] * r[1]);
    }

    // The motor models: one per rotor, all alike.
    s.rotors.clear();
    double arm = 0;
    bool first = true;
    for (const auto& [key, plugin] : *model) {
        if (key != "plugin" ||
            plugin.get<std::string>("<xmlattr>.filename", "").find("multicopter-motor-model") == std::string::npos)
            continue;
        const std::string link = plugin.get<std::string>("linkName", "");
        const auto up = plugin.get_optional<double>("timeConstantUp");
        const auto down = plugin.get_optional<double>("timeConstantDown");
        const auto wmax = plugin.get_optional<double>("maxRotVelocity");
        const auto kf = plugin.get_optional<double>("motorConstant");
        const auto km = plugin.get_optional<double>("momentConstant");
        if (!links.count(link) || !up || !down || !wmax || !kf || !km) {
            err = path + ": a motor model without a known linkName, timeConstantUp/Down, maxRotVelocity, motorConstant "
                         "or momentConstant";
            return false;
        }
        if (first) {
            s.time_constant_up = *up;
            s.time_constant_down = *down;
            s.max_rot_velocity = *wmax;
            s.motor_constant = *kf;
            s.moment_constant = *km;
            first = false;
        } else if (*up != s.time_constant_up || *down != s.time_constant_down || *wmax != s.max_rot_velocity ||
                   *kf != s.motor_constant || *km != s.moment_constant) {
            err = path + ": the motor models differ (" + link + "): the specs assume identical rotors";
            return false;
        }
        const Link& l = links[link];
        const double x = l.p[0] - c[0], y = l.p[1] - c[1];
        s.rotors.push_back({x, -y});  // FLU -> FRD
        arm += std::hypot(x, y);
    }
    s.rotor_count = static_cast<int>(s.rotors.size());
    if (s.rotor_count == 0 || !(s.max_rot_velocity > 0) || !(s.motor_constant > 0)) {
        err = path + ": no usable multicopter motor model";
        return false;
    }
    s.arm_m = arm / s.rotor_count;
    s.t_max_n = s.motor_constant * s.max_rot_velocity * s.max_rot_velocity;
    s.thrust_to_weight = s.rotor_count * s.t_max_n / (s.mass_kg * g);
    s.hover_thrust_frac = 1.0 / s.thrust_to_weight;
    return true;
}

// Replaces the one occurrence of marker in text. False when it is not there exactly once.
bool replace_once(std::string& text, const std::string& marker, const std::string& with) {
    const auto a = text.find(marker);
    if (a == std::string::npos || text.find(marker, a + 1) != std::string::npos) return false;
    text.replace(a, marker.size(), with);
    return true;
}

// Replaces the text of the one <tag>...</tag> in text.
bool replace_element(std::string& text, const std::string& tag, const std::string& with) {
    const std::string open = "<" + tag + ">", close = "</" + tag + ">";
    const auto a = text.find(open);
    if (a == std::string::npos) return false;
    const auto b = text.find(close, a);
    if (b == std::string::npos || text.find(open, b) != std::string::npos) return false;
    text.replace(a + open.size(), b - a - open.size(), with);
    return true;
}

// Removes every <visual> of the model text that names a model:// resource. Returns how many.
int drop_resource_visuals(std::string& text) {
    int n = 0;
    std::size_t pos = 0;
    while ((pos = text.find("<visual ", pos)) != std::string::npos) {
        const auto end = text.find("</visual>", pos);
        if (end == std::string::npos) break;
        const std::size_t stop = end + 9;
        if (text.substr(pos, stop - pos).find("model://") == std::string::npos) {
            pos = stop;
            continue;
        }
        const auto nl = text.rfind('\n', pos);
        const std::size_t from = nl == std::string::npos ? pos : nl;
        text.erase(from, stop - from);
        pos = from;
        ++n;
    }
    return n;
}

// The world's wind: nothing in calm air, else <wind> and the WindEffects system.
std::string wind_block(const Env& e) {
    if (e.wind_speed_ms == 0 && e.gust_sigma_ms == 0) return "<!-- Calm air: no <wind>, no WindEffects system. -->";
    double east, north;
    wind_enu(e.wind_speed_ms, e.wind_dir_deg, east, north);
    std::string s =
        "<!-- Wind " + num(e.wind_speed_ms) + " m/s from " + num(e.wind_dir_deg) +
        " deg (compass, the bearing it blows FROM), so toward bearing + 180: ENU air velocity (" + num(east) + ", " +
        num(north) +
        ", 0) m/s.\n"
        "         How gz-sim 8.15 applies it: the motor models' rotor drag, -|w| * rotorDragCoefficient * (the rotor's\n"
        "         velocity relative to the AIR, perpendicular to its axis) (Martin and Salaun 2010), reads the wind\n"
        "         entity's velocity (MulticopterMotorModel.cc:614-637 @ gz-sim8_8.15.0). WindEffects sets that velocity\n"
        "         every physics step: the magnitude rises from 0 with time_for_rise and carries Gaussian noise of sigma\n"
        "         gust_sigma_ms, drawn afresh every 1 ms step -- WHITE noise, not a Dryden or von Karman gust model\n"
        "         (WindEffects.cc:488-565). No link opts in to WindEffects' own force, mass * (air - link velocity)\n"
        "         (WindEffects.cc:568-609): it has no drag coefficient behind it and at its default scale is 10 N on\n"
        "         the x500 at 5 m/s. -->\n"
        "    <wind><linear_velocity>" +
        num(east) + " " + num(north) +
        " 0</linear_velocity></wind>\n"
        "    <plugin filename=\"gz-sim-wind-effects-system\" name=\"gz::sim::systems::WindEffects\">\n"
        "      <horizontal>\n"
        "        <magnitude>\n"
        "          <time_for_rise>1</time_for_rise>\n";
    if (e.gust_sigma_ms > 0)
        s += "          <noise type=\"gaussian\"><mean>0</mean><stddev>" + num(e.gust_sigma_ms) + "</stddev></noise>\n";
    s += "        </magnitude>\n"
         "      </horizontal>\n"
         "    </plugin>";
    return s;
}

}  // namespace

void wind_enu(double speed, double dir_deg, double& east, double& north) {
    const double to = (dir_deg + 180.0) * kPi / 180.0;  // the bearing it blows toward; east = sin, north = cos
    east = speed * std::sin(to);
    north = speed * std::cos(to);
    if (std::fabs(east) < 1e-9 * (1 + speed)) east = 0;
    if (std::fabs(north) < 1e-9 * (1 + speed)) north = 0;
}

std::vector<std::string> airframe_ids(const std::string& repo) {
    std::vector<std::string> ids;
    std::error_code ec;
    for (const auto& d : fs::directory_iterator(repo + "/sitl/airframes", ec))
        if (fs::is_regular_file(d.path() / "airframe.json", ec)) ids.push_back(d.path().filename().string());
    std::sort(ids.begin(), ids.end());
    return ids;
}

bool load_airframe(const std::string& repo, const std::string& id, Airframe& out, std::string& err) {
    if (id.empty() || id.find_first_not_of("abcdefghijklmnopqrstuvwxyz0123456789_-") != std::string::npos) {
        err = "airframe id '" + id + "': want [a-z0-9_-]+";
        return false;
    }
    const std::string dir = repo + "/sitl/airframes/" + id;
    std::string text, tmpl;
    if (!read_text(dir + "/airframe.json", text)) {
        err = "no airframe '" + id + "' (" + dir + "/airframe.json)";
        return false;
    }
    boost::system::error_code ec;
    const json::value v = json::parse(text, ec);
    if (ec || !v.is_object()) {
        err = dir + "/airframe.json: not a JSON object";
        return false;
    }
    const json::object& o = v.get_object();
    const auto str = [&](const char* key, std::string& s) {
        const json::value* x = o.if_contains(key);
        if (!x || !x->is_string()) return false;
        s = std::string(x->get_string());
        return true;
    };
    Airframe a;
    const json::value* z = o.if_contains("spawn_z");
    if (!str("id", a.id) || a.id != id || !str("label", a.label) || !str("frame", a.frame) || !str("source", a.source) ||
        !z || !z->is_number() || !(z->to_number<double>() >= 0)) {
        err = dir + "/airframe.json: want {id: \"" + id + "\", label, frame, source, spawn_z >= 0, mesh_path?}";
        return false;
    }
    a.spawn_z = z->to_number<double>();
    if (o.contains("mesh_path") && !str("mesh_path", a.mesh_path)) {
        err = dir + "/airframe.json: mesh_path is not a string";
        return false;
    }
    if (a.mesh_path.rfind("~/", 0) == 0) {
        const char* home = std::getenv("HOME");
        a.mesh_path = std::string(home ? home : "") + a.mesh_path.substr(1);
    }
    double g;
    if (!read_text(repo + "/sitl/gazebo/world.sdf.in", tmpl) || !gravity(tmpl, g)) {
        err = repo + "/sitl/gazebo/world.sdf.in: missing, or no <gravity>0 0 -g</gravity>";
        return false;
    }
    if (!parse_specs(dir + "/model.sdf", g, a.specs, err)) return false;
    if (a.frame == "quad-x" && a.specs.rotor_count != 4) {
        err = dir + ": frame quad-x with " + std::to_string(a.specs.rotor_count) + " motor models";
        return false;
    }
    out = std::move(a);
    return true;
}

bool env_from_json(const json::object& m, Env& out, std::string& err) {
    Env e;
    struct Field {
        const char* key;
        double* dst;
        double lo, hi;
    };
    const Field fields[] = {{"wind_speed_ms", &e.wind_speed_ms, 0, 40},  {"wind_dir_deg", &e.wind_dir_deg, 0, 360},
                            {"gust_sigma_ms", &e.gust_sigma_ms, 0, 20},  {"lat", &e.lat, -90, 90},
                            {"lon", &e.lon, -180, 180},                  {"elevation_m", &e.elevation_m, -500, 9000}};
    for (const Field& f : fields) {
        const json::value* v = m.if_contains(f.key);
        if (!v) continue;
        const double d = v->is_number() ? v->to_number<double>() : NAN;
        if (!(d >= f.lo && d <= f.hi)) {
            err = std::string("env.") + f.key + ": want a number in [" + num(f.lo) + ", " + num(f.hi) + "]";
            return false;
        }
        *f.dst = d;
    }
    const struct {
        const char* key;
        std::optional<double>* dst;
        double lo, hi;
    } optional[] = {{"temperature_c", &e.temperature_c, -90, 60}, {"pressure_pa", &e.pressure_pa, 30000, 110000}};
    for (const auto& f : optional) {
        const json::value* v = m.if_contains(f.key);
        if (!v || v->is_null()) continue;
        const double d = v->is_number() ? v->to_number<double>() : NAN;
        if (!(d >= f.lo && d <= f.hi)) {
            err = std::string("env.") + f.key + ": want null or a number in [" + num(f.lo) + ", " + num(f.hi) + "]";
            return false;
        }
        *f.dst = d;
    }
    out = e;
    return true;
}

json::object env_json(const Env& e) {
    return {{"wind_speed_ms", e.wind_speed_ms},
            {"wind_dir_deg", e.wind_dir_deg},
            {"gust_sigma_ms", e.gust_sigma_ms},
            {"lat", e.lat},
            {"lon", e.lon},
            {"elevation_m", e.elevation_m},
            {"temperature_c", e.temperature_c ? json::value(*e.temperature_c) : json::value(nullptr)},
            {"pressure_pa", e.pressure_pa ? json::value(*e.pressure_pa) : json::value(nullptr)}};
}

json::object airframe_json(const Airframe& a) {
    const Specs& s = a.specs;
    json::array rotors;
    for (const auto& r : s.rotors) rotors.push_back(json::array{r[0], r[1]});
    return {{"id", a.id},
            {"label", a.label},
            {"frame", a.frame},
            {"source", a.source},
            {"specs", json::object{{"mass_kg", s.mass_kg},
                                   {"ixx", s.ixx},
                                   {"iyy", s.iyy},
                                   {"izz", s.izz},
                                   {"arm_m", s.arm_m},
                                   {"rotor_count", s.rotor_count},
                                   {"motor_constant", s.motor_constant},
                                   {"moment_constant", s.moment_constant},
                                   {"max_rot_velocity", s.max_rot_velocity},
                                   {"time_constant_up", s.time_constant_up},
                                   {"time_constant_down", s.time_constant_down},
                                   {"t_max_n", s.t_max_n},
                                   {"thrust_to_weight", s.thrust_to_weight},
                                   {"hover_thrust_frac", s.hover_thrust_frac},
                                   {"rotors", std::move(rotors)}}}};
}

bool generate(const std::string& repo, const Airframe& a, const Env& env, const std::string& out_path,
              std::string& resource_path, std::vector<std::string>& notes, std::string& err) {
    json::object m = env_json(env);
    Env e;
    if (!env_from_json(m, e, err)) return false;
    std::string world, sensors, model;
    const std::string model_path = repo + "/sitl/airframes/" + a.id + "/model.sdf";
    if (!read_text(repo + "/sitl/gazebo/world.sdf.in", world) ||
        !read_text(repo + "/sitl/gazebo/fc3_sensors.sdf.in", sensors) || !read_text(model_path, model)) {
        err = "cannot read sitl/gazebo/world.sdf.in, sitl/gazebo/fc3_sensors.sdf.in or " + model_path;
        return false;
    }

    // The model's content: everything inside <model name="marv_quad">, with the sensors in its base_link.
    const std::string open = "<model name=\"marv_quad\">";
    const auto a0 = model.find(open), a1 = model.rfind("</model>");
    if (a0 == std::string::npos || a1 == std::string::npos || a1 < a0) {
        err = model_path + ": no <model name=\"marv_quad\">...</model>";
        return false;
    }
    std::string content = model.substr(a0 + open.size(), a1 - a0 - open.size());
    const auto b0 = content.find("<link name=\"base_link\">");
    const auto b1 = b0 == std::string::npos ? b0 : content.find("</link>", b0);
    if (b1 == std::string::npos) {
        err = model_path + ": no <link name=\"base_link\">...</link>";
        return false;
    }
    content.insert(content.rfind('\n', b1) + 1, sensors);

    resource_path.clear();
    if (!a.mesh_path.empty()) {
        std::error_code ec;
        if (fs::is_directory(a.mesh_path, ec)) {
            resource_path = a.mesh_path;
        } else {
            const int n = drop_resource_visuals(content);
            notes.push_back("no " + a.mesh_path + ": " + a.id + " flies with its " + std::to_string(n) +
                            " model:// visuals removed (the physics is unchanged; the viewer shows no aircraft)");
        }
    }
    while (!content.empty() && (content.front() == '\n' || content.front() == ' ')) content.erase(0, 1);
    while (!content.empty() && (content.back() == '\n' || content.back() == ' ')) content.pop_back();

    if (!replace_once(world, "@SPAWN_Z@", num(a.spawn_z)) || !replace_once(world, "<!-- @MODEL@ -->", content) ||
        !replace_once(world, "<!-- @WIND@ -->", wind_block(e)) || !replace_element(world, "latitude_deg", num(e.lat)) ||
        !replace_element(world, "longitude_deg", num(e.lon)) || !replace_element(world, "elevation", num(e.elevation_m))) {
        err = repo + "/sitl/gazebo/world.sdf.in: a marker or element the generator fills is missing or repeated";
        return false;
    }
    std::string env_text = "wind " + num(e.wind_speed_ms) + " m/s from " + num(e.wind_dir_deg) + " deg, gust sigma " +
                           num(e.gust_sigma_ms) + " m/s (white), at " + num(e.lat) + ", " + num(e.lon) + ", " +
                           num(e.elevation_m) + " m";
    world.insert(world.find('\n') + 1, "<!-- GENERATED by gcs/src/worldgen.cpp; do not edit. Airframe " + a.id + " (" +
                                           a.source + "); " + env_text + ". -->\n");

    if (e.temperature_c || e.pressure_pa)
        notes.push_back("temperature_c/pressure_pa are recorded, not simulated: gz-sim 8.15's air-pressure sensor uses "
                        "the ISA sea-level constants (101325 Pa, 288.15 K; gz-sensors AirPressureSensor.cc:47-49) and "
                        "no system the world loads reads <atmosphere>");
    if (e.lat != Env{}.lat || e.lon != Env{}.lon)
        notes.push_back("gz computes the magnetic field at " + num(e.lat) + ", " + num(e.lon) +
                        "; the flight software's Earth-field reference (params mag_ref_ned_ut_x/y) is Zurich's, so the "
                        "heading carries the declination difference until those are set for this place");

    std::error_code ec;
    fs::create_directories(fs::path(out_path).parent_path(), ec);
    const std::string tmp = out_path + ".tmp." + std::to_string(::getpid());
    {
        std::ofstream f(tmp, std::ios::binary | std::ios::trunc);
        f << world;
        if (!f.flush()) {
            err = "cannot write " + tmp;
            return false;
        }
    }
    if (std::rename(tmp.c_str(), out_path.c_str()) != 0) {
        std::remove(tmp.c_str());
        err = "cannot rename " + tmp + " to " + out_path;
        return false;
    }
    return true;
}

}  // namespace marv::gcs::worldgen
