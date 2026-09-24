// The SITL world generator: the airframe catalog sitl/airframes/<id>/ (model.sdf + airframe.json), each airframe's specs
// parsed from its model.sdf, and a Gazebo world written from sitl/gazebo/world.sdf.in, the airframe's model with
// sitl/gazebo/fc3_sensors.sdf.in in its base_link, and an environment (wind, gusts, location).
#pragma once

#include <array>
#include <optional>
#include <string>
#include <vector>

#include <boost/json/array.hpp>
#include <boost/json/object.hpp>

namespace marv::gcs::worldgen {

struct Specs {
    double mass_kg = 0;             // every link, rotors included
    double ixx = 0, iyy = 0, izz = 0;  // kg m^2, the whole vehicle about its centre of mass, body axes
    double arm_m = 0;               // mean horizontal distance of the rotor axes from the centre of mass
    int rotor_count = 0;
    double motor_constant = 0;      // N / (rad/s)^2
    double moment_constant = 0;     // m (yaw moment per newton of thrust)
    double max_rot_velocity = 0;    // rad/s: the rotor speed of a motor command of 1
    double time_constant_up = 0, time_constant_down = 0;  // s
    double t_max_n = 0;             // full thrust of ONE rotor, motor_constant * max_rot_velocity^2
    double thrust_to_weight = 0;    // rotor_count * t_max_n / (mass_kg * g)
    double hover_thrust_frac = 0;   // mass_kg * g / (rotor_count * t_max_n)
    std::vector<std::array<double, 2>> rotors;  // x forward, y right (FRD), m, about the centre of mass
};

struct Airframe {
    std::string id, label, frame, source;
    double spawn_z = 0;
    std::string mesh_path;  // airframe.json's, ~ expanded; empty when it has none
    Specs specs;
};

struct Env {
    double wind_speed_ms = 0;
    double wind_dir_deg = 0;  // compass bearing the wind blows FROM
    double gust_sigma_ms = 0;
    double lat = 47.376388, lon = 8.547778, elevation_m = 408;  // ETH Zurich, sitl/gazebo/world.sdf.in
    std::optional<double> temperature_c, pressure_pa;          // recorded; gz-sim 8.15 simulates neither
};

// The ENU air velocity of a wind of speed m/s blowing FROM compass bearing dir_deg (toward dir_deg + 180).
void wind_enu(double speed, double dir_deg, double& east, double& north);

// The catalog's airframe ids, sorted.
std::vector<std::string> airframe_ids(const std::string& repo);
// Reads sitl/airframes/<id>/airframe.json and parses model.sdf's specs. False, with err, when either is unusable.
bool load_airframe(const std::string& repo, const std::string& id, Airframe& out, std::string& err);

// env from a sim_launch's {wind_speed_ms, wind_dir_deg, gust_sigma_ms, lat, lon, elevation_m, temperature_c|null,
// pressure_pa|null}: every member optional (the defaults above), each a finite number in its range.
bool env_from_json(const boost::json::object& m, Env& out, std::string& err);
boost::json::object env_json(const Env& e);
// {id, label, frame, source, specs{...}} as GET /api/sim/airframes lists it.
boost::json::object airframe_json(const Airframe& a);

// Writes the world for airframe a in environment e to out_path (atomically: a temporary file, then a rename).
// resource_path: the directory GZ_SIM_RESOURCE_PATH needs for the model's meshes, or empty. notes: what the world
// leaves out (meshes not found; temperature and pressure), one line each.
bool generate(const std::string& repo, const Airframe& a, const Env& e, const std::string& out_path,
              std::string& resource_path, std::vector<std::string>& notes, std::string& err);

}  // namespace marv::gcs::worldgen
