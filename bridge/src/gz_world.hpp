// The Gazebo side of the bridge: steps world "marv" a block of physics steps at a time, turns what each
// step published into a SensorBus and a truth State, and forwards actuator commands to the motor models.
//
// Two ordering facts from gz-sim 8.15 / gz-transport 13 that the lockstep depends on:
//  - Every gz-sim publication leaves the server through ONE zmq PUB socket and reaches this process
//    over ONE connection, in publication order, handled by one reception thread.
//  - The server loop publishes /world/marv/clock at the start of every iteration, paused or not, and
//    the sensors publish in that iteration's PostUpdate. So a clock stamped t that arrives AFTER the
//    IMU stamped t comes from a later iteration, and every message of step t is already here. For a
//    block, that is the clock after the block's last IMU: every message of the block is then here.
#pragma once

#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include <gz/msgs/clock.pb.h>
#include <gz/msgs/fluid_pressure.pb.h>
#include <gz/msgs/imu.pb.h>
#include <gz/msgs/magnetometer.pb.h>
#include <gz/msgs/navsat.pb.h>
#include <gz/msgs/odometry.pb.h>
#include <gz/transport/Node.hh>

#include <marv/fsw/contracts.hpp>

namespace marv::bridge {

// Physics steps per WorldControl request. A paused gz-sim 8.15 loop sleeps ~1 ms per iteration and acts on
// a request only at the end of an iteration (SimulationRunner.cc ~995), so one request per 1 ms step costs
// ~2.3 ms of wall time: 0.43 sim-s per wall-s. A block of 4 pays that once per 4 ms. The flight software
// still runs every 1 ms step, in order, on that step's own sensors; the motor models hold the block's last
// command through the next block, so command latency grows by at most 3 ms against a 12.5 ms motor time
// constant.
inline constexpr int kStepsPerBlock = 4;

class GzWorld {
public:
    struct Step {
        SensorBus bus;
        State truth;
    };

    // max_rot_velocity: the rotor speed, rad/s, of a motor command of 1 -- the world's maxRotVelocity. link: the
    // model's link that carries the sensors; their topics are /world/marv/model/marv_quad/link/<link>/sensor/...
    GzWorld(double max_rot_velocity, const std::string& link);

    // Waits until the world's clock is arriving and the motor models subscribe to the command topic.
    bool connect(std::string& err);

    // Advances the world kStepsPerBlock physics steps and fills one Step per physics step, in order. In each
    // bus the IMU is always fresh; the slower sensors are fresh only on the step they published and
    // otherwise keep their last value.
    bool step(Step (&block)[kStepsPerBlock], std::string& err);

    // Sets the rotor speeds the motor models apply from the next block on.
    void command(const ActuatorCommand& cmd);

private:
    void on_clock(const gz::msgs::Clock& m);
    void on_imu(const gz::msgs::IMU& m);
    void on_odom(const gz::msgs::Odometry& m);
    void on_baro(const gz::msgs::FluidPressure& m);
    void on_mag(const gz::msgs::Magnetometer& m);
    void on_gnss(const gz::msgs::NavSat& m);

    const double max_rot_velocity_;
    const std::string sensor_;  // the sensor topics' prefix
    gz::transport::Node node_;
    gz::transport::Node::Publisher motor_pub_;

    std::mutex mu_;
    std::condition_variable cv_;
    std::uint64_t clocks_ = 0;     // clock messages seen
    std::uint64_t settled_us_ = 0; // latest step whose messages have all arrived
    std::uint64_t imu_us_ = 0;     // stamp of the latest IMU sample
    // Every message of the block being taken, in arrival order; cleared before each request.
    std::vector<gz::msgs::IMU> imu_;
    std::vector<gz::msgs::Odometry> odom_;
    std::vector<gz::msgs::FluidPressure> baro_;
    std::vector<gz::msgs::Magnetometer> mag_;
    std::vector<gz::msgs::NavSat> gnss_;
    SensorBus bus_{};        // the last step's bus: the slower sensors' last values
    std::uint8_t seen_ = 0;  // SensorBit mask of the streams heard from since connect

    std::uint64_t t_us_ = 0;  // sim time of the last step taken
    bool have_origin_ = false;
    double origin_ned_[3] = {0, 0, 0};
};

}  // namespace marv::bridge
