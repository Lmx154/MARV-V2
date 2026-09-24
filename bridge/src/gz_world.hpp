// The Gazebo side of the bridge: steps world "marv" one physics step at a time, turns what the step
// published into a SensorBus and a truth State, and forwards actuator commands to the motor models.
//
// Two ordering facts from gz-sim 8.15 / gz-transport 13 that the lockstep depends on:
//  - Every gz-sim publication leaves the server through ONE zmq PUB socket and reaches this process
//    over ONE connection, in publication order, handled by one reception thread.
//  - The server loop publishes /world/marv/clock at the start of every iteration, paused or not, and
//    the sensors publish in that iteration's PostUpdate. So a clock stamped t that arrives AFTER the
//    IMU stamped t comes from a later iteration, and every message of step t is already here.
#pragma once

#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>

#include <gz/msgs/clock.pb.h>
#include <gz/msgs/fluid_pressure.pb.h>
#include <gz/msgs/imu.pb.h>
#include <gz/msgs/magnetometer.pb.h>
#include <gz/msgs/navsat.pb.h>
#include <gz/msgs/odometry.pb.h>
#include <gz/transport/Node.hh>

#include <marv/fsw/contracts.hpp>

namespace marv::bridge {

class GzWorld {
public:
    GzWorld();

    // Waits until the world's clock is arriving and the motor models subscribe to the command topic.
    bool connect(std::string& err);

    // Advances the world one physics step. Updates bus in place: the IMU is always fresh, the slower
    // sensors are fresh only if they published during the step and otherwise keep their last value.
    bool step(SensorBus& bus, State& truth, std::string& err);

    // Sets the rotor speeds the motor models apply from the next step on.
    void command(const ActuatorCommand& cmd);

private:
    void on_clock(const gz::msgs::Clock& m);
    void on_imu(const gz::msgs::IMU& m);
    void on_odom(const gz::msgs::Odometry& m);
    void on_baro(const gz::msgs::FluidPressure& m);
    void on_mag(const gz::msgs::Magnetometer& m);
    void on_gnss(const gz::msgs::NavSat& m);

    gz::transport::Node node_;
    gz::transport::Node::Publisher motor_pub_;

    std::mutex mu_;
    std::condition_variable cv_;
    std::uint64_t clocks_ = 0;     // clock messages seen
    std::uint64_t settled_us_ = 0; // latest step whose messages have all arrived
    std::uint64_t imu_us_ = 0, odom_us_ = 0;
    gz::msgs::IMU imu_;
    gz::msgs::Odometry odom_;
    gz::msgs::FluidPressure baro_;
    gz::msgs::Magnetometer mag_;
    gz::msgs::NavSat gnss_;
    bool baro_new_ = false, mag_new_ = false, gnss_new_ = false;
    std::uint8_t seen_ = 0;  // SensorBit mask of the streams heard from since connect

    std::uint64_t t_us_ = 0;  // sim time of the last step taken
    bool have_origin_ = false;
    double origin_ned_[3] = {0, 0, 0};
};

}  // namespace marv::bridge
