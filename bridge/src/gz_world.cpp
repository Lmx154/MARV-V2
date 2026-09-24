#include "gz_world.hpp"

#include <chrono>
#include <cmath>

#include <gz/msgs/actuators.pb.h>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/world_control.pb.h>

namespace marv::bridge {
namespace {

const std::string kControl = "/world/marv/control";
const std::string kClock = "/world/marv/clock";
const std::string kModel = "marv_quad";
const std::string kOdomTopic = "/world/marv/model/" + kModel + "/odometry";
const std::string kMotor = "/" + kModel + "/command/motor_speed";

std::uint64_t to_us(const gz::msgs::Time& t) {
    return static_cast<std::uint64_t>(t.sec()) * 1000000u + static_cast<std::uint64_t>(t.nsec()) / 1000u;
}

// Gazebo's body frame is FLU; the contracts' is FRD.
Vec3 flu_to_frd(const gz::msgs::Vector3d& v, double scale) {
    return {static_cast<float>(v.x() * scale), static_cast<float>(-v.y() * scale),
            static_cast<float>(-v.z() * scale)};
}

struct Qd {
    double w, x, y, z;
};

Qd mul(const Qd& a, const Qd& b) {
    return {a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z, a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x, a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
}

// Rotates v by the unit quaternion q.
void rotate(const Qd& q, const double v[3], double out[3]) {
    const Qd r = mul(mul(q, {0, v[0], v[1], v[2]}), {q.w, -q.x, -q.y, -q.z});
    out[0] = r.x;
    out[1] = r.y;
    out[2] = r.z;
}

}  // namespace

GzWorld::GzWorld(double max_rot_velocity, const std::string& link)
    : max_rot_velocity_(max_rot_velocity),
      sensor_("/world/marv/model/" + kModel + "/link/" + link + "/sensor/"),
      motor_pub_(node_.Advertise<gz::msgs::Actuators>(kMotor)) {}

bool GzWorld::connect(std::string& err) {
    if (!motor_pub_ || !node_.Subscribe(sensor_ + "imu_sensor/imu", &GzWorld::on_imu, this) ||
        !node_.Subscribe(kOdomTopic, &GzWorld::on_odom, this) ||
        !node_.Subscribe(sensor_ + "baro_sensor/air_pressure", &GzWorld::on_baro, this) ||
        !node_.Subscribe(sensor_ + "mag_sensor/magnetometer", &GzWorld::on_mag, this) ||
        !node_.Subscribe(sensor_ + "gps_sensor/navsat", &GzWorld::on_gnss, this) ||
        // Last. The first clock proves this process's filters on the server's PUB connection, but the server
        // publishes a topic here only once its discovery has registered this subscriber for that topic: one UDP
        // datagram per topic, never resent. scripts/sim.sh keeps that traffic small enough not to be dropped
        // (GZ_IP); step()'s 250 ms check catches a stream lost anyway.
        !node_.Subscribe(kClock, &GzWorld::on_clock, this)) {
        err = "cannot advertise or subscribe the gz topics";
        return false;
    }
    std::unique_lock<std::mutex> lk(mu_);
    const bool ok = cv_.wait_for(lk, std::chrono::seconds(10), [&] {
        return clocks_ > 0 && motor_pub_.HasConnections();
    });
    if (!ok) {
        err = clocks_ == 0 ? "no " + kClock + " within 10 s: is the world running?"
                           : "the motor models never subscribed to " + kMotor + " within 10 s";
        return false;
    }
    if (imu_us_ != 0) {
        err = "the world is not paused (IMU samples are arriving unrequested): start it without -r";
        return false;
    }
    return true;
}

bool GzWorld::step(Step (&block)[kStepsPerBlock], std::string& err) {
    {
        std::lock_guard<std::mutex> lk(mu_);
        imu_.clear();
        odom_.clear();
        baro_.clear();
        mag_.clear();
        gnss_.clear();
    }
    gz::msgs::WorldControl req;
    req.set_pause(true);
    req.set_multi_step(kStepsPerBlock);
    gz::msgs::Boolean rep;
    bool result = false;
    if (!node_.Request(kControl, req, 1000, rep, result) || !result || !rep.data()) {
        err = kControl + " did not accept the step request";
        return false;
    }

    const auto n = static_cast<std::size_t>(kStepsPerBlock);
    std::unique_lock<std::mutex> lk(mu_);
    if (!cv_.wait_for(lk, std::chrono::seconds(2), [&] { return imu_.size() >= n && settled_us_ == imu_us_; })) {
        err = "no " + std::to_string(n) + " IMU samples after t_us=" + std::to_string(t_us_) + " within 2 s (got " +
              std::to_string(imu_.size()) + ")";
        return false;
    }
    if (imu_.size() != n) {
        err = std::to_string(imu_.size()) + " IMU samples for a block of " + std::to_string(n) + " steps";
        return false;
    }
    std::size_t placed = 0;  // baro/mag/GNSS messages matched to a step of the block
    for (std::size_t k = 0; k < n; ++k) {
        const gz::msgs::IMU& imu = imu_[k];
        const std::uint64_t t = to_us(imu.header().stamp());
        if (t <= t_us_) {
            err = "IMU stamped t_us=" + std::to_string(t) + " after t_us=" + std::to_string(t_us_);
            return false;
        }
        const gz::msgs::Odometry* odom = nullptr;
        for (const auto& m : odom_)
            if (to_us(m.header().stamp()) == t) odom = &m;
        // OdometryPublisher only records its start time on the world's first step (OdometryPublisher.cc:
        // 368-373), so that one step has no truth. Any other gap is an error.
        if (!odom && t_us_ != 0) {
            err = "no odometry stamped t_us=" + std::to_string(t);
            return false;
        }
        t_us_ = t;

        SensorBus& bus = bus_;
        bus.t_us = t;
        bus.fresh = kImu;
        bus.imu.accel_frd = flu_to_frd(imu.linear_acceleration(), 1.0);
        bus.imu.gyro_frd = flu_to_frd(imu.angular_velocity(), 1.0);
        for (const auto& m : baro_) {
            if (to_us(m.header().stamp()) != t) continue;
            ++placed;
            bus.fresh |= kBaro;
            bus.baro.pressure_pa = static_cast<float>(m.pressure());
            // Gazebo's air-pressure sensor does not model temperature; 15 C is the ISA sea-level value.
            bus.baro.temperature_c = 15.0f;
        }
        for (const auto& m : mag_) {
            if (to_us(m.header().stamp()) != t) continue;
            ++placed;
            bus.fresh |= kMag;
            bus.mag.field_frd_ut = flu_to_frd(m.field_tesla(), 1e6);
        }
        for (const auto& m : gnss_) {
            if (to_us(m.header().stamp()) != t) continue;
            ++placed;
            bus.fresh |= kGnss;
            bus.gnss.lat_e7 = static_cast<std::int32_t>(std::llround(m.latitude_deg() * 1e7));
            bus.gnss.lon_e7 = static_cast<std::int32_t>(std::llround(m.longitude_deg() * 1e7));
            bus.gnss.alt_m = static_cast<float>(m.altitude());
            bus.gnss.vel_ned = {static_cast<float>(m.velocity_north()), static_cast<float>(m.velocity_east()),
                                static_cast<float>(-m.velocity_up())};
            bus.gnss.fix = true;
        }
        // A sensor stream that never starts is a lost subscription, not a sensor fault: fly nothing on it. Every
        // stream publishes by t = 1 ms and the slowest (GNSS, 10 Hz) again by 101 ms; 250 ms leaves margin.
        seen_ |= bus.fresh;
        if (t >= 250000 && seen_ != (kImu | kBaro | kMag | kGnss)) {
            err = std::string("no message from") + (seen_ & kBaro ? "" : " baro") + (seen_ & kMag ? "" : " mag") +
                  (seen_ & kGnss ? "" : " gnss") + " in the first 250 ms: the subscription failed, rerun";
            return false;
        }
        block[k].bus = bus;

        State& truth = block[k].truth;
        truth.t_us = t;
        truth.valid = odom != nullptr;
        if (!truth.valid) {
            const float nan = std::nanf("");
            truth.p_ned = truth.v_ned = truth.w_frd = {nan, nan, nan};
            truth.q = {nan, nan, nan, nan};
            continue;
        }
        // Truth. World ENU -> NED is (n, e, d) = (y, x, -z), a half turn about (1, 1, 0)/sqrt2; body FLU ->
        // FRD is a half turn about x. So q(FRD -> NED) = q(ENU -> NED) * q(FLU -> ENU) * q(FRD -> FLU).
        const auto& pose = odom->pose();
        const Qd q_gz{pose.orientation().w(), pose.orientation().x(), pose.orientation().y(), pose.orientation().z()};
        const double s = std::sqrt(0.5);
        Qd q = mul(mul(Qd{0, s, s, 0}, q_gz), Qd{0, 1, 0, 0});
        if (q.w < 0) q = {-q.w, -q.x, -q.y, -q.z};
        const double p_ned[3] = {pose.position().y(), pose.position().x(), -pose.position().z()};
        if (!have_origin_) {
            for (int i = 0; i < 3; ++i) origin_ned_[i] = p_ned[i];
            have_origin_ = true;
        }
        // The odometry twist is in the body (FLU) frame, a 10-sample rolling mean (OdometryPublisher.cc).
        const double v_flu[3] = {odom->twist().linear().x(), odom->twist().linear().y(), odom->twist().linear().z()};
        double v_enu[3];
        rotate(q_gz, v_flu, v_enu);
        truth.p_ned = {static_cast<float>(p_ned[0] - origin_ned_[0]), static_cast<float>(p_ned[1] - origin_ned_[1]),
                       static_cast<float>(p_ned[2] - origin_ned_[2])};
        truth.v_ned = {static_cast<float>(v_enu[1]), static_cast<float>(v_enu[0]), static_cast<float>(-v_enu[2])};
        truth.q = {static_cast<float>(q.w), static_cast<float>(q.x), static_cast<float>(q.y), static_cast<float>(q.z)};
        truth.w_frd = flu_to_frd(odom->twist().angular(), 1.0);
    }
    // A baro/mag/GNSS message stamped off every IMU step of the block would otherwise be dropped silently.
    const std::size_t slow = baro_.size() + mag_.size() + gnss_.size();
    if (placed != slow) {
        err = std::to_string(slow - placed) + " baro/mag/gnss message(s) of the block ending t_us=" +
              std::to_string(t_us_) + " match no IMU step";
        return false;
    }
    return true;
}

void GzWorld::command(const ActuatorCommand& cmd) {
    gz::msgs::Actuators m;
    for (float u : cmd.motor) {
        const float c = !(u > 0.0f) ? 0.0f : (u > 1.0f ? 1.0f : u);  // NaN -> 0
        m.add_velocity(cmd.armed ? c * max_rot_velocity_ : 0.0);
    }
    motor_pub_.Publish(m);
}

void GzWorld::on_clock(const gz::msgs::Clock& m) {
    std::lock_guard<std::mutex> lk(mu_);
    ++clocks_;
    const std::uint64_t t = to_us(m.sim());
    if (t != 0 && t == imu_us_) settled_us_ = t;  // IMU(t) came first: this clock is a later iteration
    cv_.notify_all();
}

void GzWorld::on_imu(const gz::msgs::IMU& m) {
    std::lock_guard<std::mutex> lk(mu_);
    imu_.push_back(m);
    imu_us_ = to_us(m.header().stamp());
}

void GzWorld::on_odom(const gz::msgs::Odometry& m) {
    std::lock_guard<std::mutex> lk(mu_);
    odom_.push_back(m);
}

void GzWorld::on_baro(const gz::msgs::FluidPressure& m) {
    std::lock_guard<std::mutex> lk(mu_);
    baro_.push_back(m);
}

void GzWorld::on_mag(const gz::msgs::Magnetometer& m) {
    std::lock_guard<std::mutex> lk(mu_);
    mag_.push_back(m);
}

void GzWorld::on_gnss(const gz::msgs::NavSat& m) {
    std::lock_guard<std::mutex> lk(mu_);
    gnss_.push_back(m);
}

}  // namespace marv::bridge
