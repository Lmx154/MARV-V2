// Jerk-limited trajectory smoothing, ported from PX4@af2e7b43:
//   VelocitySmoothing   src/lib/motion_planning/VelocitySmoothing.{hpp,cpp}: one axis driven toward a velocity
//                       setpoint in three phases (jerk, zero jerk, opposite jerk) within jerk, acceleration and velocity
//                       limits, integrated in closed form
//   PositionSmoothing   src/lib/motion_planning/PositionSmoothing.{hpp,cpp}: three such axes toward a waypoint or along
//                       a triplet (previous, target, next), the speed from src/lib/motion_planning/TrajectoryConstraints.hpp
//                       and src/lib/mathlib/math/TrajMath.hpp:61-92, the integration slowed while the vehicle lags
// float only, fixed arrays, no heap, no virtual dispatch. Not ported: PX4's NaN convention for an axis without a target
// (here every target is a full 3-D position and the feed-forward velocity is always finite), forceSet*, and the per-axis
// getters and setters nothing here calls.
//
// Copyright (c) 2018-2021 PX4 Development Team. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
// following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
//    disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
//    following disclaimer in the documentation and/or other materials provided with the distribution.
// 3. Neither the name PX4 nor the names of its contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
// INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#pragma once

#include <marv/fsw/contracts.hpp>

namespace marv {

namespace traj {

// TrajectoryConstraints.hpp:48-59 VehicleDynamicLimits.
struct VehicleDynamicLimits {
    float z_accept_rad;
    float xy_accept_rad;
    float max_acc_xy;
    float max_jerk;
    float max_speed_xy;
    float max_acc_xy_radius_scale;
};

// TrajMath.hpp:61-71 computeMaxSpeedFromDistance: the most speed from which a braking at accel, reached after
// 2 accel / jerk, still arrives at final_speed within braking_distance; never below final_speed.
float max_speed_from_distance(float jerk, float accel, float braking_distance, float final_speed);
// TrajMath.hpp:87-93 computeMaxSpeedInWaypoint: the speed on a circle tangent to two segments of length d that meet at
// the angle alpha, at the lateral acceleration accel: sqrt(accel d tan(alpha / 2)).
float max_speed_in_waypoint(float alpha, float accel, float d);
// TrajectoryConstraints.hpp:70-95 computeStartXYSpeedFromWaypoints.
float start_xy_speed_from_waypoints(Vec3 start_position, Vec3 target, Vec3 next_target, float exit_speed,
                                    const VehicleDynamicLimits& config);
// TrajectoryConstraints.hpp:107-123 computeXYSpeedFromWaypoints<3>: the most horizontal speed at waypoints[0] from
// which the rest can be followed within the limits.
float xy_speed_from_waypoints(const Vec3 (&waypoints)[3], const VehicleDynamicLimits& config);

}  // namespace traj

// VelocitySmoothing.hpp:61-213.
class VelocitySmoothing {
public:
    struct Trajectory {
        float j;  // jerk
        float a;  // acceleration
        float v;  // velocity
        float x;  // position
    };

    explicit VelocitySmoothing(float initial_accel = 0.f, float initial_vel = 0.f, float initial_pos = 0.f);

    void reset(float accel, float vel, float pos);
    // T1, T2 and T3 from the state toward vel_setpoint; every cycle, before update_traj.
    void update_durations(float vel_setpoint);
    // The state dt * time_stretch further along the phases.
    void update_traj(float dt, float time_stretch = 1.f);

    float max_jerk() const { return max_jerk_; }
    void set_max_jerk(float max_jerk) { max_jerk_ = max_jerk; }
    float max_accel() const { return max_accel_; }
    void set_max_accel(float max_accel) { max_accel_ = max_accel; }
    float max_vel() const { return max_vel_; }
    void set_max_vel(float max_vel) { max_vel_ = max_vel; }

    float current_jerk() const { return state_.j; }
    void set_current_acceleration(float accel) { state_.a = state_init_.a = accel; }
    float current_acceleration() const { return state_.a; }
    void set_current_velocity(float vel) { state_.v = state_init_.v = vel; }
    float current_velocity() const { return state_.v; }
    void set_current_position(float pos) { state_.x = state_init_.x = pos; }
    float current_position() const { return state_.x; }

    float t1() const { return t1_; }
    float t2() const { return t2_; }
    float t3() const { return t3_; }
    float total_time() const { return t1_ + t2_ + t3_; }

    // The n_traj trajectories take the total time of the longest (straight lines).
    static void time_synchronization(VelocitySmoothing* traj, int n_traj);

private:
    void update_durations_minimize_total_time();
    void update_durations_given_total_time(float t123);
    int compute_direction() const;
    float compute_vel_at_zero_acc() const;
    float compute_t1(float a0, float v3, float j_max, float a_max) const;
    float compute_t1(float t123, float a0, float v3, float j_max, float a_max) const;
    float saturate_t1_for_accel(float a0, float j_max, float t1, float a_max) const;
    float compute_t2(float t1, float t3, float a0, float v3, float j_max) const;
    float compute_t2(float t123, float t1, float t3) const;
    float compute_t3(float t1, float a0, float j_max) const;
    Trajectory evaluate_poly(float j, float a0, float v0, float x0, float t, int d) const;

    float vel_sp_ = 0.f;
    float max_jerk_ = 0.f;
    float max_accel_ = 0.f;
    float max_vel_ = 0.f;
    Trajectory state_{};
    int direction_ = 0;
    Trajectory state_init_{};
    float t1_ = 0.f;  // s, increasing acceleration
    float t2_ = 0.f;  // s, constant acceleration
    float t3_ = 0.f;  // s, decreasing acceleration
    float local_time_ = 0.f;
};

// PositionSmoothing.hpp:52-463.
class PositionSmoothing {
public:
    struct Setpoints {
        Vec3 jerk;
        Vec3 acceleration;
        Vec3 velocity;
        Vec3 position;
        Vec3 unsmoothed_velocity;
    };

    // PositionSmoothing.hpp:80-91: along the triplet waypoints (0 previous, 1 target, 2 the one after), from position.
    void generate_setpoints(Vec3 position, const Vec3 (&waypoints)[3], Vec3 feedforward_velocity, float delta_time,
                            bool force_zero_velocity_setpoint, Setpoints& out);
    // PositionSmoothing.hpp:104-116: to the one waypoint, ending at rest there.
    void generate_setpoints(Vec3 position, Vec3 waypoint, Vec3 feedforward_velocity, float delta_time,
                            bool force_zero_velocity_setpoint, Setpoints& out);
    // PositionSmoothing.hpp:126-131.
    void reset(Vec3 acceleration, Vec3 velocity, Vec3 position);

    // PositionSmoothing.hpp:265-270, 292-297, 319-324, 329-375.
    void set_max_jerk(float jerk);
    void set_max_acceleration(Vec3 accel);
    void set_max_velocity(Vec3 vel);
    void set_max_allowed_horizontal_error(float error) { max_allowed_horizontal_error_ = error; }
    void set_max_allowed_vertical_error(float error) { max_allowed_vertical_error_ = error; }
    void set_vertical_acceptance_radius(float radius) { vertical_acceptance_radius_ = radius; }
    void set_cruise_speed(float speed) { cruise_speed_ = speed; }
    void set_horizontal_trajectory_gain(float gain) { horizontal_trajectory_gain_ = gain; }
    void set_target_acceptance_radius(float radius) { target_acceptance_radius_ = radius; }

private:
    float max_allowed_horizontal_error_ = 0.f;
    float max_allowed_vertical_error_ = 0.f;
    float vertical_acceptance_radius_ = 0.f;
    float cruise_speed_ = 0.f;
    float horizontal_trajectory_gain_ = 0.f;
    float target_acceptance_radius_ = 0.f;

    VelocitySmoothing trajectory_[3];  // north, east, down
    float max_speed_previous_ = 0.f;

    Vec3 current_position() const;
    bool is_turning(Vec3 target) const;
    void generate(Vec3 position, const Vec3 (&waypoints)[3], bool is_single_waypoint, Vec3 feedforward_velocity,
                  float delta_time, bool force_zero_velocity_setpoint, Setpoints& out);
    Vec3 generate_velocity_setpoint(Vec3 position, const Vec3 (&waypoints)[3], bool is_single_waypoint,
                                    Vec3 feedforward_velocity_setpoint);
    Vec3 l1_point(Vec3 position, const Vec3 (&waypoints)[3]) const;
    Vec3 crossing_point(Vec3 position, const Vec3 (&waypoints)[3]) const;
    float max_xy_speed(const Vec3 (&waypoints)[3]) const;
    float max_z_speed(const Vec3 (&waypoints)[3]) const;
    void generate_trajectory(Vec3 position, Vec3 velocity_setpoint, float delta_time, Setpoints& out);
};

}  // namespace marv
