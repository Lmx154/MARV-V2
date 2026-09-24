// Ported from PX4@af2e7b43 src/lib/motion_planning/VelocitySmoothing.cpp, PositionSmoothing.cpp,
// TrajectoryConstraints.hpp and src/lib/mathlib/math/TrajMath.hpp; the notice and the conditions are in smoothing.hpp.
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
#include <marv/fsw/smoothing.hpp>

#include <cfloat>
#include <cmath>

#include <marv/fsw/math.hpp>

namespace marv {
namespace {

constexpr float kPiF = 3.14159265f;  // M_PI_F, platforms/common/include/px4_platform_common/defines.h:124

// mathlib/math/Limits.hpp min, max, constrain (NaN handling as there).
float min(float a, float b) { return (a < b) ? a : b; }
float max(float a, float b) { return (a > b) ? a : b; }
float constrain(float val, float min_val, float max_val) {
    return (val < min_val) ? min_val : ((val > max_val) ? max_val : val);
}
// matrix/helper_functions.hpp:147-150 sign.
int sign(float val) { return (0.f < val) - (val < 0.f); }

// The horizontal part (z zero): PX4's Vector2f of a Vector3f.
Vec3 xy(Vec3 v) { return {v.x, v.y, 0.f}; }
// matrix/Vector.hpp:132-141 unit_or_zero, eps 1e-5.
Vec3 unit_or_zero(Vec3 v) {
    const float n = norm(v);
    return n > 1e-5f ? (1.f / n) * v : Vec3{0.f, 0.f, 0.f};
}
// matrix/Vector.hpp:148-151 longerThan.
bool longer_than(Vec3 v, float test_val) { return dot(v, v) > test_val * test_val; }
float& at(Vec3& v, int i) { return i == 0 ? v.x : (i == 1 ? v.y : v.z); }

}  // namespace

namespace traj {

// TrajMath.hpp:61-71.
float max_speed_from_distance(float jerk, float accel, float braking_distance, float final_speed) {
    const auto sqr = [](float f) { return f * f; };
    const float b = 4.0f * sqr(accel) / jerk;
    const float c = -2.0f * accel * braking_distance - sqr(final_speed);
    const float max_speed = 0.5f * (-b + std::sqrt(sqr(b) - 4.0f * c));
    return std::fmax(max_speed, final_speed);
}

// TrajMath.hpp:87-93.
float max_speed_in_waypoint(float alpha, float accel, float d) {
    const float tan_alpha = std::tan(alpha / 2.0f);
    return std::sqrt(accel * d * tan_alpha);
}

// TrajectoryConstraints.hpp:70-95.
float start_xy_speed_from_waypoints(Vec3 start_position, Vec3 target, Vec3 next_target, float exit_speed,
                                    const VehicleDynamicLimits& config) {
    const float distance_target_next = norm(xy(target - next_target));
    const bool target_next_different = distance_target_next > 0.001f;
    const bool waypoint_overlap = distance_target_next < config.xy_accept_rad;
    float speed_at_target = 0.0f;
    if (target_next_different && !waypoint_overlap) {
        const float alpha = std::acos(dot(unit_or_zero(xy(target - start_position)), unit_or_zero(xy(target - next_target))));
        const float safe_alpha = constrain(alpha, 0.f, kPiF - FLT_EPSILON);
        const float accel_tmp = config.max_acc_xy_radius_scale * config.max_acc_xy;
        const float max_speed_in_turn = max_speed_in_waypoint(safe_alpha, accel_tmp, config.xy_accept_rad);
        speed_at_target = min(min(max_speed_in_turn, exit_speed), config.max_speed_xy);
    }
    const float start_to_target = norm(xy(start_position - target));
    const float max_speed = max_speed_from_distance(config.max_jerk, config.max_acc_xy, start_to_target, speed_at_target);
    return min(config.max_speed_xy, max_speed);
}

// TrajectoryConstraints.hpp:107-123 with N = 3: backwards through the waypoints.
float xy_speed_from_waypoints(const Vec3 (&waypoints)[3], const VehicleDynamicLimits& config) {
    constexpr int kN = 3;
    float max_speed = 0.f;
    for (int i = kN - 2; i >= 0; i--) {
        const int next = i + 2 < kN - 1 ? i + 2 : kN - 1;
        max_speed = start_xy_speed_from_waypoints(waypoints[i], waypoints[i + 1], waypoints[next], max_speed, config);
    }
    return max_speed;
}

}  // namespace traj

// ---- VelocitySmoothing ------------------------------------------------------------------------------------------------

// VelocitySmoothing.cpp:43-46.
VelocitySmoothing::VelocitySmoothing(float initial_accel, float initial_vel, float initial_pos) {
    reset(initial_accel, initial_vel, initial_pos);
}

// VelocitySmoothing.cpp:48-56.
void VelocitySmoothing::reset(float accel, float vel, float pos) {
    state_.j = 0.f;
    state_.a = accel;
    state_.v = vel;
    state_.x = pos;
    state_init_ = state_;
}

// VelocitySmoothing.cpp:58-72.
float VelocitySmoothing::saturate_t1_for_accel(float a0, float j_max, float t1, float a_max) const {
    const float accel_t1 = a0 + j_max * t1;
    float t1_new = t1;
    if (accel_t1 > a_max) t1_new = (a_max - a0) / j_max;
    else if (accel_t1 < -a_max) t1_new = (-a_max - a0) / j_max;
    return t1_new;
}

// VelocitySmoothing.cpp:74-102.
float VelocitySmoothing::compute_t1(float a0, float v3, float j_max, float a_max) const {
    const float delta = 2.f * a0 * a0 + 4.f * j_max * v3;
    if (delta < 0.f) return 0.f;  // not real
    const float sqrt_delta = std::sqrt(delta);
    const float t1_plus = (-a0 + 0.5f * sqrt_delta) / j_max;
    const float t1_minus = (-a0 - 0.5f * sqrt_delta) / j_max;
    const float t3_plus = a0 / j_max + t1_plus;
    const float t3_minus = a0 / j_max + t1_minus;
    float t1 = 0.f;
    if (t1_plus >= 0.f && t3_plus >= 0.f) t1 = t1_plus;
    else if (t1_minus >= 0.f && t3_minus >= 0.f) t1 = t1_minus;
    t1 = saturate_t1_for_accel(a0, j_max, t1, a_max);
    return max(t1, 0.f);
}

// VelocitySmoothing.cpp:104-135.
float VelocitySmoothing::compute_t1(float t123, float a0, float v3, float j_max, float a_max) const {
    const float a = -j_max;
    const float b = j_max * t123 - a0;
    const float delta = t123 * t123 * j_max * j_max + 2.f * t123 * a0 * j_max - a0 * a0 - 4.f * j_max * v3;
    if (delta < 0.f) return 0.f;  // not real
    const float sqrt_delta = std::sqrt(delta);
    const float denominator_inv = 1.f / (2.f * a);
    const float t1_plus = max((-b + sqrt_delta) * denominator_inv, 0.f);
    const float t1_minus = max((-b - sqrt_delta) * denominator_inv, 0.f);
    const float t3_plus = a0 / j_max + t1_plus;
    const float t3_minus = a0 / j_max + t1_minus;
    float t1 = 0.f;
    if ((t1_plus >= 0.f && t3_plus >= 0.f) && ((t1_plus + t3_plus) <= t123)) t1 = t1_plus;
    else if ((t1_minus >= 0.f && t3_minus >= 0.f) && ((t1_minus + t3_minus) <= t123)) t1 = t1_minus;
    return saturate_t1_for_accel(a0, j_max, t1, a_max);
}

// VelocitySmoothing.cpp:138-149.
float VelocitySmoothing::compute_t2(float t1, float t3, float a0, float v3, float j_max) const {
    float t2 = 0.f;
    const float den = a0 + j_max * t1;
    if (std::fabs(den) > FLT_EPSILON)
        t2 = (-0.5f * t1 * t1 * j_max - t1 * t3 * j_max - t1 * a0 + 0.5f * t3 * t3 * j_max - t3 * a0 + v3) / den;
    return max(t2, 0.f);
}

// VelocitySmoothing.cpp:151-155.
float VelocitySmoothing::compute_t2(float t123, float t1, float t3) const { return max(t123 - t1 - t3, 0.f); }

// VelocitySmoothing.cpp:157-161.
float VelocitySmoothing::compute_t3(float t1, float a0, float j_max) const { return max(a0 / j_max + t1, 0.f); }

// VelocitySmoothing.cpp:163-172.
void VelocitySmoothing::update_durations(float vel_setpoint) {
    vel_sp_ = constrain(vel_setpoint, -max_vel_, max_vel_);
    local_time_ = 0.f;
    state_init_ = state_;
    direction_ = compute_direction();
    update_durations_minimize_total_time();
}

// VelocitySmoothing.cpp:174-190: the jerk's sign toward the setpoint from the velocity once the acceleration is braked
// to zero; exactly there, the acceleration's.
int VelocitySmoothing::compute_direction() const {
    const float vel_zero_acc = compute_vel_at_zero_acc();
    int direction = sign(vel_sp_ - vel_zero_acc);
    if (direction == 0) direction = sign(state_.a);
    return direction;
}

// VelocitySmoothing.cpp:192-203.
float VelocitySmoothing::compute_vel_at_zero_acc() const {
    float vel_zero_acc = state_.v;
    if (std::fabs(state_.a) > FLT_EPSILON) {
        const float j_zero_acc = -static_cast<float>(sign(state_.a)) * max_jerk_;
        const float t_zero_acc = -state_.a / j_zero_acc;
        vel_zero_acc = state_.v + state_.a * t_zero_acc + 0.5f * j_zero_acc * t_zero_acc * t_zero_acc;
    }
    return vel_zero_acc;
}

// VelocitySmoothing.cpp:205-223.
void VelocitySmoothing::update_durations_minimize_total_time() {
    const float jerk_max_t1 = static_cast<float>(direction_) * max_jerk_;
    const float delta_v = vel_sp_ - state_.v;
    if (std::fabs(jerk_max_t1) > FLT_EPSILON) {  // zero direction or jerk: no division by zero
        t1_ = compute_t1(state_.a, delta_v, jerk_max_t1, max_accel_);
        t3_ = compute_t3(t1_, state_.a, jerk_max_t1);
        t2_ = compute_t2(t1_, t3_, state_.a, delta_v, jerk_max_t1);
    } else {
        t1_ = t2_ = t3_ = 0.f;
    }
}

// VelocitySmoothing.cpp:225-238.
VelocitySmoothing::Trajectory VelocitySmoothing::evaluate_poly(float j, float a0, float v0, float x0, float t, int d) const {
    Trajectory traj;
    const float jt = static_cast<float>(d) * j;
    const float t2 = t * t;
    const float t3 = t2 * t;
    traj.j = jt;
    traj.a = a0 + jt * t;
    traj.v = v0 + a0 * t + 0.5f * jt * t2;
    traj.x = x0 + v0 * t + 0.5f * a0 * t2 + 1.f / 6.f * jt * t3;
    return traj;
}

// VelocitySmoothing.cpp:240-264.
void VelocitySmoothing::update_traj(float dt, float time_stretch) {
    local_time_ += dt * time_stretch;
    float t_remain = local_time_;

    const float t1 = min(t_remain, t1_);
    state_ = evaluate_poly(max_jerk_, state_init_.a, state_init_.v, state_init_.x, t1, direction_);
    t_remain -= t1;

    if (t_remain > 0.f) {
        const float t2 = min(t_remain, t2_);
        state_ = evaluate_poly(0.f, state_.a, state_.v, state_.x, t2, 0);
        t_remain -= t2;
    }
    if (t_remain > 0.f) {
        const float t3 = min(t_remain, t3_);
        state_ = evaluate_poly(max_jerk_, state_.a, state_.v, state_.x, t3, -direction_);
        t_remain -= t3;
    }
    if (t_remain > 0.f) state_ = evaluate_poly(0.f, 0.f, state_.v, state_.x, t_remain, 0);
}

// VelocitySmoothing.cpp:266-288.
void VelocitySmoothing::time_synchronization(VelocitySmoothing* traj, int n_traj) {
    float desired_time = 0.f;
    int longest_traj_index = 0;
    for (int i = 0; i < n_traj; i++) {
        const float t123 = traj[i].total_time();
        if (t123 > desired_time) {
            desired_time = t123;
            longest_traj_index = i;
        }
    }
    if (desired_time > FLT_EPSILON) {
        for (int i = 0; i < n_traj; i++)
            if ((i != longest_traj_index) && (traj[i].total_time() < desired_time))
                traj[i].update_durations_given_total_time(desired_time);
    }
}

// VelocitySmoothing.cpp:290-308.
void VelocitySmoothing::update_durations_given_total_time(float t123) {
    const float jerk_max_t1 = static_cast<float>(direction_) * max_jerk_;
    const float delta_v = vel_sp_ - state_.v;
    if (std::fabs(jerk_max_t1) > FLT_EPSILON) {  // zero direction or jerk: no division by zero
        t1_ = compute_t1(t123, state_.a, delta_v, jerk_max_t1, max_accel_);
        t3_ = compute_t3(t1_, state_.a, jerk_max_t1);
        t2_ = compute_t2(t123, t1_, t3_);
    } else {
        t1_ = t2_ = t3_ = 0.f;
    }
}

// ---- PositionSmoothing ------------------------------------------------------------------------------------------------

// PositionSmoothing.hpp:80-91.
void PositionSmoothing::generate_setpoints(Vec3 position, const Vec3 (&waypoints)[3], Vec3 feedforward_velocity,
                                           float delta_time, bool force_zero_velocity_setpoint, Setpoints& out) {
    generate(position, waypoints, false, feedforward_velocity, delta_time, force_zero_velocity_setpoint, out);
}

// PositionSmoothing.hpp:104-116.
void PositionSmoothing::generate_setpoints(Vec3 position, Vec3 waypoint, Vec3 feedforward_velocity, float delta_time,
                                           bool force_zero_velocity_setpoint, Setpoints& out) {
    const Vec3 waypoints[3] = {waypoint, waypoint, waypoint};
    generate(position, waypoints, true, feedforward_velocity, delta_time, force_zero_velocity_setpoint, out);
}

// PositionSmoothing.hpp:126-131.
void PositionSmoothing::reset(Vec3 acceleration, Vec3 velocity, Vec3 position) {
    for (int i = 0; i < 3; i++) trajectory_[i].reset(at(acceleration, i), at(velocity, i), at(position, i));
}

// PositionSmoothing.hpp:265-270.
void PositionSmoothing::set_max_jerk(float jerk) {
    for (VelocitySmoothing& t : trajectory_) t.set_max_jerk(jerk);
}

// PositionSmoothing.hpp:292-297.
void PositionSmoothing::set_max_acceleration(Vec3 accel) {
    for (int i = 0; i < 3; i++) trajectory_[i].set_max_accel(at(accel, i));
}

// PositionSmoothing.hpp:319-324.
void PositionSmoothing::set_max_velocity(Vec3 vel) {
    for (int i = 0; i < 3; i++) trajectory_[i].set_max_vel(at(vel, i));
}

// PositionSmoothing.hpp:240-243 getCurrentPosition.
Vec3 PositionSmoothing::current_position() const {
    return {trajectory_[0].current_position(), trajectory_[1].current_position(), trajectory_[2].current_position()};
}

// PositionSmoothing.cpp:39-62.
void PositionSmoothing::generate(Vec3 position, const Vec3 (&waypoints)[3], bool is_single_waypoint,
                                 Vec3 feedforward_velocity, float delta_time, bool force_zero_velocity_setpoint,
                                 Setpoints& out) {
    Vec3 velocity_setpoint{0.f, 0.f, 0.f};
    if (!force_zero_velocity_setpoint)
        velocity_setpoint = generate_velocity_setpoint(position, waypoints, is_single_waypoint, feedforward_velocity);
    out.unsmoothed_velocity = velocity_setpoint;
    generate_trajectory(position, velocity_setpoint, delta_time, out);
}

// PositionSmoothing.cpp:65-83: turning while the trajectory moves faster than 0.2 m/s more than about 10 deg off the
// direction to the target and is still outside the acceptance radius.
bool PositionSmoothing::is_turning(Vec3 target) const {
    const Vec3 vel_traj{trajectory_[0].current_velocity(), trajectory_[1].current_velocity(), 0.f};
    const Vec3 pos_traj{trajectory_[0].current_position(), trajectory_[1].current_position(), 0.f};
    const Vec3 u_vel_traj = unit_or_zero(vel_traj);
    const Vec3 pos_to_target = xy(target) - pos_traj;
    const float cos_align = dot(u_vel_traj, unit_or_zero(pos_to_target));
    return longer_than(vel_traj, 0.2f) && cos_align < 0.98f && longer_than(pos_to_target, target_acceptance_radius_);
}

// PositionSmoothing.cpp:85-105.
float PositionSmoothing::max_xy_speed(const Vec3 (&waypoints)[3]) const {
    traj::VehicleDynamicLimits config;
    config.z_accept_rad = vertical_acceptance_radius_;
    config.xy_accept_rad = target_acceptance_radius_;
    config.max_acc_xy = trajectory_[0].max_accel();
    config.max_jerk = trajectory_[0].max_jerk();
    config.max_speed_xy = cruise_speed_;
    config.max_acc_xy_radius_scale = horizontal_trajectory_gain_;
    const Vec3 pos_to_waypoints[3] = {current_position(), waypoints[1], waypoints[2]};
    return traj::xy_speed_from_waypoints(pos_to_waypoints, config);
}

// PositionSmoothing.cpp:107-140 (the (horizontal distance from the origin, down) plane as there).
float PositionSmoothing::max_z_speed(const Vec3 (&waypoints)[3]) const {
    const Vec3 start_position = current_position();
    const Vec3& target = waypoints[1];
    const Vec3& next_target = waypoints[2];
    const Vec3 start_position_xy_z{norm(xy(start_position)), start_position.z, 0.f};
    const Vec3 target_xy_z{norm(xy(target)), target.z, 0.f};
    const Vec3 next_target_xy_z{norm(xy(next_target)), next_target.z, 0.f};

    float arrival_z_speed = 0.0f;
    const bool target_next_different = std::fabs(target.z - next_target.z) > 0.001f;
    if (target_next_different) {
        const float alpha =
            std::acos(dot(unit_or_zero(target_xy_z - start_position_xy_z), unit_or_zero(target_xy_z - next_target_xy_z)));
        const float safe_alpha = constrain(alpha, 0.f, kPiF - FLT_EPSILON);
        const float accel_tmp = trajectory_[2].max_accel();
        const float max_speed_in_turn = traj::max_speed_in_waypoint(safe_alpha, accel_tmp, vertical_acceptance_radius_);
        arrival_z_speed = min(max_speed_in_turn, trajectory_[2].max_vel());
    }
    const float distance_start_target = std::fabs(target.z - start_position.z);
    return min(trajectory_[2].max_vel(),
               traj::max_speed_from_distance(trajectory_[2].max_jerk(), trajectory_[2].max_accel(), distance_start_target,
                                             arrival_z_speed));
}

// PositionSmoothing.cpp:142-152.
Vec3 PositionSmoothing::crossing_point(Vec3 position, const Vec3 (&waypoints)[3]) const {
    const Vec3& target = waypoints[1];
    if (!is_turning(target)) return target;
    return l1_point(position, waypoints);  // L1-style guidance
}

// PositionSmoothing.cpp:154-176: where a circle of radius max(acceptance radius, 5 m) about the trajectory meets the line
// from the previous waypoint to the target.
Vec3 PositionSmoothing::l1_point(Vec3, const Vec3 (&waypoints)[3]) const {
    const Vec3 pos_traj = current_position();
    const Vec3 u_prev_to_target = unit_or_zero(waypoints[1] - waypoints[0]);
    const Vec3 prev_to_pos = pos_traj - waypoints[0];
    const Vec3 prev_to_closest = dot(prev_to_pos, u_prev_to_target) * u_prev_to_target;
    const Vec3 closest_pt = waypoints[0] + prev_to_closest;
    const float crosstrack_error = norm(closest_pt - pos_traj);
    const float l1 = max(target_acceptance_radius_, 5.f);
    float alongtrack_error = 0.f;
    if (l1 > crosstrack_error) alongtrack_error = std::sqrt(l1 * l1 - crosstrack_error * crosstrack_error);
    return closest_pt + alongtrack_error * u_prev_to_target;
}

// PositionSmoothing.cpp:178-232, the 3-D position branch: toward the crossing point at the horizontal and vertical speeds
// the waypoints allow (the horizontal one held through a turn), plus the feed-forward velocity.
Vec3 PositionSmoothing::generate_velocity_setpoint(Vec3 position, const Vec3 (&waypoints)[3], bool is_single_waypoint,
                                                   Vec3 feedforward_velocity_setpoint) {
    const Vec3& target = waypoints[1];
    Vec3 velocity_setpoint = feedforward_velocity_setpoint;

    const Vec3 pos_traj = current_position();
    const Vec3 crossing = is_single_waypoint ? target : crossing_point(position, waypoints);

    float xy_speed = max_xy_speed(waypoints);
    const float z_speed = max_z_speed(waypoints);
    if (!is_single_waypoint && is_turning(target)) xy_speed = min(max_speed_previous_, xy_speed);  // limit through a turn
    else max_speed_previous_ = xy_speed;

    // XY and Z independently, so neither clamps the other.
    const Vec3 pos_to_dest = crossing - pos_traj;
    const Vec3 u_pos_to_dest_xy = unit_or_zero(xy(pos_to_dest));
    const float z_sign = static_cast<float>(sign(pos_to_dest.z));
    velocity_setpoint += Vec3{u_pos_to_dest_xy.x * xy_speed, u_pos_to_dest_xy.y * xy_speed, z_sign * z_speed};
    return velocity_setpoint;
}

// PositionSmoothing.cpp:285-350: integrate, then plan toward the new velocity setpoint. The horizontal and the vertical
// integration each slow to a stop as the vehicle falls behind the trajectory by the allowed error, only while the
// trajectory moves away from it.
void PositionSmoothing::generate_trajectory(Vec3 position, Vec3 velocity_setpoint, float delta_time, Setpoints& out) {
    if (!(std::isfinite(velocity_setpoint.x) && std::isfinite(velocity_setpoint.y) && std::isfinite(velocity_setpoint.z)))
        return;

    const Vec3 position_trajectory_xy{trajectory_[0].current_position(), trajectory_[1].current_position(), 0.f};
    const Vec3 vel_traj_xy{trajectory_[0].current_velocity(), trajectory_[1].current_velocity(), 0.f};
    const Vec3 drone_to_trajectory_xy = position_trajectory_xy - xy(position);
    const float position_error_xy = norm(drone_to_trajectory_xy);

    float time_stretch_xy = 1.f;
    if ((max_allowed_horizontal_error_ > FLT_EPSILON) && dot(drone_to_trajectory_xy, vel_traj_xy) >= 0)
        time_stretch_xy = 1.f - constrain(position_error_xy / max_allowed_horizontal_error_, 0.f, 1.f);
    if (dot(drone_to_trajectory_xy, vel_traj_xy) < 0.f) time_stretch_xy = 1.f;

    float time_stretch_z = 1.f;
    const float drone_to_traj_z = trajectory_[2].current_position() - position.z;
    const float vel_traj_z = trajectory_[2].current_velocity();
    if ((max_allowed_vertical_error_ > FLT_EPSILON) && drone_to_traj_z * vel_traj_z >= 0.f)
        time_stretch_z = 1.f - constrain(std::fabs(drone_to_traj_z) / max_allowed_vertical_error_, 0.f, 1.f);

    const float axis_stretch[3] = {time_stretch_xy, time_stretch_xy, time_stretch_z};
    for (int i = 0; i < 3; ++i) {
        trajectory_[i].update_traj(delta_time, axis_stretch[i]);
        at(out.jerk, i) = trajectory_[i].current_jerk();
        at(out.acceleration, i) = trajectory_[i].current_acceleration();
        at(out.velocity, i) = trajectory_[i].current_velocity();
        at(out.position, i) = trajectory_[i].current_position();
    }
    for (int i = 0; i < 3; ++i) trajectory_[i].update_durations(at(velocity_setpoint, i));
    VelocitySmoothing::time_synchronization(trajectory_, 3);
}

}  // namespace marv
