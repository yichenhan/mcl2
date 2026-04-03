#include "sim/sim_chassis.hpp"

#include "distance_localization.hpp"

#include <cmath>

namespace sim {

namespace {
constexpr double kPi = 3.14159265358979323846;
double deg2rad(double deg) { return deg * kPi / 180.0; }
} // namespace

SimChassis::SimChassis(const Field& field,
                       const PhysicsConfig& phys_config,
                       const RobotState& initial_state)
    : physics_(field, phys_config),
      field_(field),
      odom_state_(initial_state) {
    physics_.set_state(initial_state);
    // Ensure initial odom heading is wrapped
    odom_state_.heading_deg = wrap_heading(initial_state.heading_deg);
}

SimChassis::Pose SimChassis::getPose() const {
    return {
        static_cast<float>(odom_state_.x),
        static_cast<float>(odom_state_.y),
        static_cast<float>(odom_state_.heading_deg)
    };
}

void SimChassis::setPose(float x, float y, float theta) {
    odom_state_.x = static_cast<double>(x);
    odom_state_.y = static_cast<double>(y);
    odom_state_.heading_deg = wrap_heading(static_cast<double>(theta));
}

StepResult SimChassis::step(Action action) {
    return physics_.step(action);
}

StepResult SimChassis::step(double linear_vel, double angular_vel_deg) {
    return physics_.step_continuous(linear_vel, angular_vel_deg);
}

void SimChassis::applyOdomNoise(const MotionDelta& delta, double heading_deg) {
    // Integrate delta in the odom frame using the current odom heading.
    // heading_deg is written directly (not accumulated) -- the caller
    // provides the final heading value from IMU/noise observation.
    const double heading_rad = deg2rad(odom_state_.heading_deg);
    const double s = std::sin(heading_rad);
    const double c = std::cos(heading_rad);
    odom_state_.x += delta.forward_in * s + delta.lateral_in * c;
    odom_state_.y += delta.forward_in * c - delta.lateral_in * s;
    odom_state_.heading_deg = wrap_heading(heading_deg);
    field_.clamp(odom_state_.x, odom_state_.y);
}

void SimChassis::teleport(const RobotState& target) {
    physics_.set_state(target);
}

RobotState SimChassis::ground_truth() const {
    return physics_.state();
}

double SimChassis::wrap_heading(double deg) {
    if (!std::isfinite(deg)) return 0.0;
    deg = std::fmod(deg, 360.0);
    if (deg < 0.0) deg += 360.0;
    return deg;
}

} // namespace sim
