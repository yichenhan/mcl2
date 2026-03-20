#include "sim/physics.hpp"

#include <cmath>

namespace sim {

namespace {
double wrap_heading(double deg) {
    deg = std::fmod(deg, 360.0);
    if (deg < 0.0) deg += 360.0;
    return deg;
}
} // namespace

Physics::Physics(const Field& field, const PhysicsConfig& config)
    : field_(field), config_(config), state_{} {}

StepResult Physics::step_continuous(double linear_vel, double angular_vel_deg) {
    StepResult result{};

    const double clamped_linear = std::max(-config_.max_velocity, std::min(config_.max_velocity, linear_vel));
    const double clamped_angular = std::max(-config_.max_angular_vel, std::min(config_.max_angular_vel, angular_vel_deg));

    const double delta_rot = clamped_angular * config_.dt;
    state_.heading_deg = wrap_heading(state_.heading_deg + delta_rot);
    result.delta.rotation_deg = delta_rot;

    const double requested_forward = clamped_linear * config_.dt;

    constexpr double kPi = 3.14159265358979323846;
    const double heading_rad = state_.heading_deg * kPi / 180.0;
    const double ux = std::sin(heading_rad);
    const double uy = std::cos(heading_rad);

    const double x0 = state_.x;
    const double y0 = state_.y;

    double x1 = x0 + requested_forward * ux;
    double y1 = y0 + requested_forward * uy;
    const double x1_unclamped = x1;
    const double y1_unclamped = y1;

    field_.clamp(x1, y1);
    const bool wall_collision =
        (std::fabs(x1 - x1_unclamped) > 1e-9 || std::fabs(y1 - y1_unclamped) > 1e-9);
    const bool obstacle_collision = field_.segment_hits_obstacle(x0, y0, x1, y1)
                                    || !field_.is_passable(x1, y1);

    if (obstacle_collision) {
        state_.x = x0;
        state_.y = y0;
    } else {
        state_.x = x1;
        state_.y = y1;
    }

    result.colliding = wall_collision || obstacle_collision;

    const double actual_dx = state_.x - x0;
    const double actual_dy = state_.y - y0;
    result.delta.forward_in = actual_dx * ux + actual_dy * uy;
    return result;
}

StepResult Physics::step(Action action) {
    StepResult result{};

    if (action == Action::NONE) {
        return result;
    }

    if (action == Action::ROTATE_CW) return step_continuous(0.0, config_.max_angular_vel);
    if (action == Action::ROTATE_CCW) return step_continuous(0.0, -config_.max_angular_vel);
    if (action == Action::FORWARD) return step_continuous(config_.max_velocity, 0.0);
    return step_continuous(-config_.max_velocity, 0.0);
}

const RobotState& Physics::state() const {
    return state_;
}

void Physics::set_state(const RobotState& s) {
    state_ = s;
    state_.heading_deg = wrap_heading(state_.heading_deg);
    field_.clamp(state_.x, state_.y);
}

} // namespace sim
