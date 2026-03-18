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

StepResult Physics::step(Action action) {
    StepResult result{};

    if (action == Action::NONE) {
        return result;
    }

    const double rot_step = config_.max_angular_vel * config_.dt;
    if (action == Action::ROTATE_CW || action == Action::ROTATE_CCW) {
        const double delta_rot = (action == Action::ROTATE_CW) ? rot_step : -rot_step;
        state_.heading_deg = wrap_heading(state_.heading_deg + delta_rot);
        result.delta.rotation_deg = delta_rot;
        return result;
    }

    const double transl_step = config_.max_velocity * config_.dt;
    const double requested_forward = (action == Action::FORWARD) ? transl_step : -transl_step;

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

const RobotState& Physics::state() const {
    return state_;
}

void Physics::set_state(const RobotState& s) {
    state_ = s;
    state_.heading_deg = wrap_heading(state_.heading_deg);
    field_.clamp(state_.x, state_.y);
}

} // namespace sim
