#include "pursuit/waypoint_follower.hpp"

#include <algorithm>
#include <cmath>

namespace pursuit {

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kTurnGain = 4.0;
constexpr double kMoveTurnGain = 3.0;
constexpr double kMoveHeadingCutoffDeg = 20.0;
constexpr double kMaxTurnInPlaceThresholdDeg = 35.0;
} // namespace

WaypointFollower::WaypointFollower(const FollowerConfig& config)
    : config_(config) {}

double WaypointFollower::normalize_signed_deg(double deg) {
    while (deg > 180.0) deg -= 360.0;
    while (deg < -180.0) deg += 360.0;
    return deg;
}

double WaypointFollower::clamp(double v, double lo, double hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

double WaypointFollower::distance(double x0, double y0, double x1, double y1) {
    const double dx = x1 - x0;
    const double dy = y1 - y0;
    return std::sqrt(dx * dx + dy * dy);
}

double WaypointFollower::heading_to_target_deg(double x0, double y0, double x1, double y1) {
    // Heading convention matches physics: 0 deg = +Y and CW positive.
    return std::atan2(x1 - x0, y1 - y0) * 180.0 / kPi;
}

bool WaypointFollower::passed_waypoint(
    const sim::RobotState& state,
    const distance_loc::Vec2& prev,
    const distance_loc::Vec2& target,
    double tolerance) {
    const double sx = target.x - prev.x;
    const double sy = target.y - prev.y;
    const double seg_len2 = sx * sx + sy * sy;
    if (seg_len2 < 1e-9) return false;

    // Progress along the current segment: >1 means we've moved beyond target.
    const double px = state.x - prev.x;
    const double py = state.y - prev.y;
    const double t = (px * sx + py * sy) / seg_len2;
    if (t <= 1.0) return false;

    // Only count as passed if we're still reasonably close to the waypoint.
    // This prevents accepting unrelated waypoints when localization jumps.
    const double pass_radius = std::max(18.0, tolerance * 6.0);
    return distance(state.x, state.y, target.x, target.y) <= pass_radius;
}

Command WaypointFollower::turnToHeading(const sim::RobotState& state, double target_heading_deg) const {
    Command cmd{};
    const double heading_error = normalize_signed_deg(target_heading_deg - state.heading_deg);
    cmd.linear_vel = 0.0;
    cmd.angular_vel_deg = clamp(
        kTurnGain * heading_error,
        -config_.max_angular_velocity_deg,
        config_.max_angular_velocity_deg);
    return cmd;
}

Command WaypointFollower::moveToPoint(const sim::RobotState& state, const distance_loc::Vec2& target) const {
    Command cmd{};
    const double target_heading = heading_to_target_deg(state.x, state.y, target.x, target.y);
    const double heading_error_deg = normalize_signed_deg(target_heading - state.heading_deg);
    const double abs_heading_error_deg = std::fabs(heading_error_deg);
    // Do not push forward while still significantly misaligned; this avoids
    // plowing into walls on U-turn transitions.
    if (abs_heading_error_deg > kMoveHeadingCutoffDeg) {
        cmd.linear_vel = 0.0;
    } else {
        const double heading_error_rad = heading_error_deg * kPi / 180.0;
        const double heading_scale = std::max(0.0, std::cos(heading_error_rad));
        cmd.linear_vel = config_.linear_velocity * heading_scale;
    }
    cmd.angular_vel_deg = clamp(
        kMoveTurnGain * heading_error_deg,
        -config_.max_angular_velocity_deg,
        config_.max_angular_velocity_deg);
    return cmd;
}

Command WaypointFollower::compute(const sim::RobotState& state, const std::vector<distance_loc::Vec2>& waypoints) {
    Command cmd{};
    if (waypoints.empty()) {
        complete_ = true;
        return cmd;
    }
    if (complete_) return cmd;

    const int n = static_cast<int>(waypoints.size());
    if (current_idx_ < 0) current_idx_ = 0;

    while (current_idx_ < n) {
        const auto& target = waypoints[static_cast<size_t>(current_idx_)];
        const bool in_radius =
            distance(state.x, state.y, target.x, target.y) <= config_.waypoint_tolerance;
        bool passed = false;
        if (!in_radius && current_idx_ > 0) {
            const auto& prev = waypoints[static_cast<size_t>(current_idx_ - 1)];
            passed = passed_waypoint(state, prev, target, config_.waypoint_tolerance);
        }
        if (!in_radius && !passed) break;
        current_idx_++;
    }

    if (current_idx_ >= n) {
        complete_ = true;
        return cmd;
    }

    const auto& target = waypoints[static_cast<size_t>(current_idx_)];
    const double target_heading = heading_to_target_deg(state.x, state.y, target.x, target.y);
    const double heading_error = std::fabs(normalize_signed_deg(target_heading - state.heading_deg));
    const double turn_threshold = std::min(config_.turn_in_place_threshold_deg, kMaxTurnInPlaceThresholdDeg);
    if (heading_error >= turn_threshold) {
        return turnToHeading(state, target_heading);
    }
    return moveToPoint(state, target);
}

bool WaypointFollower::is_complete() const {
    return complete_;
}

int WaypointFollower::current_waypoint_index() const {
    return current_idx_;
}

void WaypointFollower::reset() {
    current_idx_ = 0;
    complete_ = false;
}

} // namespace pursuit

