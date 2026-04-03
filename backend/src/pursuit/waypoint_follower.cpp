#include "pursuit/waypoint_follower.hpp"

#include <algorithm>
#include <cmath>

namespace pursuit {

namespace {
constexpr double kPi = 3.14159265358979323846;

double deg2rad(double d) { return d * kPi / 180.0; }
double rad2deg(double r) { return r * 180.0 / kPi; }
} // namespace

WaypointFollower::WaypointFollower(const FollowerConfig& config)
    : config_(config) {}

double WaypointFollower::wrap_angle(double deg) {
    deg = std::fmod(deg, 360.0);
    if (deg > 180.0)  deg -= 360.0;
    if (deg <= -180.0) deg += 360.0;
    return deg;
}

double WaypointFollower::euclidean(double x0, double y0, double x1, double y1) {
    const double dx = x0 - x1;
    const double dy = y0 - y1;
    return std::sqrt(dx * dx + dy * dy);
}

FollowerResult WaypointFollower::compute(const sim::RobotState& pose,
                                          const std::vector<Waypoint>& waypoints,
                                          int current_index) const {
    FollowerResult result;
    result.current_index = current_index;

    if (waypoints.empty() || current_index < 0 ||
        current_index >= static_cast<int>(waypoints.size())) {
        result.all_done = true;
        return result;
    }

    const Waypoint& target = waypoints[static_cast<size_t>(current_index)];
    const double dist = euclidean(pose.x, pose.y, target.x, target.y);

    // Check if waypoint reached
    if (dist <= config_.waypoint_tolerance) {
        result.waypoint_reached = true;
        const int next = current_index + 1;
        if (next >= static_cast<int>(waypoints.size())) {
            result.all_done = true;
        }
        result.current_index = current_index;
        return result;
    }

    // Heading from robot to target (degrees, field frame)
    const double dx = target.x - pose.x;
    const double dy = target.y - pose.y;
    // Field frame: +y = forward at heading 0, +x = right
    // Heading measured clockwise from +y:
    const double target_heading = rad2deg(std::atan2(dx, dy));
    const double heading_error = wrap_angle(target_heading - pose.heading_deg);

    // Angular velocity: proportional to heading error, capped
    double angular_vel = heading_error * config_.heading_gain;
    angular_vel = std::clamp(angular_vel,
                              -config_.max_angular_vel_deg,
                               config_.max_angular_vel_deg);

    // Linear velocity: slow down when close OR when heading is very wrong
    const double heading_factor = std::max(0.0, std::cos(deg2rad(heading_error)));
    const double dist_factor = std::min(1.0, dist / config_.slowdown_radius);
    double linear_vel = config_.linear_velocity * heading_factor * dist_factor;
    linear_vel = std::max(0.0, linear_vel);

    result.linear_velocity = linear_vel;
    result.angular_vel_deg = angular_vel;
    return result;
}

const FollowerConfig& WaypointFollower::config() const {
    return config_;
}

void WaypointFollower::set_config(const FollowerConfig& cfg) {
    config_ = cfg;
}

} // namespace pursuit
