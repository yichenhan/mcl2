#pragma once

#include "sim/physics.hpp"

#include <vector>

namespace pursuit {

// ---------------------------------------------------------------------------
// Waypoint
// ---------------------------------------------------------------------------
struct Waypoint {
    double x = 0.0;
    double y = 0.0;
};

// ---------------------------------------------------------------------------
// FollowerConfig
// ---------------------------------------------------------------------------
struct FollowerConfig {
    double linear_velocity = 24.0;       // inches/sec -- maximum forward speed
    double waypoint_tolerance = 3.0;     // inches -- distance to declare waypoint reached
    double max_angular_vel_deg = 180.0;  // degrees/sec -- angular velocity cap
    double heading_gain = 2.5;           // proportional gain: heading_error_deg -> angular_vel
    double slowdown_radius = 12.0;       // inches -- begin slowing down at this distance
};

// ---------------------------------------------------------------------------
// FollowerResult
// ---------------------------------------------------------------------------
struct FollowerResult {
    double linear_velocity = 0.0;     // commanded linear velocity (in/s)
    double angular_vel_deg = 0.0;     // commanded angular velocity (deg/s)
    bool waypoint_reached = false;    // true if current waypoint is within tolerance
    bool all_done = false;            // true if all waypoints have been reached
    int current_index = 0;            // current waypoint index being targeted
};

// ---------------------------------------------------------------------------
// WaypointFollower
// ---------------------------------------------------------------------------
// Stateless proportional controller for sequential waypoint navigation.
// All state (current waypoint index) is held by the caller (NavigationState).
// Call compute() every tick with the current chassis pose and active waypoints.
// ---------------------------------------------------------------------------
class WaypointFollower {
public:
    explicit WaypointFollower(const FollowerConfig& config = {});

    // Compute a velocity command to move toward waypoints[current_index].
    // Returns FollowerResult with the velocity command and whether the waypoint
    // was reached this tick.
    FollowerResult compute(const sim::RobotState& pose,
                           const std::vector<Waypoint>& waypoints,
                           int current_index) const;

    const FollowerConfig& config() const;
    void set_config(const FollowerConfig& cfg);

private:
    static double wrap_angle(double deg);
    static double euclidean(double x0, double y0, double x1, double y1);

    FollowerConfig config_;
};

} // namespace pursuit
