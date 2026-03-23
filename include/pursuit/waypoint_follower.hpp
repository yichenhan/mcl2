#pragma once

#include "distance_localization.hpp"
#include "sim/physics.hpp"

#include <vector>

namespace pursuit {

struct FollowerConfig {
    double linear_velocity = 30.0;
    double waypoint_tolerance = 3.0;
    double max_angular_velocity_deg = 240.0;
    // Heading error above this rotates in place before translating.
    double turn_in_place_threshold_deg = 45.0;
};

struct Command {
    double linear_vel = 0.0;
    double angular_vel_deg = 0.0;
};

class WaypointFollower {
public:
    explicit WaypointFollower(const FollowerConfig& config = {});

    Command moveToPoint(const sim::RobotState& state, const distance_loc::Vec2& target) const;
    Command turnToHeading(const sim::RobotState& state, double target_heading_deg) const;
    Command compute(const sim::RobotState& state, const std::vector<distance_loc::Vec2>& waypoints);
    bool is_complete() const;
    int current_waypoint_index() const;
    void reset();

private:
    static double normalize_signed_deg(double deg);
    static double clamp(double v, double lo, double hi);
    static double distance(double x0, double y0, double x1, double y1);
    static double heading_to_target_deg(double x0, double y0, double x1, double y1);
    static bool passed_waypoint(
        const sim::RobotState& state,
        const distance_loc::Vec2& prev,
        const distance_loc::Vec2& target,
        double tolerance);

    FollowerConfig config_;
    int current_idx_ = 0;
    bool complete_ = false;
};

} // namespace pursuit

