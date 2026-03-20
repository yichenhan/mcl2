#pragma once

#include "distance_localization.hpp"
#include "sim/physics.hpp"

#include <vector>

namespace pursuit {

struct PurePursuitConfig {
    double lookahead_distance = 12.0;
    double linear_velocity = 30.0;
    double waypoint_tolerance = 3.0;
    double max_angular_velocity_deg = 240.0;
};

struct Command {
    double linear_vel = 0.0;
    double angular_vel_deg = 0.0;
};

class PurePursuit {
public:
    explicit PurePursuit(const PurePursuitConfig& config = {});

    Command compute(const sim::RobotState& state, const std::vector<distance_loc::Vec2>& waypoints);
    bool is_complete() const;
    int current_waypoint_index() const;
    void reset();

private:
    struct PathPoint {
        double x = 0.0;
        double y = 0.0;
        int seg_idx = 0;
        double seg_t = 0.0;
    };

    static double normalize_signed_deg(double deg);
    static double distance(double x0, double y0, double x1, double y1);
    static double clamp(double v, double lo, double hi);
    static PathPoint closest_point_on_segment(
        double px, double py,
        const distance_loc::Vec2& a,
        const distance_loc::Vec2& b,
        int seg_idx);
    static bool segment_circle_first_intersection(
        double px, double py, double radius,
        const distance_loc::Vec2& a,
        const distance_loc::Vec2& b,
        double min_t,
        PathPoint& out,
        int seg_idx);

    PurePursuitConfig config_;
    int current_idx_ = 0;
    bool complete_ = false;
};

} // namespace pursuit

