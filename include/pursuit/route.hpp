#pragma once

#include "distance_localization.hpp"
#include "noise/failure_injector.hpp"
#include "pursuit/waypoint_follower.hpp"
#include "sim/field.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace pursuit {

struct Range {
    double min = 0.0;
    double max = 0.0;
};

struct KidnapEvent {
    int tick = 0;
    bool random_target = true;
    double x = 0.0;
    double y = 0.0;
};

struct FailureGenConfig {
    double sensor_dead_prob = 0.0;
    double sensor_stuck_prob = 0.0;
    double odom_spike_prob = 0.0;
    double heading_bias_prob = 0.0;
    double spurious_reflection_prob = 0.0;
    double kidnap_prob = 0.0;
    int min_duration_ticks = 1;
    int max_duration_ticks = 1;
    Range odom_spike_range{1.5, 3.0};
    Range heading_bias_range{-10.0, 10.0};
    Range spurious_reflection_range{2.0, 12.0};
};

struct RouteDefinition {
    std::string name;
    std::string description;
    std::vector<distance_loc::Vec2> waypoints;
    std::vector<sim::AABB> obstacles;
    distance_loc::Vec2 initial_position{0.0, 0.0};
    double initial_heading_deg = 0.0;
    FollowerConfig follower{};
    uint64_t failure_seed = 42;
    FailureGenConfig failure_config{};
    std::vector<KidnapEvent> kidnap_events;
    // Gate estimated pose updates used by route control.
    // If the MCL estimate jumps farther than this speed budget per tick,
    // the controller keeps the previous estimate instead of accepting the jump.
    double max_estimate_speed_ft_per_s = 360.0 / 12.0;
    int max_ticks = 500;
};

RouteDefinition load_route(const std::string& path);
void validate_route(const RouteDefinition& route, double field_half = 72.0);
std::vector<noise::FailureEvent> generate_random_failures(
    const FailureGenConfig& config,
    int max_ticks,
    uint64_t seed,
    double field_half);
std::vector<noise::FailureEvent> build_failure_schedule(
    const RouteDefinition& route,
    double field_half = 72.0);

} // namespace pursuit

