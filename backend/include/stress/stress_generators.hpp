#pragma once

#include "noise/failure_config.hpp"
#include "noise/failure_injector.hpp"
#include "sim/field.hpp"
#include "sim/physics.hpp"
#include "sim/sensor_model.hpp"

#include <cstdint>
#include <utility>
#include <vector>

namespace stress {

struct RouteResult {
    sim::RobotState start_pose{};
    std::vector<std::pair<double, double>> velocity_commands;
};

RouteResult generate_route(const sim::Field& field, int total_ticks, uint64_t seed);

std::vector<sim::Obstacle> generate_random_obstacles(uint64_t seed, const sim::Field& base_field, int count);

struct StressNoiseBundle {
    sim::OdomNoiseConfig odom{};
    sim::SensorNoiseConfig sensor{};
    pursuit::FailureGenConfig failure_gen{};
    std::vector<noise::FailureEvent> failure_events;
};

StressNoiseBundle generate_stress_noise(int profile_id, int max_ticks, uint64_t seed, double field_half);

void apply_failure_guardrails(std::vector<noise::FailureEvent>& events, int max_ticks);

} // namespace stress
