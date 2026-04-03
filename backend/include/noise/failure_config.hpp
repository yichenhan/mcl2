#pragma once

#include "noise/failure_injector.hpp"

#include <cstdint>
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
    /// Minimum duration (ticks) for multi-tick failures that use `min_duration_ticks` / per-type max.
    int min_duration_ticks = 1;
    /// Legacy cap when a per-type `*_max_duration_ticks` is 0 (unset).
    int max_duration_ticks = 1;
    /// Per-failure max duration (ticks). 0 means fall back to `max_duration_ticks` (except spurious, see cpp).
    int sensor_dead_max_duration_ticks = 0;
    int sensor_stuck_max_duration_ticks = 0;
    int odom_spike_max_duration_ticks = 0;
    int heading_bias_max_duration_ticks = 0;
    /// 0 defaults to 1 tick (legacy behavior); set to N for 1..N tick reflections.
    int spurious_reflection_max_duration_ticks = 0;
    Range odom_spike_range{1.5, 3.0};
    Range heading_bias_range{-10.0, 10.0};
    Range spurious_reflection_range{2.0, 12.0};
};

std::vector<noise::FailureEvent> generate_random_failures(
    const FailureGenConfig& config,
    int max_ticks,
    uint64_t seed,
    double field_half);

} // namespace pursuit
