#pragma once

#include "noise/noise_generator.hpp"
#include "ray/ray_cast_obstacles.hpp"
#include "distance_localization.hpp"
#include "sim/physics.hpp"

#include <array>
#include <cstdint>
#include <vector>

namespace sim {

struct OdomNoiseConfig {
    double trans_noise_frac = 0.03;   // forward noise proportional to distance
    double drift_noise_frac = 0.01;   // lateral noise proportional to distance
    double drift_per_tick_in = 0.01;  // constant lateral drift bias per tick
};

struct SensorNoiseConfig {
    double gaussian_stddev_mm = 15.0;
    double dropout_probability = 0.02;
    double long_dropout_probability = 0.005;
    int long_dropout_min_ticks = 10;
    int long_dropout_max_ticks = 60;
    double range_noise_start_mm = 1700.0;
    double range_max_mm = 2000.0;
    double range_noise_slope = 0.05;
    double spurious_reflection_probability = 0.01;
    double imu_noise_stddev_deg = 0.1;
    int collision_stall_ticks = 5;
};

class SensorModel {
public:
    SensorModel(
        uint64_t seed,
        const OdomNoiseConfig& odom_config = {},
        const SensorNoiseConfig& sensor_config = {}
    );

    // Apply odom noise in robot-local frame.
    // forward_in: noisy forward component
    // lateral_in: noisy lateral component (right-positive)
    // rotation_deg: passthrough in Phase 4
    MotionDelta apply_odom_noise(const MotionDelta& true_delta, double heading_deg);

    // Per-tick IMU heading observation (no cumulative drift).
    double observe_heading(double true_heading_deg);

    // Full distance sensor pipeline for a single sensor.
    // Returns inches; invalid reading is -1.
    double observe_distance_in(
        double true_distance_in,
        int sensor_idx
    );

    // Ray-cast walls from truth pose then apply observe_distance_in().
    double observe_distance_sensor(
        const distance_loc::Vec2& robot_pos,
        double heading_deg,
        const distance_loc::Vec2& sensor_offset_rb,
        double sensor_rel_deg,
        int sensor_idx,
        const std::vector<Obstacle>& obstacles = {}
    );

    // Backward-compatible overload for AABB-only obstacle vectors.
    double observe_distance_sensor(
        const distance_loc::Vec2& robot_pos,
        double heading_deg,
        const distance_loc::Vec2& sensor_offset_rb,
        double sensor_rel_deg,
        int sensor_idx,
        const std::vector<AABB>& obstacles
    );

    // Collision-stall pipeline:
    // after collision, odom delta is forced to zero for configured ticks.
    MotionDelta apply_collision_stall(const MotionDelta& odom_delta, bool colliding);

    // Reset temporal state (dropout timers, stall timer).
    void reset_temporal_state();

private:
    static double wrap_heading_deg(double deg);

    noise::NoiseGenerator rng_;
    OdomNoiseConfig odom_config_;
    SensorNoiseConfig sensor_config_;
    std::array<int, 4> long_dropout_remaining_ = { 0, 0, 0, 0 };
    int collision_stall_remaining_ = 0;
};

} // namespace sim
