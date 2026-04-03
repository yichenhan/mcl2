#include "sim/sensor_model.hpp"

#include <cmath>
#include <limits>

namespace sim {

namespace {
constexpr double kInToMm = 25.4;
} // namespace

SensorModel::SensorModel(
    uint64_t seed,
    const OdomNoiseConfig& odom_config,
    const SensorNoiseConfig& sensor_config)
    : rng_(seed), odom_config_(odom_config), sensor_config_(sensor_config) {}

MotionDelta SensorModel::apply_odom_noise(const MotionDelta& true_delta, double /*heading_deg*/) {
    const double travel = std::fabs(true_delta.forward_in);
    const double sigma_fwd = odom_config_.trans_noise_frac * travel;
    const double sigma_lat = odom_config_.drift_noise_frac * travel;

    MotionDelta noisy = true_delta;
    noisy.forward_in = rng_.gaussian(true_delta.forward_in, sigma_fwd);
    noisy.lateral_in = rng_.gaussian(0.0, sigma_lat) + odom_config_.drift_per_tick_in;
    // Rotation noise is introduced in later phases.
    noisy.rotation_deg = true_delta.rotation_deg;
    return noisy;
}

double SensorModel::observe_heading(double true_heading_deg) {
    const double observed = rng_.gaussian(true_heading_deg, sensor_config_.imu_noise_stddev_deg);
    return wrap_heading_deg(observed);
}

double SensorModel::observe_distance_in(double true_distance_in, int sensor_idx) {
    if (sensor_idx < 0 || sensor_idx >= static_cast<int>(long_dropout_remaining_.size())) return -1.0;
    if (!std::isfinite(true_distance_in) || true_distance_in < 0.0) return -1.0;

    // 1) Long-dropout active.
    if (long_dropout_remaining_[sensor_idx] > 0) {
        long_dropout_remaining_[sensor_idx]--;
        return -1.0;
    }

    // 2) Random dropout (and optional long-dropout start).
    if (rng_.bernoulli(sensor_config_.dropout_probability)) {
        if (rng_.bernoulli(sensor_config_.long_dropout_probability)) {
            const int min_t = std::max(1, sensor_config_.long_dropout_min_ticks);
            const int max_t = std::max(min_t, sensor_config_.long_dropout_max_ticks);
            const int span = max_t - min_t + 1;
            int dur = min_t;
            if (span > 1) {
                const double u = rng_.uniform(0.0, 1.0);
                const int bucket = std::min(span - 1, static_cast<int>(u * span));
                dur = min_t + bucket;
            }
            // Current tick already returns -1; reserve remaining ticks.
            long_dropout_remaining_[sensor_idx] = std::max(0, dur - 1);
        }
        return -1.0;
    }

    // 3-4) True distance in mm.
    const double true_mm = true_distance_in * kInToMm;
    // 5) Range gate.
    if (true_mm > sensor_config_.range_max_mm) return -1.0;

    // 6-7) Range-ramp + base gaussian noise.
    double noisy_mm = true_mm + rng_.gaussian(0.0, sensor_config_.gaussian_stddev_mm);
    if (true_mm > sensor_config_.range_noise_start_mm) {
        const double extra_std = (true_mm - sensor_config_.range_noise_start_mm)
                                 * sensor_config_.range_noise_slope;
        noisy_mm += rng_.gaussian(0.0, extra_std);
    }

    // 8) Spurious reflection.
    if (rng_.bernoulli(sensor_config_.spurious_reflection_probability)) {
        noisy_mm = true_mm * rng_.uniform(0.3, 0.9);
    }

    // 9) Clamp and convert back to inches.
    if (!std::isfinite(noisy_mm)) return -1.0;
    noisy_mm = std::max(0.0, std::min(9999.0, noisy_mm));
    return noisy_mm / kInToMm;
}

double SensorModel::observe_distance_sensor(
    const distance_loc::Vec2& robot_pos,
    double heading_deg,
    const distance_loc::Vec2& sensor_offset_rb,
    double sensor_rel_deg,
    int sensor_idx,
    const std::vector<Obstacle>& obstacles) {
    const double true_dist_in = ray::ray_distance_with_obstacles(
        robot_pos, heading_deg, sensor_offset_rb, sensor_rel_deg, obstacles);
    return observe_distance_in(true_dist_in, sensor_idx);
}

double SensorModel::observe_distance_sensor(
    const distance_loc::Vec2& robot_pos,
    double heading_deg,
    const distance_loc::Vec2& sensor_offset_rb,
    double sensor_rel_deg,
    int sensor_idx,
    const std::vector<AABB>& obstacles) {
    std::vector<Obstacle> wrapped;
    wrapped.reserve(obstacles.size());
    for (const auto& b : obstacles) {
        wrapped.push_back(Obstacle{b, ""});
    }
    return observe_distance_sensor(robot_pos, heading_deg, sensor_offset_rb, sensor_rel_deg, sensor_idx, wrapped);
}

MotionDelta SensorModel::apply_collision_stall(const MotionDelta& odom_delta, bool colliding) {
    if (colliding) {
        collision_stall_remaining_ = std::max(0, sensor_config_.collision_stall_ticks);
    }

    if (collision_stall_remaining_ > 0) {
        collision_stall_remaining_--;
        return MotionDelta{ 0.0, 0.0, 0.0 };
    }
    return odom_delta;
}

void SensorModel::reset_temporal_state() {
    long_dropout_remaining_ = { 0, 0, 0, 0 };
    collision_stall_remaining_ = 0;
}

double SensorModel::wrap_heading_deg(double deg) {
    if (!std::isfinite(deg)) return 0.0;
    deg = std::fmod(deg, 360.0);
    if (deg < 0.0) deg += 360.0;
    return deg;
}

} // namespace sim
