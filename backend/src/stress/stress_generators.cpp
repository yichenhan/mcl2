#include "stress/stress_generators.hpp"

#include "noise/failure_config.hpp"

#include <algorithm>
#include <cmath>
#include <random>

namespace stress {

RouteResult generate_route(const sim::Field& field, int total_ticks, uint64_t seed) {
    std::mt19937 rng(static_cast<uint32_t>(seed));
    RouteResult result;
    const double fh = field.field_half;
    std::uniform_real_distribution<double> pos_dist(-fh + 10.0, fh - 10.0);
    std::uniform_real_distribution<double> heading_dist(0.0, 360.0);
    for (int attempt = 0; attempt < 100; ++attempt) {
        const double x = pos_dist(rng);
        const double y = pos_dist(rng);
        if (field.is_passable(x, y)) {
            result.start_pose = {x, y, heading_dist(rng)};
            break;
        }
    }

    sim::Physics physics(field);
    physics.set_state(result.start_pose);
    result.velocity_commands.reserve(static_cast<size_t>(total_ticks));

    std::pair<double, double> last_cmd{0.0, 0.0};
    while (static_cast<int>(result.velocity_commands.size()) < total_ticks) {
        const int straight_len = std::uniform_int_distribution<int>(15, 45)(rng);
        const double linear_vel = std::uniform_real_distribution<double>(10.0, 32.0)(rng);
        for (int t = 0; t < straight_len && static_cast<int>(result.velocity_commands.size()) < total_ticks; ++t) {
            const auto step = physics.step_continuous(linear_vel, 0.0);
            last_cmd = {linear_vel, 0.0};
            if (step.colliding) break;
            result.velocity_commands.push_back(last_cmd);
        }
        const int turn_len = std::uniform_int_distribution<int>(4, 12)(rng);
        const double angular_vel = std::uniform_real_distribution<double>(-120.0, 120.0)(rng);
        for (int t = 0; t < turn_len && static_cast<int>(result.velocity_commands.size()) < total_ticks; ++t) {
            physics.step_continuous(0.0, angular_vel);
            last_cmd = {0.0, angular_vel};
            result.velocity_commands.push_back(last_cmd);
        }
    }
    while (static_cast<int>(result.velocity_commands.size()) < total_ticks)
        result.velocity_commands.push_back(last_cmd);
    return result;
}

std::vector<sim::Obstacle> generate_random_obstacles(uint64_t seed, const sim::Field& base_field, int count) {
    std::mt19937 rng(static_cast<uint32_t>(seed + 17u));
    std::vector<sim::Obstacle> out;
    const double fh = base_field.field_half;
    std::uniform_real_distribution<double> pos(-fh + 14.0, fh - 14.0);
    std::uniform_real_distribution<double> size_dist(8.0, 18.0);
    for (int i = 0; i < count; ++i) {
        const double cx = pos(rng);
        const double cy = pos(rng);
        const double w = size_dist(rng);
        const double h = size_dist(rng);
        out.emplace_back(sim::AABB{cx - w * 0.5, cy - h * 0.5, cx + w * 0.5, cy + h * 0.5});
    }
    return out;
}

StressNoiseBundle generate_stress_noise(int profile_id, int max_ticks, uint64_t seed, double field_half) {
    std::mt19937 rng(static_cast<uint32_t>(seed + static_cast<uint64_t>(profile_id) * 7919u));
    StressNoiseBundle b;
    pursuit::FailureGenConfig& fg = b.failure_gen;
    fg.min_duration_ticks = 5;
    fg.max_duration_ticks = 25;
    fg.sensor_dead_max_duration_ticks = 25;
    fg.sensor_stuck_max_duration_ticks = 6;
    fg.odom_spike_max_duration_ticks = 12;
    fg.heading_bias_max_duration_ticks = 35;
    fg.spurious_reflection_max_duration_ticks = 1;

    switch (profile_id % 6) {
    case 0:
        b.odom.trans_noise_frac = std::uniform_real_distribution<double>(0.02, 0.05)(rng);
        b.sensor.gaussian_stddev_mm = std::uniform_real_distribution<double>(10.0, 22.0)(rng);
        break;
    case 1:
        b.odom.trans_noise_frac = std::uniform_real_distribution<double>(0.05, 0.11)(rng);
        fg.odom_spike_prob = 0.02;
        fg.odom_spike_range = {1.35, 2.1};
        break;
    case 2:
        b.sensor.dropout_probability = std::uniform_real_distribution<double>(0.03, 0.07)(rng);
        fg.sensor_dead_prob = 0.015;
        fg.sensor_stuck_prob = 0.01;
        fg.spurious_reflection_prob = 0.012;
        break;
    case 3:
        fg.heading_bias_prob = 0.012;
        fg.heading_bias_range = {-8.0, 8.0};
        b.sensor.imu_noise_stddev_deg = std::uniform_real_distribution<double>(0.35, 0.75)(rng);
        break;
    case 4:
        b.odom.trans_noise_frac = 0.06;
        fg.sensor_dead_prob = 0.01;
        fg.odom_spike_prob = 0.008;
        fg.heading_bias_prob = 0.006;
        break;
    case 5:
        b.odom.trans_noise_frac = 0.08;
        fg.sensor_dead_prob = 0.018;
        fg.kidnap_prob = 0.04;
        fg.odom_spike_prob = 0.015;
        break;
    default:
        break;
    }

    b.failure_events = pursuit::generate_random_failures(fg, max_ticks, seed, field_half);
    apply_failure_guardrails(b.failure_events, max_ticks);
    return b;
}

void apply_failure_guardrails(std::vector<noise::FailureEvent>& events, int max_ticks) {
    for (auto& e : events) {
        if (e.start_tick < 50) e.start_tick = 50;
        if (e.type == noise::FailureType::HeadingBias) {
            e.param = std::max(-15.0, std::min(15.0, e.param));
        }
    }
    std::sort(events.begin(), events.end(), [](const noise::FailureEvent& a, const noise::FailureEvent& b) {
        return a.start_tick < b.start_tick;
    });
    int last_kidnap = -10000;
    for (auto& e : events) {
        if (e.type == noise::FailureType::Kidnap) {
            if (e.start_tick - last_kidnap < 200) e.start_tick = last_kidnap + 200;
            last_kidnap = e.start_tick;
        }
        if (e.start_tick + e.duration_ticks > max_ticks) {
            e.duration_ticks = std::max(1, max_ticks - e.start_tick);
        }
    }
}

} // namespace stress
