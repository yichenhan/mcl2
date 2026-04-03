#include "noise/failure_config.hpp"

#include <algorithm>
#include <array>
#include <random>

namespace pursuit {

namespace {

double uniform(std::mt19937& rng, double lo, double hi) {
    std::uniform_real_distribution<double> dist(lo, hi);
    return dist(rng);
}

int uniform_int(std::mt19937& rng, int lo, int hi) {
    std::uniform_int_distribution<int> dist(lo, hi);
    return dist(rng);
}

bool roll(std::mt19937& rng, double p) {
    if (p <= 0.0) return false;
    if (p >= 1.0) return true;
    std::bernoulli_distribution d(p);
    return d(rng);
}

noise::FailureEvent make_sensor_event(noise::FailureType type, int tick, int duration, int sensor_idx, double param = 0.0) {
    noise::FailureEvent e;
    e.type = type;
    e.start_tick = tick;
    e.duration_ticks = duration;
    e.sensor_idx = sensor_idx;
    e.param = param;
    return e;
}

} // namespace

std::vector<noise::FailureEvent> generate_random_failures(
    const FailureGenConfig& config,
    int max_ticks,
    uint64_t seed,
    double field_half) {
    std::vector<noise::FailureEvent> out;
    std::mt19937 rng(static_cast<std::mt19937::result_type>(seed));
    if (max_ticks <= 0) return out;

    const int min_d = std::max(1, config.min_duration_ticks);
    const int legacy_max = std::max(min_d, config.max_duration_ticks);

    auto cap_ticks = [&](int per_type_max) {
        const int hi = (per_type_max > 0) ? per_type_max : legacy_max;
        return std::max(min_d, hi);
    };

    std::array<int, 4> sensor_busy_until{0, 0, 0, 0};

    for (int t = 0; t < max_ticks; ++t) {
        const int dead_cap = cap_ticks(config.sensor_dead_max_duration_ticks);
        const int stuck_cap = cap_ticks(config.sensor_stuck_max_duration_ticks);

        for (int s = 0; s < 4; ++s) {
            if (sensor_busy_until[static_cast<size_t>(s)] > t) continue;
            // When caps match, draw duration once (preserves RNG vs legacy single max_d).
            const int dur_shared =
                (dead_cap == stuck_cap) ? uniform_int(rng, min_d, dead_cap) : -1;
            if (roll(rng, config.sensor_dead_prob)) {
                const int dur =
                    dead_cap == stuck_cap ? dur_shared : uniform_int(rng, min_d, dead_cap);
                out.push_back(make_sensor_event(noise::FailureType::SensorDead, t, dur, s));
                sensor_busy_until[static_cast<size_t>(s)] = t + dur;
                continue;
            }
            if (roll(rng, config.sensor_stuck_prob)) {
                const int dur =
                    dead_cap == stuck_cap ? dur_shared : uniform_int(rng, min_d, stuck_cap);
                out.push_back(make_sensor_event(noise::FailureType::SensorStuck, t, dur, s));
                sensor_busy_until[static_cast<size_t>(s)] = t + dur;
                continue;
            }
            if (roll(rng, config.spurious_reflection_prob)) {
                const int sp_hi = std::max(1, config.spurious_reflection_max_duration_ticks > 0
                    ? config.spurious_reflection_max_duration_ticks
                    : 1);
                const int spurious_dur = uniform_int(rng, 1, sp_hi);
                out.push_back(make_sensor_event(
                    noise::FailureType::SpuriousReflection,
                    t,
                    spurious_dur,
                    s,
                    uniform(rng, config.spurious_reflection_range.min, config.spurious_reflection_range.max)));
                sensor_busy_until[static_cast<size_t>(s)] = t + spurious_dur;
            }
        }

        if (roll(rng, config.odom_spike_prob)) {
            noise::FailureEvent e;
            e.type = noise::FailureType::OdomSpike;
            e.start_tick = t;
            e.duration_ticks = uniform_int(rng, min_d, cap_ticks(config.odom_spike_max_duration_ticks));
            e.param = uniform(rng, config.odom_spike_range.min, config.odom_spike_range.max);
            out.push_back(e);
        }
        if (roll(rng, config.heading_bias_prob)) {
            noise::FailureEvent e;
            e.type = noise::FailureType::HeadingBias;
            e.start_tick = t;
            e.duration_ticks = uniform_int(rng, min_d, cap_ticks(config.heading_bias_max_duration_ticks));
            e.param = uniform(rng, config.heading_bias_range.min, config.heading_bias_range.max);
            out.push_back(e);
        }
        if (roll(rng, config.kidnap_prob)) {
            noise::FailureEvent e;
            e.type = noise::FailureType::Kidnap;
            e.start_tick = t;
            e.duration_ticks = 1;
            e.target_x = uniform(rng, -field_half, field_half);
            e.target_y = uniform(rng, -field_half, field_half);
            e.param = 1.0;
            out.push_back(e);
        }
    }
    return out;
}

} // namespace pursuit
