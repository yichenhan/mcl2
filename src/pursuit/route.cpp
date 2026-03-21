#include "pursuit/route.hpp"

#include "nlohmann/json.hpp"

#include <fstream>
#include <random>
#include <stdexcept>

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

distance_loc::Vec2 pick_passable_target(
    const sim::Field& field,
    std::mt19937& rng,
    double preferred_x,
    double preferred_y,
    const std::vector<distance_loc::Vec2>& fallbacks) {
    double x = preferred_x;
    double y = preferred_y;
    field.clamp(x, y);
    if (field.is_passable(x, y)) return {x, y};

    for (const auto& fb : fallbacks) {
        double fx = fb.x;
        double fy = fb.y;
        field.clamp(fx, fy);
        if (field.is_passable(fx, fy)) return {fx, fy};
    }

    // Deterministic random retry loop.
    const double lo = -(field.field_half - field.robot_radius);
    const double hi = (field.field_half - field.robot_radius);
    for (int i = 0; i < 256; ++i) {
        const double rx = uniform(rng, lo, hi);
        const double ry = uniform(rng, lo, hi);
        if (field.is_passable(rx, ry)) return {rx, ry};
    }

    // Last resort: coarse grid scan for first passable point.
    for (int gy = -64; gy <= 64; ++gy) {
        for (int gx = -64; gx <= 64; ++gx) {
            const double tx = gx;
            const double ty = gy;
            if (field.is_passable(tx, ty)) return {tx, ty};
        }
    }
    return {0.0, 0.0};
}
} // namespace

void validate_route(const RouteDefinition& route, double field_half) {
    if (route.waypoints.empty()) {
        throw std::runtime_error("route must contain at least one waypoint");
    }
    for (const auto& w : route.waypoints) {
        if (w.x < -field_half || w.x > field_half || w.y < -field_half || w.y > field_half) {
            throw std::runtime_error("waypoint out of field bounds");
        }
    }
    sim::Field field;
    field.field_half = field_half;
    field.obstacles = route.obstacles;
    for (const auto& w : route.waypoints) {
        if (!field.is_passable(w.x, w.y)) {
            throw std::runtime_error("waypoint not passable (inside obstacle or outside field)");
        }
    }
    for (size_t i = 0; i + 1 < route.waypoints.size(); ++i) {
        const auto& a = route.waypoints[i];
        const auto& b = route.waypoints[i + 1];
        if (field.segment_hits_obstacle(a.x, a.y, b.x, b.y)) {
            throw std::runtime_error("route segment intersects obstacle");
        }
    }
    if (!field.is_passable(route.initial_position.x, route.initial_position.y)) {
        throw std::runtime_error("initial_position not passable");
    }
    if (route.max_ticks <= 0) {
        throw std::runtime_error("max_ticks must be positive");
    }
}

RouteDefinition load_route(const std::string& path) {
    std::ifstream in(path);
    if (!in.is_open()) {
        throw std::runtime_error("failed to open route file: " + path);
    }
    nlohmann::json j;
    in >> j;

    RouteDefinition out;
    out.name = j.value("name", std::string("unnamed_route"));
    out.description = j.value("description", std::string(""));
    if (j.contains("initial_state") && j["initial_state"].is_object()) {
        out.initial_position.x = j["initial_state"].value("x", out.initial_position.x);
        out.initial_position.y = j["initial_state"].value("y", out.initial_position.y);
    }
    out.initial_heading_deg = j.value("initial_heading_deg", 0.0);
    out.failure_seed = j.value("failure_seed", static_cast<uint64_t>(42));
    out.max_inches_odom_delta_per_tick = j.value("max_inches_odom_delta_per_tick", out.max_inches_odom_delta_per_tick);
    out.max_ticks = j.value("max_ticks", 500);

    if (j.contains("waypoints") && j["waypoints"].is_array()) {
        for (const auto& w : j["waypoints"]) {
            out.waypoints.push_back(distance_loc::Vec2{
                w.value("x", 0.0),
                w.value("y", 0.0),
            });
        }
    }

    if (j.contains("obstacles") && j["obstacles"].is_array()) {
        for (const auto& o : j["obstacles"]) {
            out.obstacles.push_back(sim::AABB{
                o.value("min_x", 0.0),
                o.value("min_y", 0.0),
                o.value("max_x", 0.0),
                o.value("max_y", 0.0),
            });
        }
    }

    if (j.contains("pure_pursuit") && j["pure_pursuit"].is_object()) {
        const auto& pp = j["pure_pursuit"];
        out.pure_pursuit.lookahead_distance = pp.value("lookahead_distance", out.pure_pursuit.lookahead_distance);
        out.pure_pursuit.linear_velocity = pp.value("linear_velocity", out.pure_pursuit.linear_velocity);
        out.pure_pursuit.waypoint_tolerance = pp.value("waypoint_tolerance", out.pure_pursuit.waypoint_tolerance);
        out.pure_pursuit.max_angular_velocity_deg =
            pp.value("max_angular_velocity_deg", out.pure_pursuit.max_angular_velocity_deg);
    }

    if (j.contains("failure_config") && j["failure_config"].is_object()) {
        const auto& fc = j["failure_config"];
        out.failure_config.sensor_dead_prob = fc.value("sensor_dead_prob", out.failure_config.sensor_dead_prob);
        out.failure_config.sensor_stuck_prob = fc.value("sensor_stuck_prob", out.failure_config.sensor_stuck_prob);
        out.failure_config.odom_spike_prob = fc.value("odom_spike_prob", out.failure_config.odom_spike_prob);
        out.failure_config.heading_bias_prob = fc.value("heading_bias_prob", out.failure_config.heading_bias_prob);
        out.failure_config.spurious_reflection_prob =
            fc.value("spurious_reflection_prob", out.failure_config.spurious_reflection_prob);
        out.failure_config.kidnap_prob = fc.value("kidnap_prob", out.failure_config.kidnap_prob);
        out.failure_config.min_duration_ticks = fc.value("min_duration_ticks", out.failure_config.min_duration_ticks);
        out.failure_config.max_duration_ticks = fc.value("max_duration_ticks", out.failure_config.max_duration_ticks);
        if (fc.contains("odom_spike_range") && fc["odom_spike_range"].is_array() && fc["odom_spike_range"].size() == 2) {
            out.failure_config.odom_spike_range.min = fc["odom_spike_range"][0].get<double>();
            out.failure_config.odom_spike_range.max = fc["odom_spike_range"][1].get<double>();
        }
        if (fc.contains("heading_bias_range") && fc["heading_bias_range"].is_array() && fc["heading_bias_range"].size() == 2) {
            out.failure_config.heading_bias_range.min = fc["heading_bias_range"][0].get<double>();
            out.failure_config.heading_bias_range.max = fc["heading_bias_range"][1].get<double>();
        }
        if (fc.contains("spurious_reflection_range") &&
            fc["spurious_reflection_range"].is_array() &&
            fc["spurious_reflection_range"].size() == 2) {
            out.failure_config.spurious_reflection_range.min = fc["spurious_reflection_range"][0].get<double>();
            out.failure_config.spurious_reflection_range.max = fc["spurious_reflection_range"][1].get<double>();
        }
    }

    if (j.contains("kidnap_events") && j["kidnap_events"].is_array()) {
        for (const auto& ke : j["kidnap_events"]) {
            KidnapEvent event;
            event.tick = ke.value("tick", 0);
            if (ke.contains("target") && ke["target"].is_object()) {
                event.random_target = false;
                event.x = ke["target"].value("x", 0.0);
                event.y = ke["target"].value("y", 0.0);
            } else {
                event.random_target = true;
            }
            out.kidnap_events.push_back(event);
        }
    }

    validate_route(out);
    return out;
}

std::vector<noise::FailureEvent> generate_random_failures(
    const FailureGenConfig& config,
    int max_ticks,
    uint64_t seed,
    double field_half) {
    std::vector<noise::FailureEvent> out;
    std::mt19937 rng(static_cast<std::mt19937::result_type>(seed));
    if (max_ticks <= 0) return out;

    const int min_d = std::max(1, config.min_duration_ticks);
    const int max_d = std::max(min_d, config.max_duration_ticks);

    std::array<int, 4> sensor_busy_until{0, 0, 0, 0};

    for (int t = 0; t < max_ticks; ++t) {
        for (int s = 0; s < 4; ++s) {
            if (sensor_busy_until[static_cast<size_t>(s)] > t) continue;
            const int dur = uniform_int(rng, min_d, max_d);
            if (roll(rng, config.sensor_dead_prob)) {
                out.push_back(make_sensor_event(noise::FailureType::SensorDead, t, dur, s));
                sensor_busy_until[static_cast<size_t>(s)] = t + dur;
                continue;
            }
            if (roll(rng, config.sensor_stuck_prob)) {
                out.push_back(make_sensor_event(noise::FailureType::SensorStuck, t, dur, s));
                sensor_busy_until[static_cast<size_t>(s)] = t + dur;
                continue;
            }
            if (roll(rng, config.spurious_reflection_prob)) {
                // Spurious reflections should be sparse, instantaneous glitches.
                // Keep them to exactly one tick regardless of duration config.
                const int spurious_dur = 1;
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
            e.duration_ticks = uniform_int(rng, min_d, max_d);
            e.param = uniform(rng, config.odom_spike_range.min, config.odom_spike_range.max);
            out.push_back(e);
        }
        if (roll(rng, config.heading_bias_prob)) {
            noise::FailureEvent e;
            e.type = noise::FailureType::HeadingBias;
            e.start_tick = t;
            e.duration_ticks = uniform_int(rng, min_d, max_d);
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

std::vector<noise::FailureEvent> build_failure_schedule(const RouteDefinition& route, double field_half) {
    std::vector<noise::FailureEvent> out =
        generate_random_failures(route.failure_config, route.max_ticks, route.failure_seed, field_half);

    sim::Field field;
    field.field_half = field_half;
    field.obstacles = route.obstacles;

    std::mt19937 rng(static_cast<std::mt19937::result_type>(route.failure_seed ^ 0x9e3779b97f4a7c15ULL));
    for (auto& e : out) {
        if (e.type != noise::FailureType::Kidnap) continue;
        const auto safe = pick_passable_target(
            field, rng, e.target_x, e.target_y, route.waypoints);
        e.target_x = safe.x;
        e.target_y = safe.y;
    }

    for (const auto& k : route.kidnap_events) {
        noise::FailureEvent e;
        e.type = noise::FailureType::Kidnap;
        e.start_tick = k.tick;
        e.duration_ticks = 1;
        distance_loc::Vec2 safe;
        if (k.random_target) {
            safe = pick_passable_target(field, rng, 0.0, 0.0, route.waypoints);
        } else {
            safe = pick_passable_target(field, rng, k.x, k.y, route.waypoints);
        }
        e.target_x = safe.x;
        e.target_y = safe.y;
        out.push_back(e);
    }
    return out;
}

} // namespace pursuit

