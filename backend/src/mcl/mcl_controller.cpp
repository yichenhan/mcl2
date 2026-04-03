#include "mcl/mcl_controller.hpp"

#include "ray/ray_cast_obstacles.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <sstream>
#include <utility>

namespace mcl {

namespace {

nlohmann::json estimate_to_json(const Estimate& est) {
    return nlohmann::json{
        { "x", est.x },
        { "y", est.y },
    };
}

void remove_particles_fields(nlohmann::json& value) {
    if (value.is_object()) {
        value.erase("particles");
        for (auto& item : value.items()) {
            remove_particles_fields(item.value());
        }
        return;
    }
    if (value.is_array()) {
        for (auto& item : value) {
            remove_particles_fields(item);
        }
    }
}

void flatten_json(const nlohmann::json& value, const std::string& prefix,
                  std::vector<std::pair<std::string, std::string>>& out) {
    if (value.is_object()) {
        for (const auto& [k, v] : value.items()) {
            flatten_json(v, prefix.empty() ? k : prefix + "." + k, out);
        }
    } else if (value.is_array()) {
        for (size_t i = 0; i < value.size(); ++i) {
            flatten_json(value[i], prefix + "." + std::to_string(i), out);
        }
    } else {
        out.emplace_back(prefix, value.dump());
    }
}

} // namespace

MCLController::MCLController(
    const MCLConfig& mcl_config,
    const GateConfig& gate_config,
    LogFn log_fn)
    : engine_(mcl_config), gate_config_(gate_config), log_fn_(std::move(log_fn)) {
    if (!log_fn_) {
#ifdef NDEBUG_LOG
        log_fn_ = [](const std::string&) {};
#else
        log_fn_ = [](const std::string& msg) { std::cout << msg; };
#endif
    }
#ifndef NDEBUG_LOG
    std::printf("[MCLController] initialized with %d particles, field_half=%.1f\n",
                mcl_config.num_particles, mcl_config.field_half);
#endif
}

void MCLController::initialize_uniform(uint64_t seed) {
    engine_.initialize_uniform(seed);
    tick_count_ = 0;
}

void MCLController::predict(double delta_forward, double delta_rotation, double heading_deg, double delta_lateral) {
    engine_.predict(delta_forward, delta_rotation, heading_deg, delta_lateral);
   
}

void MCLController::update(const double readings[4], double heading_deg) {
    engine_.update(readings, heading_deg);
    int valid_sensor_count = 0;
    for (int i = 0; i < 4; ++i) {
        if (readings[i] >= 0.0) {
            valid_sensor_count++;
        }
    }
    emit_log(
        "post_update",
        nlohmann::json{
            { "valid_sensor_count", valid_sensor_count },
            { "update_skipped", false },
            { "post_update", snapshot_json() },
        });
}

void MCLController::resample() {
    engine_.resample();
    emit_log(
        "post_resample",
        nlohmann::json{
            { "post_resample", snapshot_json() },
            { "n_eff", engine_.n_eff() },
        });
}

Estimate MCLController::estimate() const {
    return engine_.estimate();
}

ClusterStats MCLController::cluster_stats() const {
    return engine_.cluster_stats();
}

double MCLController::n_eff() const {
    return engine_.n_eff();
}

const std::vector<Particle>& MCLController::particles() const {
    return engine_.particles();
}

MCLTickResult MCLController::tick(
    double delta_forward,
    double delta_rotation,
    double heading_deg,
    double delta_lateral,
    const std::array<double, 4>& readings,
    int min_sensors_for_update,
    const sim::Field* field,
    const Estimate* prev_accepted,
    double dt_sec,
    const GateEnables* gate_enables,
    const Pose* odom_pose) {
    MCLTickResult out;
    out.tick_count = ++tick_count_;
    if (odom_pose != nullptr) {
        out.odom_pose = *odom_pose;
    }
    out.observed_readings = readings;
    for (double reading : readings) {
        if (reading >= 0.0) {
            out.valid_sensor_count++;
        }
    }
    out.update_skipped = (out.valid_sensor_count < min_sensors_for_update);

    predict(delta_forward, delta_rotation, heading_deg, delta_lateral);
    fill_snapshot(out.post_predict);

    if (!out.update_skipped) {
        update(readings.data(), heading_deg);
    }
    fill_snapshot(out.post_update);

    resample();
    fill_snapshot(out.post_resample);

    const Estimate est = estimate();
    out.raw_estimate.x = est.x;
    out.raw_estimate.y = est.y;
    out.raw_estimate.theta = heading_deg;
    out.cluster_stats = cluster_stats();
    out.n_eff = n_eff();

    if (field != nullptr) {
        const auto& sensors = engine_.config().sensors;
        const distance_loc::Vec2 pos{ est.x, est.y };
        for (size_t i = 0; i < 4; ++i) {
            const double reading = readings[i];
            if (reading < 0.0) continue;
            const double predicted = ray::ray_distance_with_obstacles(
                pos,
                heading_deg,
                sensors[i].offset,
                sensors[i].angle_deg,
                field->obstacles);
            out.mcl_predicted_readings[i] = predicted;
            out.mcl_sensor_residuals[i] = std::fabs(predicted - reading);
        }
    }

    if (field != nullptr && prev_accepted != nullptr) {
        const GateEnables effective_enables = (gate_enables != nullptr) ? *gate_enables : GateEnables{};
        out.gate = gate_estimate(*field, readings, heading_deg, *prev_accepted, dt_sec, effective_enables);
    }

    emit_log("tick", nlohmann::json(out));
    return out;
}

const MCLEngine& MCLController::engine() const {
    return engine_;
}

const GateConfig& MCLController::gate_config() const {
    return gate_config_;
}

nlohmann::json MCLController::snapshot_json() const {
    nlohmann::json particles = nlohmann::json::array();
    for (const auto& p : engine_.particles()) {
        particles.push_back({
            { "x", p.x },
            { "y", p.y },
            { "weight", p.weight },
        });
    }
    const ClusterStats cs = engine_.cluster_stats();
    return nlohmann::json{
        { "particles", std::move(particles) },
        { "estimate", estimate_to_json(cs.centroid) },
        { "n_eff", engine_.n_eff() },
        { "spread", cs.spread },
        { "radius_90", cs.radius_90 },
    };
}

void MCLController::emit_log(const char* phase, nlohmann::json extra) const {
    extra["phase"] = phase;
#ifndef NDEBUG_LOG
    if (std::string(phase) == "tick" &&
        (log_interval_ticks_ <= 1 || tick_count_ % static_cast<uint64_t>(log_interval_ticks_) == 0)) {
        remove_particles_fields(extra);
        std::vector<std::pair<std::string, std::string>> kvs;
        flatten_json(extra, "", kvs);
        for (const auto& [key, val] : kvs) {
            std::printf("[TICK=%llu || %s]=%s\n",
                        static_cast<unsigned long long>(tick_count_), key.c_str(), val.c_str());
        }
        std::fflush(stdout);
    }
#endif
}

GateDecision MCLController::fail_decision(const char* reason) const {
    GateDecision d;
    d.accepted = false;
    d.reason = reason;
    return d;
}

GateDecision MCLController::gate_estimate(
    const sim::Field& field,
    const std::array<double, 4>& readings,
    double heading_deg,
    const Estimate& prev_estimate,
    double dt_sec,
    const GateEnables& gate_enables) const {
    GateDecision d;
    const Estimate est = engine_.estimate();
    const ClusterStats cs = engine_.cluster_stats();
    d.radius_90_in = cs.radius_90;
    d.spread_in = cs.spread;
    d.n_eff_at_gate = n_eff();

    const double dx = static_cast<double>(est.x) - static_cast<double>(prev_estimate.x);
    const double dy = static_cast<double>(est.y) - static_cast<double>(prev_estimate.y);
    d.jump_in = std::sqrt(dx * dx + dy * dy);
    const double safe_dt = std::max(dt_sec, 1e-6);
    const double speed_ft_per_s = (d.jump_in / 12.0) / safe_dt;
    d.centroid_jump_ft_per_s = speed_ft_per_s;

    d.would_fail_r90 = (cs.radius_90 > gate_config_.max_radius_90_in);

    {
        int valid_count = 0;
        double worst_residual = 0.0;
        double worst_threshold = 0.0;
        bool any_residual_fail = false;
        const auto& sensors = engine_.config().sensors;
        for (int i = 0; i < 4; ++i) {
            if (readings[static_cast<size_t>(i)] < 0.0) continue;
            valid_count++;
            const distance_loc::Vec2 pos{est.x, est.y};
            const double pred = ray::ray_distance_with_obstacles(
                pos, heading_deg, sensors[i].offset, sensors[i].angle_deg, field.obstacles);
            const double residual = std::fabs(pred - readings[static_cast<size_t>(i)]);
            const double max_resid = (readings[static_cast<size_t>(i)] < gate_config_.sensor_close_range_in)
                ? gate_config_.sensor_close_tolerance_in
                : readings[static_cast<size_t>(i)] * gate_config_.sensor_far_tolerance_pct;
            if (residual > max_resid) any_residual_fail = true;
            if (residual > worst_residual) {
                worst_residual = residual;
                worst_threshold = max_resid;
            }
        }
        d.max_sensor_residual_in = worst_residual;
        d.residual_threshold_in = worst_threshold;
        d.would_fail_residual =
            any_residual_fail || (valid_count < gate_config_.min_valid_sensors_for_residual);
    }

    d.would_fail_centroid_jump = (speed_ft_per_s > gate_config_.max_centroid_jump_ft_per_s);
    d.would_fail_passability = !field.is_passable(est.x, est.y);

    if (gate_enables.r90 && d.would_fail_r90) {
        d.accepted = false;
        d.failed_r90 = true;
        d.reason = "r90 gate";
    } else if (gate_enables.residual && d.would_fail_residual) {
        d.accepted = false;
        d.failed_residual = true;
        d.reason = (d.max_sensor_residual_in > d.residual_threshold_in && d.residual_threshold_in > 0.0)
            ? "sensor residual gate"
            : "insufficient valid sensors";
    } else if (gate_enables.centroid_jump && d.would_fail_centroid_jump) {
        d.accepted = false;
        d.failed_centroid_jump = true;
        d.reason = "particle centroid jump gate";
    } else if (gate_enables.passability && d.would_fail_passability) {
        d.accepted = false;
        d.failed_passability = true;
        d.reason = "passability gate";
    }

    emit_log("gate", nlohmann::json{ { "gate", nlohmann::json(d) } });
    return d;
}

void MCLController::fill_snapshot(PhaseSnapshot& out) const {
    out.particles = particles();
    const ClusterStats cs = cluster_stats();
    out.estimate = cs.centroid;
    out.n_eff = n_eff();
    out.spread = cs.spread;
    out.radius_90 = cs.radius_90;
}

} // namespace mcl
