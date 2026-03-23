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

} // namespace

MCLController::MCLController(
    const MCLConfig& mcl_config,
    const GateConfig& gate_config,
    LogFn log_fn)
    : engine_(mcl_config), gate_config_(gate_config), log_fn_(std::move(log_fn)) {
    if (!log_fn_) {
        log_fn_ = [](const std::string& msg) { std::cout << msg; };
    }
}

void MCLController::initialize_uniform(uint64_t seed) {
    engine_.initialize_uniform(seed);
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
    const GateEnables* gate_enables) {
    MCLTickResult out;
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

    if (field != nullptr && prev_accepted != nullptr) {
        const GateEnables effective_enables = (gate_enables != nullptr) ? *gate_enables : GateEnables{};
        out.gate = gate_estimate(*field, readings, heading_deg, *prev_accepted, dt_sec, effective_enables);
    }

    const std::string tick_json = nlohmann::json(out).dump();
    std::printf("[MCL_JSON_START] %s [MCL_JSON_FINISH]\n", tick_json.c_str());
    std::fflush(stdout);

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
    if (!log_fn_) return;
    extra["phase"] = phase;
}

GateDecision MCLController::fail_decision(const char* reason) const {
    GateDecision d;
    d.accepted = false;
    d.reason = reason;
    return d;
}

bool MCLController::wall_sum_ok(
    const std::array<double, 4>& readings,
    double heading_deg,
    const sim::Field& field) const {
    // Wall-sum logic is only valid in wall-only maps.
    if (!field.obstacles.empty()) return true;
    const bool has_l = readings[0] >= 0.0;
    const bool has_r = readings[1] >= 0.0;
    const bool has_f = readings[2] >= 0.0;
    const bool has_b = readings[3] >= 0.0;

    const auto& sensors = engine_.config().sensors;
    const double h = distance_loc::deg2rad(heading_deg);
    const double c = std::cos(h);
    const double s = std::sin(h);
    const double field_size = 2.0 * field.field_half;
    const double tol = gate_config_.wall_sum_tolerance_in;

    if (has_l && has_r) {
        const double left_proj = readings[0] * std::fabs(c);
        const double right_proj = readings[1] * std::fabs(c);
        const double lox = sensors[0].offset.x * c + sensors[0].offset.y * s;
        const double rox = sensors[1].offset.x * c + sensors[1].offset.y * s;
        const double distance_from_left = left_proj - lox;
        const double distance_from_right = right_proj + rox;
        const double wall_sum = distance_from_left + distance_from_right;
        if (std::fabs(wall_sum - field_size) > tol) return false;
    }
    if (has_f && has_b) {
        const double front_proj = readings[2] * std::fabs(c);
        const double back_proj = readings[3] * std::fabs(c);
        const double foy = -sensors[2].offset.x * s + sensors[2].offset.y * c;
        const double boy = -sensors[3].offset.x * s + sensors[3].offset.y * c;
        const double distance_from_front = front_proj + foy;
        const double distance_from_back = back_proj - boy;
        const double wall_sum = distance_from_front + distance_from_back;
        if (std::fabs(wall_sum - field_size) > tol) return false;
    }
    return true;
}

GateDecision MCLController::gate_estimate(
    const sim::Field& field,
    const std::array<double, 4>& readings,
    double heading_deg,
    const Estimate& prev_accepted,
    double dt_sec,
    const GateEnables& gate_enables) const {
    GateDecision d;
    const Estimate est = engine_.estimate();
    const ClusterStats cs = engine_.cluster_stats();
    d.radius_90_in = cs.radius_90;
    d.spread_in = cs.spread;

    if (gate_enables.r90 && cs.radius_90 > gate_config_.max_radius_90_in) {
        d.accepted = false;
        d.failed_r90 = true;
        d.reason = "r90 gate";
        emit_log("gate", nlohmann::json{ { "gate", nlohmann::json(d) } });
        return d;
    }

    const double dx = static_cast<double>(est.x) - static_cast<double>(prev_accepted.x);
    const double dy = static_cast<double>(est.y) - static_cast<double>(prev_accepted.y);
    d.jump_in = std::sqrt(dx * dx + dy * dy);

    if (gate_enables.residual) {
        int valid_count = 0;
        const auto& sensors = engine_.config().sensors;
        for (int i = 0; i < 4; ++i) {
            if (readings[static_cast<size_t>(i)] < 0.0) continue;
            valid_count++;
            const distance_loc::Vec2 pos{est.x, est.y};
            const double pred = ray::ray_distance_with_obstacles(
                pos, heading_deg, sensors[i].offset, sensors[i].angle_deg, field.obstacles);
            const double residual = std::fabs(pred - readings[static_cast<size_t>(i)]);
            if (residual > gate_config_.max_sensor_residual_in) {
                d.accepted = false;
                d.failed_residual = true;
                d.reason = "sensor residual gate";
                emit_log("gate", nlohmann::json{ { "gate", nlohmann::json(d) } });
                return d;
            }
        }
        if (valid_count < gate_config_.min_valid_sensors_for_residual) {
            d.accepted = false;
            d.failed_residual = true;
            d.reason = "insufficient valid sensors";
            emit_log("gate", nlohmann::json{ { "gate", nlohmann::json(d) } });
            return d;
        }
    }

    const double safe_dt = std::max(dt_sec, 1e-6);
    const double speed_ft_per_s = (d.jump_in / 12.0) / safe_dt;
    if (gate_enables.velocity && speed_ft_per_s > gate_config_.max_estimate_speed_ft_per_s) {
        d.accepted = false;
        d.failed_velocity = true;
        d.reason = "velocity gate";
        emit_log("gate", nlohmann::json{ { "gate", nlohmann::json(d) } });
        return d;
    }

    if (gate_enables.passability && !field.is_passable(est.x, est.y)) {
        d.accepted = false;
        d.failed_passability = true;
        d.reason = "passability gate";
        emit_log("gate", nlohmann::json{ { "gate", nlohmann::json(d) } });
        return d;
    }

    if (gate_enables.wall_sum && !wall_sum_ok(readings, heading_deg, field)) {
        d.accepted = false;
        d.failed_wall_sum = true;
        d.reason = "wall-sum gate";
        emit_log("gate", nlohmann::json{ { "gate", nlohmann::json(d) } });
        return d;
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
