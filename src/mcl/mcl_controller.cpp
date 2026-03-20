#include "mcl/mcl_controller.hpp"

#include "ray/ray_cast_obstacles.hpp"

#include <cmath>
#include <sstream>

namespace mcl {

MCLController::MCLController(const MCLConfig& mcl_config, const GateConfig& gate_config)
    : engine_(mcl_config), gate_config_(gate_config) {}

void MCLController::initialize_uniform(uint64_t seed) {
    engine_.initialize_uniform(seed);
}

void MCLController::predict(double delta_forward, double delta_rotation, double heading_deg, double delta_lateral) {
    engine_.predict(delta_forward, delta_rotation, heading_deg, delta_lateral);
}

void MCLController::update(const double readings[4], double heading_deg) {
    engine_.update(readings, heading_deg);
}

void MCLController::resample() {
    engine_.resample();
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

const MCLEngine& MCLController::engine() const {
    return engine_;
}

const GateConfig& MCLController::gate_config() const {
    return gate_config_;
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
    double dt_sec) const {
    GateDecision d;
    const Estimate est = engine_.estimate();
    const ClusterStats cs = engine_.cluster_stats();
    d.radius_90_in = cs.radius_90;
    d.spread_in = cs.spread;

    const double dx = static_cast<double>(est.x) - static_cast<double>(prev_accepted.x);
    const double dy = static_cast<double>(est.y) - static_cast<double>(prev_accepted.y);
    d.jump_in = std::sqrt(dx * dx + dy * dy);

    const double max_jump_speed =
        std::max(0.0, gate_config_.max_estimate_speed_ft_per_s) * 12.0 * std::max(1e-6, dt_sec);
    const double max_jump = std::min(gate_config_.max_jump_in, max_jump_speed);
    if (d.jump_in > max_jump) {
        d.accepted = false;
        d.failed_velocity = true;
        d.reason = "velocity/jump gate";
        return d;
    }

    if (cs.radius_90 > gate_config_.max_radius_90_in || cs.spread > gate_config_.max_spread_in) {
        d.accepted = false;
        d.failed_spread = true;
        d.reason = "spread gate";
        return d;
    }

    if (!field.is_passable(est.x, est.y)) {
        d.accepted = false;
        d.failed_passability = true;
        d.reason = "passability gate";
        return d;
    }

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
            return d;
        }
    }
    if (valid_count < gate_config_.min_valid_sensors_for_residual) {
        d.accepted = false;
        d.failed_residual = true;
        d.reason = "insufficient valid sensors";
        return d;
    }

    if (!wall_sum_ok(readings, heading_deg, field)) {
        d.accepted = false;
        d.failed_wall_sum = true;
        d.reason = "wall-sum gate";
        return d;
    }
    return d;
}

} // namespace mcl
