#pragma once

#include "mcl/mcl_engine.hpp"
#include "sim/field.hpp"
#include "nlohmann/json.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace mcl {

struct GateConfig {
    double max_centroid_jump_ft_per_s = 360.0 / 12.0;
    double max_jump_in = 12.0;
    double max_radius_90_in = 6.0;
    // Distance-dependent residual thresholds (100% tolerance over sensor spec).
    // < 200mm: spec ±15mm → ×2 = 30mm = 1.181 in.
    // ≥ 200mm: spec 5%   → ×2 = 10% of reading.
    double sensor_close_range_in = 7.874;
    double sensor_close_tolerance_in = 1.181;
    double sensor_far_tolerance_pct = 0.10;
    int min_valid_sensors_for_residual = 2;
};

struct GateEnables {
    bool centroid_jump = true;
    bool r90 = true;
    bool passability = true;
    bool residual = true;
};

struct GateDecision {
    bool accepted = true;
    bool failed_centroid_jump = false;
    bool failed_r90 = false;
    bool failed_passability = false;
    bool failed_residual = false;
    double jump_in = 0.0;
    double radius_90_in = 0.0;
    double spread_in = 0.0;
    std::string reason;

    bool would_fail_centroid_jump = false;
    bool would_fail_r90 = false;
    bool would_fail_passability = false;
    bool would_fail_residual = false;

    double max_sensor_residual_in = 0.0;
    double residual_threshold_in = 0.0;
    double centroid_jump_ft_per_s = 0.0;
    double n_eff_at_gate = 0.0;
};

struct Pose {
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
};

struct PhaseSnapshot {
    std::vector<Particle> particles;
    Estimate estimate{ 0.0f, 0.0f };
    double n_eff = 0.0;
    double spread = 0.0;
    double radius_90 = 0.0;
};

struct MCLTickResult {
    uint64_t tick_count = 0;
    Pose raw_odom{};
    Pose raw_estimate{};
    Pose chassis_pose{};
    std::array<double, 4> observed_readings{ -1.0, -1.0, -1.0, -1.0 };
    std::array<double, 4> mcl_sensor_residuals{ 0.0, 0.0, 0.0, 0.0 };
    std::array<double, 4> mcl_predicted_readings{ -1.0, -1.0, -1.0, -1.0 };
    GateDecision gate{};
    int valid_sensor_count = 0;
    bool update_skipped = false;
    ClusterStats cluster_stats{ 0.0, 0.0, Estimate{ 0.0f, 0.0f } };
    double n_eff = 0.0;
    PhaseSnapshot post_predict{};
    PhaseSnapshot post_update{};
    PhaseSnapshot post_resample{};
};

inline void to_json(nlohmann::json& j, const Pose& p) {
    j = nlohmann::json{
        { "x", p.x },
        { "y", p.y },
        { "theta", p.theta },
    };
}

inline void from_json(const nlohmann::json& j, Pose& p) {
    p.x = j.value("x", 0.0);
    p.y = j.value("y", 0.0);
    p.theta = j.value("theta", 0.0);
}

inline void to_json(nlohmann::json& j, const PhaseSnapshot& s) {
    j = nlohmann::json::object();
    j["particles"] = nlohmann::json::array();
    for (const auto& p : s.particles) {
        j["particles"].push_back({
            { "x", p.x },
            { "y", p.y },
            { "weight", p.weight },
        });
    }
    j["estimate"] = {
        { "x", s.estimate.x },
        { "y", s.estimate.y },
    };
    j["n_eff"] = s.n_eff;
    j["spread"] = s.spread;
    j["radius_90"] = s.radius_90;
}

inline void from_json(const nlohmann::json& j, PhaseSnapshot& s) {
    s.particles.clear();
    if (j.contains("particles") && j["particles"].is_array()) {
        for (const auto& p : j["particles"]) {
            Particle part;
            part.x = p.value("x", 0.0f);
            part.y = p.value("y", 0.0f);
            part.weight = p.value("weight", 0.0f);
            s.particles.push_back(part);
        }
    }
    const auto& est = j.contains("estimate") ? j["estimate"] : nlohmann::json::object();
    s.estimate.x = est.value("x", 0.0f);
    s.estimate.y = est.value("y", 0.0f);
    s.n_eff = j.value("n_eff", 0.0);
    s.spread = j.value("spread", 0.0);
    s.radius_90 = j.value("radius_90", 0.0);
}

inline void to_json(nlohmann::json& j, const GateDecision& d) {
    j = nlohmann::json{
        { "accepted", d.accepted },
        { "failed_centroid_jump", d.failed_centroid_jump },
        { "failed_r90", d.failed_r90 },
        { "failed_passability", d.failed_passability },
        { "failed_residual", d.failed_residual },
        { "jump_in", d.jump_in },
        { "radius_90_in", d.radius_90_in },
        { "spread_in", d.spread_in },
        { "reason", d.reason },
        { "would_fail_centroid_jump", d.would_fail_centroid_jump },
        { "would_fail_r90", d.would_fail_r90 },
        { "would_fail_passability", d.would_fail_passability },
        { "would_fail_residual", d.would_fail_residual },
        { "max_sensor_residual_in", d.max_sensor_residual_in },
        { "residual_threshold_in", d.residual_threshold_in },
        { "centroid_jump_ft_per_s", d.centroid_jump_ft_per_s },
        { "n_eff_at_gate", d.n_eff_at_gate },
    };
}

inline void from_json(const nlohmann::json& j, GateDecision& d) {
    d.accepted = j.value("accepted", true);
    d.failed_centroid_jump = j.value("failed_centroid_jump", false);
    d.failed_r90 = j.value("failed_r90", false);
    d.failed_passability = j.value("failed_passability", false);
    d.failed_residual = j.value("failed_residual", false);
    d.jump_in = j.value("jump_in", 0.0);
    d.radius_90_in = j.value("radius_90_in", 0.0);
    d.spread_in = j.value("spread_in", 0.0);
    d.reason = j.value("reason", std::string{});
    d.would_fail_centroid_jump = j.value("would_fail_centroid_jump", false);
    d.would_fail_r90 = j.value("would_fail_r90", false);
    d.would_fail_passability = j.value("would_fail_passability", false);
    d.would_fail_residual = j.value("would_fail_residual", false);
    d.max_sensor_residual_in = j.value("max_sensor_residual_in", 0.0);
    d.residual_threshold_in = j.value("residual_threshold_in", 0.0);
    d.centroid_jump_ft_per_s = j.value("centroid_jump_ft_per_s", 0.0);
    d.n_eff_at_gate = j.value("n_eff_at_gate", 0.0);
}

inline void to_json(nlohmann::json& j, const MCLTickResult& r) {
    j = nlohmann::json{
        { "tick_count", r.tick_count },
        { "raw_odom", r.raw_odom },
        { "raw_estimate", r.raw_estimate },
        { "chassis_pose", r.chassis_pose },
        { "observed_readings", r.observed_readings },
        { "mcl_sensor_residuals", r.mcl_sensor_residuals },
        { "mcl_predicted_readings", r.mcl_predicted_readings },
        { "gate", r.gate },
        { "valid_sensor_count", r.valid_sensor_count },
        { "update_skipped", r.update_skipped },
        { "cluster_stats", {
            { "spread", r.cluster_stats.spread },
            { "radius_90", r.cluster_stats.radius_90 },
            { "centroid", {
                { "x", r.cluster_stats.centroid.x },
                { "y", r.cluster_stats.centroid.y },
            } },
        } },
        { "n_eff", r.n_eff },
        { "post_predict", r.post_predict },
        { "post_update", r.post_update },
        { "post_resample", r.post_resample },
    };
}

inline void from_json(const nlohmann::json& j, MCLTickResult& r) {
    r.tick_count = j.value("tick_count", 0ULL);
    r.raw_odom = j.contains("raw_odom") ? j["raw_odom"].get<Pose>()
                 : j.value("odom_pose", Pose{});
    r.raw_estimate = j.value("raw_estimate", Pose{});
    r.chassis_pose = j.value("chassis_pose", Pose{});
    r.observed_readings = j.value("observed_readings", std::array<double, 4>{ -1.0, -1.0, -1.0, -1.0 });
    r.mcl_sensor_residuals = j.value("mcl_sensor_residuals", std::array<double, 4>{ 0.0, 0.0, 0.0, 0.0 });
    r.mcl_predicted_readings = j.value("mcl_predicted_readings", std::array<double, 4>{ -1.0, -1.0, -1.0, -1.0 });
    r.gate = j.value("gate", GateDecision{});
    r.valid_sensor_count = j.value("valid_sensor_count", 0);
    r.update_skipped = j.value("update_skipped", false);
    const auto& cs = j.contains("cluster_stats") ? j["cluster_stats"] : nlohmann::json::object();
    r.cluster_stats.spread = cs.value("spread", 0.0);
    r.cluster_stats.radius_90 = cs.value("radius_90", 0.0);
    const auto& centroid = cs.contains("centroid") ? cs["centroid"] : nlohmann::json::object();
    r.cluster_stats.centroid.x = centroid.value("x", 0.0f);
    r.cluster_stats.centroid.y = centroid.value("y", 0.0f);
    r.n_eff = j.value("n_eff", 0.0);
    r.post_predict = j.value("post_predict", PhaseSnapshot{});
    r.post_update = j.value("post_update", PhaseSnapshot{});
    r.post_resample = j.value("post_resample", PhaseSnapshot{});
}

class MCLController {
public:
    using LogFn = std::function<void(const std::string&)>;

    explicit MCLController(
        const MCLConfig& mcl_config = {},
        const GateConfig& gate_config = {},
        LogFn log_fn = nullptr);

    void initialize_uniform(uint64_t seed);
    void predict(double delta_forward, double delta_rotation, double heading_deg, double delta_lateral);
    void update(const double readings[4], double heading_deg);
    void resample();

    Estimate estimate() const;
    ClusterStats cluster_stats() const;
    double n_eff() const;
    const std::vector<Particle>& particles() const;
    MCLTickResult tick(
        double delta_forward,
        double delta_rotation,
        double heading_deg,
        double delta_lateral,
        const std::array<double, 4>& readings,
        int min_sensors_for_update,
        const sim::Field* field = nullptr,
        const Estimate* prev_accepted = nullptr,
        double dt_sec = 0.0,
        const GateEnables* gate_enables = nullptr,
        const Pose* odom_pose = nullptr);

    GateDecision gate_estimate(
        const sim::Field& field,
        const std::array<double, 4>& readings,
        double heading_deg,
        const Estimate& prev_estimate,
        double dt_sec,
        const GateEnables& gate_enables = {}) const;

    const MCLEngine& engine() const;
    const GateConfig& gate_config() const;

    void set_log_interval_ticks(int n) { log_interval_ticks_ = n; }
    int log_interval_ticks() const { return log_interval_ticks_; }

    /** Emit compact tick log for an externally-built MCLTickResult (used by LocalizationController). */
    void log_tick_result(const MCLTickResult& result, double heading_deg) const;

private:
    nlohmann::json snapshot_json() const;
    void emit_log(const char* phase, nlohmann::json extra,
                  const MCLTickResult* tick_result = nullptr, double heading_deg = 0.0) const;
    GateDecision fail_decision(const char* reason) const;
    void fill_snapshot(PhaseSnapshot& out) const;

    MCLEngine engine_;
    GateConfig gate_config_;
    LogFn log_fn_;
    uint64_t tick_count_ = 0;
    int log_interval_ticks_ = 4;
};

} // namespace mcl
