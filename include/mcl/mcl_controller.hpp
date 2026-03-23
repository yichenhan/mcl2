#pragma once

#include "mcl/mcl_engine.hpp"
#include "sim/field.hpp"
#include "nlohmann/json.hpp"

#include <array>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace mcl {

struct GateConfig {
    double max_estimate_speed_ft_per_s = 360.0 / 12.0;
    double max_jump_in = 12.0;
    double max_radius_90_in = 7.0;
    double max_sensor_residual_in = 6.0;
    // Distance-dependent residual thresholds (100% tolerance over sensor spec).
    // < 200mm: spec ±15mm → ×2 = 30mm = 1.181 in.
    // ≥ 200mm: spec 5%   → ×2 = 10% of reading.
    double sensor_close_range_in = 7.874;
    double sensor_close_tolerance_in = 1.181;
    double sensor_far_tolerance_pct = 0.10;
    double wall_sum_tolerance_in = 8.0;
    int min_valid_sensors_for_residual = 2;
};

struct GateEnables {
    bool velocity = true;
    bool r90 = true;
    bool passability = true;
    bool residual = true;
    bool wall_sum = true;
};

struct GateDecision {
    bool accepted = true;
    bool failed_velocity = false;
    bool failed_r90 = false;
    bool failed_passability = false;
    bool failed_residual = false;
    bool failed_wall_sum = false;
    double jump_in = 0.0;
    double radius_90_in = 0.0;
    double spread_in = 0.0;
    std::string reason;
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
    Pose odom_pose{};
    Pose raw_estimate{};
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
        { "failed_velocity", d.failed_velocity },
        { "failed_r90", d.failed_r90 },
        { "failed_passability", d.failed_passability },
        { "failed_residual", d.failed_residual },
        { "failed_wall_sum", d.failed_wall_sum },
        { "jump_in", d.jump_in },
        { "radius_90_in", d.radius_90_in },
        { "spread_in", d.spread_in },
        { "reason", d.reason },
    };
}

inline void from_json(const nlohmann::json& j, GateDecision& d) {
    d.accepted = j.value("accepted", true);
    d.failed_velocity = j.value("failed_velocity", false);
    d.failed_r90 = j.value("failed_r90", false);
    d.failed_passability = j.value("failed_passability", false);
    d.failed_residual = j.value("failed_residual", false);
    d.failed_wall_sum = j.value("failed_wall_sum", false);
    d.jump_in = j.value("jump_in", 0.0);
    d.radius_90_in = j.value("radius_90_in", 0.0);
    d.spread_in = j.value("spread_in", 0.0);
    d.reason = j.value("reason", std::string{});
}

inline void to_json(nlohmann::json& j, const MCLTickResult& r) {
    j = nlohmann::json{
        { "tick_count", r.tick_count },
        { "odom_pose", r.odom_pose },
        { "raw_estimate", r.raw_estimate },
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
    r.odom_pose = j.value("odom_pose", Pose{});
    r.raw_estimate = j.value("raw_estimate", Pose{});
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
        const Estimate& prev_accepted,
        double dt_sec,
        const GateEnables& gate_enables = {}) const;

    const MCLEngine& engine() const;
    const GateConfig& gate_config() const;

private:
    nlohmann::json snapshot_json() const;
    void emit_log(const char* phase, nlohmann::json extra) const;
    GateDecision fail_decision(const char* reason) const;
    bool wall_sum_ok(
        const std::array<double, 4>& readings,
        double heading_deg,
        const sim::Field& field) const;
    void fill_snapshot(PhaseSnapshot& out) const;

    MCLEngine engine_;
    GateConfig gate_config_;
    LogFn log_fn_;
    uint64_t tick_count_ = 0;
};

} // namespace mcl
