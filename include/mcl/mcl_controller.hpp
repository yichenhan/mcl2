#pragma once

#include "mcl/mcl_engine.hpp"
#include "sim/field.hpp"
#include "nlohmann/json.hpp"

#include <array>
#include <functional>
#include <string>

namespace mcl {

struct GateConfig {
    double max_estimate_speed_ft_per_s = 360.0 / 12.0;
    double max_jump_in = 12.0;
    double max_radius_90_in = 5.0;
    double max_spread_in = 25.0;
    double max_sensor_residual_in = 6.0;
    double wall_sum_tolerance_in = 8.0;
    int min_valid_sensors_for_residual = 2;
};

struct GateEnables {
    bool velocity = true;
    bool spread = true;
    bool passability = true;
    bool residual = true;
    bool wall_sum = true;
};

struct GateDecision {
    bool accepted = true;
    bool failed_velocity = false;
    bool failed_spread = false;
    bool failed_passability = false;
    bool failed_residual = false;
    bool failed_wall_sum = false;
    double jump_in = 0.0;
    double radius_90_in = 0.0;
    double spread_in = 0.0;
    std::string reason;
};

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

    MCLEngine engine_;
    GateConfig gate_config_;
    LogFn log_fn_;
};

} // namespace mcl
