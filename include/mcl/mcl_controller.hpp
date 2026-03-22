#pragma once

#include "mcl/mcl_engine.hpp"
#include "sim/field.hpp"
#include "nlohmann/json.hpp"

#include <array>
#include <functional>
#include <string>

namespace mcl {

struct ReplayConfig {
    std::string directory = "replays";
    std::string session_id;
};

struct GateConfig {
    double max_inches_odom_delta_per_tick = 0.0;
    double max_radius_90_in = 20.0;
    double max_spread_in = 25.0;
    double max_sensor_residual_in = 6.0;
    double wall_sum_tolerance_in = 0.0;
    int min_valid_sensors_for_residual = 2;
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

using GateEnables = std::array<bool, 5>;

class MCLController {
public:
    using LogFn = std::function<void(const std::string&)>;

    explicit MCLController(
        const ReplayConfig& replay_config,
        const MCLConfig& mcl_config = {},
        const GateConfig& gate_config = {},
        LogFn log_fn = nullptr);
    ~MCLController();

    void initialize_uniform(uint64_t seed);
    void initialize_at_pose(uint64_t seed, float x, float y);
    void predict(double delta_forward, double delta_rotation, double heading_deg, double delta_lateral);
    void update(const double readings[4], double heading_deg);
    void resample();
    void set_route_overlay(
        nlohmann::json waypoints,
        nlohmann::json obstacles,
        nlohmann::json route_config = nlohmann::json::object());

    GateDecision record_tick(
        int tick_index,
        const sim::Field& field,
        const std::array<double, 4>& observed_readings,
        double observed_heading,
        int valid_sensor_count,
        bool update_skipped,
        const Estimate& prev_accepted);
    bool write_replay();
    void set_flush_interval(int every_n_ticks);
    void set_accepted_pose(double x, double y);

    Estimate estimate() const;
    ClusterStats cluster_stats() const;
    double n_eff() const;
    const std::vector<Particle>& particles() const;

    GateDecision gate_estimate(
        const sim::Field& field,
        const std::array<double, 4>& readings,
        double heading_deg,
        const Estimate& prev_accepted) const;

    const MCLEngine& engine() const;
    const GateConfig& gate_config() const;
    const ReplayConfig& replay_config() const;
    std::string replay_output_path() const;

private:
    nlohmann::json snapshot_json() const;
    nlohmann::json current_snapshot_json() const;
    void emit_log(const char* phase, nlohmann::json extra) const;
    GateDecision fail_decision(const char* reason) const;
    static std::string default_session_id_central_time();
    static nlohmann::json gate_to_json(const GateDecision& gate);
    bool wall_sum_ok(
        const std::array<double, 4>& readings,
        double heading_deg,
        const sim::Field& field) const;

    MCLEngine engine_;
    GateConfig gate_config_;
    ReplayConfig replay_config_;
    LogFn log_fn_;
    nlohmann::json replay_ticks_ = nlohmann::json::array();
    nlohmann::json last_post_predict_ = nlohmann::json::object();
    nlohmann::json last_post_update_ = nlohmann::json::object();
    nlohmann::json last_post_resample_ = nlohmann::json::object();
    std::array<double, 4> last_observed_readings_{ -1.0, -1.0, -1.0, -1.0 };
    double last_observed_heading_ = 0.0;
    int last_valid_sensor_count_ = 0;
    bool last_update_skipped_ = true;

    nlohmann::json overlay_waypoints_ = nlohmann::json::array();
    nlohmann::json overlay_obstacles_ = nlohmann::json::array();
    nlohmann::json overlay_route_config_ = nlohmann::json::object();
    int flush_interval_ = 0;
    int ticks_since_flush_ = 0;
    double accepted_pose_x_ = 0.0;
    double accepted_pose_y_ = 0.0;
    bool has_accepted_pose_ = false;
    Estimate prev_tick_estimate_{ 0.0f, 0.0f };
    bool has_prev_tick_estimate_ = false;
};

} // namespace mcl
