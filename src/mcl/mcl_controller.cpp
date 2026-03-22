#include "mcl/mcl_controller.hpp"

#include "ray/ray_cast_obstacles.hpp"

#include <cstdio>
#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace mcl {

namespace {

nlohmann::json estimate_to_json(const Estimate& est) {
    return nlohmann::json{
        { "x", est.x },
        { "y", est.y },
    };
}

nlohmann::json cluster_stats_to_json(const ClusterStats& stats) {
    return nlohmann::json{
        { "spread", stats.spread },
        { "radius_90", stats.radius_90 },
        { "centroid", estimate_to_json(stats.centroid) },
    };
}

nlohmann::json gate_to_json_impl(const GateDecision& gate) {
    return nlohmann::json{
        { "accepted", gate.accepted },
        { "failed_velocity", gate.failed_velocity },
        { "failed_spread", gate.failed_spread },
        { "failed_passability", gate.failed_passability },
        { "failed_residual", gate.failed_residual },
        { "failed_wall_sum", gate.failed_wall_sum },
        { "jump_in", gate.jump_in },
        { "radius_90_in", gate.radius_90_in },
        { "spread_in", gate.spread_in },
        { "reason", gate.reason },
    };
}

} // namespace

MCLController::MCLController(
    const ReplayConfig& replay_config,
    const MCLConfig& mcl_config,
    const GateConfig& gate_config,
    LogFn log_fn)
    : engine_(mcl_config),
      gate_config_(gate_config),
      replay_config_(replay_config),
      log_fn_(std::move(log_fn)) {
    if (!log_fn_) {
        log_fn_ = [](const std::string& msg) { std::cout << msg; };
    }
    if (replay_config_.directory.empty()) {
        throw std::invalid_argument("MCLController ReplayConfig.directory must be non-empty");
    }
    if (replay_config_.session_id.empty()) {
        replay_config_.session_id = default_session_id_central_time();
    }
}

MCLController::~MCLController() {
    if (!replay_ticks_.empty()) {
        (void)write_replay();
    }
}

void MCLController::initialize_uniform(uint64_t seed) {
    engine_.initialize_uniform(seed);
}

void MCLController::initialize_at_pose(uint64_t seed, float x, float y) {
    engine_.initialize_uniform(seed);
    auto& particles = const_cast<std::vector<Particle>&>(engine_.particles());
    if (particles.empty()) return;
    const float w = 1.0f / static_cast<float>(particles.size());
    for (auto& p : particles) {
        p.x = x;
        p.y = y;
        p.weight = w;
    }
}

void MCLController::predict(double delta_forward, double delta_rotation, double heading_deg, double delta_lateral) {
    engine_.predict(delta_forward, delta_rotation, heading_deg, delta_lateral);
    last_update_skipped_ = true;
    last_post_predict_ = current_snapshot_json();
    emit_log(
        "post_predict",
        nlohmann::json{
            { "raw_estimate", estimate_to_json(engine_.estimate()) },
            { "cluster_stats", cluster_stats_to_json(engine_.cluster_stats()) },
            { "n_eff", engine_.n_eff() },
            { "post_predict", last_post_predict_ },
        });
}

void MCLController::update(const double readings[4], double heading_deg) {
    for (int i = 0; i < 4; ++i) {
        last_observed_readings_[static_cast<size_t>(i)] = readings[i];
    }
    last_observed_heading_ = heading_deg;
    engine_.update(readings, heading_deg);
    int valid_sensor_count = 0;
    for (int i = 0; i < 4; ++i) {
        if (readings[i] >= 0.0) {
            valid_sensor_count++;
        }
    }
    last_valid_sensor_count_ = valid_sensor_count;
    last_update_skipped_ = false;
    last_post_update_ = current_snapshot_json();
    emit_log(
        "post_update",
        nlohmann::json{
            { "valid_sensor_count", valid_sensor_count },
            { "update_skipped", false },
            { "post_update", last_post_update_ },
        });
}

void MCLController::resample() {
    engine_.resample();
    last_post_resample_ = current_snapshot_json();
    emit_log(
        "post_resample",
        nlohmann::json{
            { "post_resample", last_post_resample_ },
            { "n_eff", engine_.n_eff() },
            { "cluster_stats", cluster_stats_to_json(engine_.cluster_stats()) },
        });
}

void MCLController::set_route_overlay(
    nlohmann::json waypoints,
    nlohmann::json obstacles,
    nlohmann::json route_config) {
    overlay_waypoints_ = std::move(waypoints);
    overlay_obstacles_ = std::move(obstacles);
    overlay_route_config_ = std::move(route_config);
}

void MCLController::set_flush_interval(int every_n_ticks) {
    flush_interval_ = every_n_ticks;
}

void MCLController::set_accepted_pose(double x, double y) {
    accepted_pose_x_ = x;
    accepted_pose_y_ = y;
    has_accepted_pose_ = true;
}

GateDecision MCLController::record_tick(
    int tick_index,
    const sim::Field& field,
    const std::array<double, 4>& observed_readings,
    double observed_heading,
    int valid_sensor_count,
    bool update_skipped,
    const Estimate& prev_accepted) {
    last_observed_readings_ = observed_readings;
    last_observed_heading_ = observed_heading;
    last_valid_sensor_count_ = valid_sensor_count;
    last_update_skipped_ = update_skipped;

    if (last_post_predict_.empty()) last_post_predict_ = current_snapshot_json();
    if (last_post_update_.empty()) last_post_update_ = current_snapshot_json();
    if (last_post_resample_.empty()) last_post_resample_ = current_snapshot_json();

    const GateDecision gate = gate_estimate(field, observed_readings, observed_heading, prev_accepted);
    nlohmann::json tick = nlohmann::json::object();
    tick["tick"] = tick_index;
    tick["observed_readings"] = observed_readings;
    tick["observed_heading"] = observed_heading;
    tick["post_predict"] = last_post_predict_;
    tick["post_update"] = last_post_update_;
    tick["post_resample"] = last_post_resample_;
    tick["valid_sensor_count"] = valid_sensor_count;
    tick["update_skipped"] = update_skipped;
    tick["pose_gated"] = !gate.accepted;
    tick["gate_decision"] = gate_to_json(gate);
    if (has_accepted_pose_) {
        tick["accepted_pose"] = { {"x", accepted_pose_x_}, {"y", accepted_pose_y_} };
    }
    replay_ticks_.push_back(std::move(tick));

    prev_tick_estimate_ = engine_.estimate();
    has_prev_tick_estimate_ = true;

    if (flush_interval_ > 0) {
        ++ticks_since_flush_;
        if (ticks_since_flush_ >= flush_interval_) {
            ticks_since_flush_ = 0;
            (void)write_replay();
        }
    }

    return gate;
}

bool MCLController::write_replay() {
    std::error_code ec;
    std::filesystem::create_directories(replay_config_.directory, ec);
    if (ec) return false;

    nlohmann::json config = nlohmann::json::object();
    config["num_particles"] = engine_.config().num_particles;
    config["sigma_sensor"] = engine_.config().sigma_sensor;
    config["max_sensor_error"] = engine_.config().max_sensor_error;
    config["random_injection"] = engine_.config().random_injection;
    config["resample_threshold"] = engine_.config().resample_threshold;
    config["roughening_sigma"] = engine_.config().roughening_sigma;
    config["bootstrap_recovery_ticks"] = engine_.config().bootstrap_recovery_ticks;
    config["lost_weight_threshold"] = engine_.config().lost_weight_threshold;
    config["lost_random_injection"] = engine_.config().lost_random_injection;
    config["field_half"] = engine_.config().field_half;
    config["predict_noise_fwd"] = engine_.config().predict_noise_fwd;
    config["predict_noise_lat"] = engine_.config().predict_noise_lat;
    config["outlier_penalty"] = engine_.config().outlier_penalty;
    config["bootstrap_resample_threshold"] = engine_.config().bootstrap_resample_threshold;
    config["weight_underflow_threshold"] = engine_.config().weight_underflow_threshold;
    config["min_motion_threshold"] = engine_.config().min_motion_threshold;
    config["gate"] = {
        { "max_inches_odom_delta_per_tick", gate_config_.max_inches_odom_delta_per_tick },
        { "max_radius_90_in", gate_config_.max_radius_90_in },
        { "max_spread_in", gate_config_.max_spread_in },
        { "max_sensor_residual_in", gate_config_.max_sensor_residual_in },
        { "wall_sum_tolerance_in", gate_config_.wall_sum_tolerance_in },
        { "min_valid_sensors_for_residual", gate_config_.min_valid_sensors_for_residual },
    };

    if (!overlay_route_config_.empty()) {
        for (auto& [key, val] : overlay_route_config_.items()) {
            if (!config.contains(key)) {
                config[key] = val;
            }
        }
    }
    if (!overlay_waypoints_.empty()) {
        config["waypoints"] = overlay_waypoints_;
    }

    nlohmann::json out = nlohmann::json::object();
    out["session_id"] = replay_config_.session_id;
    out["config"] = config;
    out["obstacles"] = overlay_obstacles_;
    out["total_ticks"] = replay_ticks_.size();
    out["ticks"] = replay_ticks_;

    const std::string final_path = replay_output_path();
    const std::string tmp_path = final_path + ".tmp";
    std::ofstream file(tmp_path, std::ios::trunc);
    if (!file.is_open()) return false;
    file << out.dump(2);
    file.close();
    if (!file) return false;

    if (std::rename(tmp_path.c_str(), final_path.c_str()) != 0) {
        std::remove(tmp_path.c_str());
        return false;
    }
    return true;
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

const ReplayConfig& MCLController::replay_config() const {
    return replay_config_;
}

std::string MCLController::replay_output_path() const {
    return replay_config_.directory + "/" + replay_config_.session_id + ".json";
}

nlohmann::json MCLController::snapshot_json() const {
    return current_snapshot_json();
}

nlohmann::json MCLController::current_snapshot_json() const {
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
    const std::string line = "[MCL_CONTROLLER_LOG] " + extra.dump() + " [MCL_CONTROLLER_LOG]\n";
    log_fn_(line);
}

GateDecision MCLController::fail_decision(const char* reason) const {
    GateDecision d;
    d.accepted = false;
    d.reason = reason;
    return d;
}

std::string MCLController::default_session_id_central_time() {
    using namespace std::chrono;
    const auto now = system_clock::now() - hours(6);
    const std::time_t tt = system_clock::to_time_t(now);

    std::tm tm{};
#if defined(_WIN32)
    gmtime_s(&tm, &tt);
#else
    gmtime_r(&tt, &tm);
#endif

    static const char* kMonths[] = {
        "January", "February", "March", "April", "May", "June",
        "July", "August", "September", "October", "November", "December"
    };

    int hour = tm.tm_hour;
    const char* am_pm = (hour >= 12) ? "PM" : "AM";
    hour %= 12;
    if (hour == 0) hour = 12;

    std::ostringstream oss;
    oss << kMonths[tm.tm_mon] << ' ' << tm.tm_mday << ": " << hour << ':'
        << std::setw(2) << std::setfill('0') << tm.tm_min << ' '
        << am_pm << " Central Time";
    return oss.str();
}

nlohmann::json MCLController::gate_to_json(const GateDecision& gate) {
    return gate_to_json_impl(gate);
}

bool MCLController::wall_sum_ok(
    const std::array<double, 4>& readings,
    double heading_deg,
    const sim::Field& field) const {
    if (gate_config_.wall_sum_tolerance_in <= 0.0) return true;
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
    const Estimate& prev_accepted) const {
    GateDecision d;
    const Estimate est = engine_.estimate();
    const ClusterStats cs = engine_.cluster_stats();
    d.radius_90_in = cs.radius_90;
    d.spread_in = cs.spread;

    const Estimate& vel_ref = has_prev_tick_estimate_ ? prev_tick_estimate_ : prev_accepted;
    const double dx = static_cast<double>(est.x) - static_cast<double>(vel_ref.x);
    const double dy = static_cast<double>(est.y) - static_cast<double>(vel_ref.y);
    d.jump_in = std::sqrt(dx * dx + dy * dy);
    if (gate_config_.max_inches_odom_delta_per_tick > 0.0
        && d.jump_in > gate_config_.max_inches_odom_delta_per_tick) {
        d.accepted = false;
        d.failed_velocity = true;
        d.reason = "velocity gate";
        emit_log("gate", nlohmann::json{ { "gate", gate_to_json(d) } });
        return d;
    }

    if (cs.radius_90 > gate_config_.max_radius_90_in || cs.spread > gate_config_.max_spread_in) {
        d.accepted = false;
        d.failed_spread = true;
        d.reason = "spread gate";
        emit_log("gate", nlohmann::json{ { "gate", gate_to_json(d) } });
        return d;
    }

    if (!field.is_passable(est.x, est.y)) {
        d.accepted = false;
        d.failed_passability = true;
        d.reason = "passability gate";
        emit_log("gate", nlohmann::json{ { "gate", gate_to_json(d) } });
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
            emit_log("gate", nlohmann::json{ { "gate", gate_to_json(d) } });
            return d;
        }
    }
    if (valid_count < gate_config_.min_valid_sensors_for_residual) {
        d.accepted = false;
        d.failed_residual = true;
        d.reason = "insufficient valid sensors";
        emit_log("gate", nlohmann::json{ { "gate", gate_to_json(d) } });
        return d;
    }

    if (!wall_sum_ok(readings, heading_deg, field)) {
        d.accepted = false;
        d.failed_wall_sum = true;
        d.reason = "wall-sum gate";
        emit_log("gate", nlohmann::json{ { "gate", gate_to_json(d) } });
        return d;
    }
    emit_log("gate", nlohmann::json{ { "gate", gate_to_json(d) } });
    return d;
}

} // namespace mcl
