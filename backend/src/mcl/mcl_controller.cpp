#include "mcl/mcl_controller.hpp"

#include "ray/ray_cast_obstacles.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <ctime>
#include <iomanip>
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

std::string iso_timestamp_utc() {
    const auto now = std::chrono::system_clock::now();
    const auto secs = std::chrono::time_point_cast<std::chrono::seconds>(now);
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - secs).count();
    const std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm tm;
#if defined(_WIN32)
    gmtime_s(&tm, &t);
#else
    gmtime_r(&t, &tm);
#endif
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%dT%H:%M:%S")
        << '.' << std::setw(3) << std::setfill('0') << ms << 'Z';
    return oss.str();
}

// Compact tick printf: [T=N||short_key]=value, ~18 chars/line avg.
// Frontend expands short keys back to full TickState paths.
void emit_compact_tick(uint64_t tick,
                       const MCLTickResult& r,
                       double heading_deg) {
    const auto T = static_cast<unsigned long long>(tick);
    auto pf = [&](const char* key, const char* fmt, ...) __attribute__((format(printf, 3, 4))) {
        std::printf("[T=%llu||%s]=", T, key);
        va_list args;
        va_start(args, fmt);
        std::vprintf(fmt, args);
        va_end(args);
        std::putchar('\n');
        std::fflush(stdout);
    };
    auto pb = [&](const char* key, bool v) { pf(key, "%d", v ? 1 : 0); };

    pf("tc", "%llu", T);
    pf("oh", "%.3f", heading_deg);
    pb("pg", !r.gate.accepted);
    pf("ts", "%s", iso_timestamp_utc().c_str());
    pf("vs", "%d", r.valid_sensor_count);
    pb("us", r.update_skipped);
    pf("ne", "%.3f", r.n_eff);

    for (int i = 0; i < 4; ++i)
        pf(( std::string("or.") + std::to_string(i) ).c_str(), "%.3f", r.observed_readings[i]);

    pf("op.x", "%.3f", r.raw_odom.x);
    pf("op.y", "%.3f", r.raw_odom.y);
    pf("op.t", "%.3f", r.raw_odom.theta);

    pf("re.x", "%.3f", r.raw_estimate.x);
    pf("re.y", "%.3f", r.raw_estimate.y);
    pf("re.t", "%.3f", r.raw_estimate.theta);

    pf("cs.x", "%.3f", static_cast<double>(r.cluster_stats.centroid.x));
    pf("cs.y", "%.3f", static_cast<double>(r.cluster_stats.centroid.y));
    pf("cs.r", "%.3f", r.cluster_stats.radius_90);
    pf("cs.s", "%.3f", r.cluster_stats.spread);

    const auto& g = r.gate;
    pb("g.a", g.accepted);
    pf("g.rsn", "%s", g.reason.c_str());
    pf("g.r", "%.3f", g.radius_90_in);
    pf("g.j", "%.3f", g.jump_in);
    pf("g.s", "%.3f", g.spread_in);
    pb("g.fcj", g.failed_centroid_jump);
    pb("g.fr", g.failed_r90);
    pb("g.fp", g.failed_passability);
    pb("g.fres", g.failed_residual);
    pb("g.wcj", g.would_fail_centroid_jump);
    pb("g.wr", g.would_fail_r90);
    pb("g.wp", g.would_fail_passability);
    pb("g.wres", g.would_fail_residual);
    pf("g.msr", "%.3f", g.max_sensor_residual_in);
    pf("g.rt", "%.3f", g.residual_threshold_in);
    pf("g.cj", "%.3f", g.centroid_jump_ft_per_s);
    pf("g.ne", "%.3f", g.n_eff_at_gate);

    for (int i = 0; i < 4; ++i)
        pf(( std::string("pr.") + std::to_string(i) ).c_str(), "%.3f", r.mcl_predicted_readings[i]);
    for (int i = 0; i < 4; ++i)
        pf(( std::string("sr.") + std::to_string(i) ).c_str(), "%.3f", r.mcl_sensor_residuals[i]);

    auto ps = [&](const char* prefix, const PhaseSnapshot& s) {
        pf(( std::string(prefix) + ".x" ).c_str(), "%.3f", static_cast<double>(s.estimate.x));
        pf(( std::string(prefix) + ".y" ).c_str(), "%.3f", static_cast<double>(s.estimate.y));
        pf(( std::string(prefix) + ".n" ).c_str(), "%.3f", s.n_eff);
        pf(( std::string(prefix) + ".s" ).c_str(), "%.3f", s.spread);
        pf(( std::string(prefix) + ".r" ).c_str(), "%.3f", s.radius_90);
    };
    ps("pp", r.post_predict);
    ps("pu", r.post_update);
    ps("ps", r.post_resample);
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
        out.raw_odom = *odom_pose;
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

    nlohmann::json log_json(out);
    log_json["observed_heading"] = heading_deg;
    log_json["pose_gated"] = !out.gate.accepted;
    log_json["timestamp_iso"] = iso_timestamp_utc();
    emit_log("tick", std::move(log_json), &out, heading_deg);
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

void MCLController::emit_log(const char* phase, nlohmann::json extra,
                             const MCLTickResult* tick_result, double heading_deg) const {
    extra["phase"] = phase;
#ifndef NDEBUG_LOG
    if (std::string(phase) == "tick" && tick_result != nullptr &&
        (log_interval_ticks_ <= 1 || tick_count_ % static_cast<uint64_t>(log_interval_ticks_) == 0)) {
        emit_compact_tick(tick_count_, *tick_result, heading_deg);
    }
#endif
}

void MCLController::log_tick_result(const MCLTickResult& result, double heading_deg) const {
#ifndef NDEBUG_LOG
    if (log_interval_ticks_ <= 1
        || result.tick_count % static_cast<uint64_t>(log_interval_ticks_) == 0) {
        emit_compact_tick(result.tick_count, result, heading_deg);
    }
#else
    (void)result; (void)heading_deg;
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
