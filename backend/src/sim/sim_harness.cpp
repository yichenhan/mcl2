#include "sim/sim_harness.hpp"

#include "distance_localization.hpp"

#include <cmath>

namespace sim {

namespace {

constexpr double kInToMm = 25.4;

// Build the ControllerConfig for PCC, stamping tick_dt from physics_config.
mcl::ControllerConfig make_pcc_config(const SimHarness::Config& cfg) {
    mcl::ControllerConfig cc = cfg.controller_config;
    cc.tick_dt_sec = cfg.physics_config.dt;
    cc.initial_pose = {
        cfg.initial_state.x,
        cfg.initial_state.y,
        cfg.initial_state.heading_deg
    };
    return cc;
}

} // namespace

SimHarness::SimHarness(const Config& config)
    : config_(config),
      sensor_model_(config_.controller_config.seed,
                    config_.odom_noise,
                    config_.sensor_noise),
      chassis_(config_.controller_config.field,
               config_.physics_config,
               config_.initial_state),
      sensor_provider_(chassis_,
                       config_.controller_config.seed + 1,
                       config_.sensor_noise,
                       config_.controller_config.mcl_config,
                       config_.controller_config.field.obstacles),
      correction_controller_(chassis_,
                              sensor_provider_,
                              make_pcc_config(config_)) {}

double SimHarness::wrap_heading(double deg) {
    if (!std::isfinite(deg)) return 0.0;
    deg = std::fmod(deg, 360.0);
    if (deg < 0.0) deg += 360.0;
    return deg;
}

double SimHarness::euclidean(double x0, double y0, double x1, double y1) {
    const double dx = x0 - x1;
    const double dy = y0 - y1;
    return std::sqrt(dx * dx + dy * dy);
}

distance_loc::DistanceReadings SimHarness::readings_to_mm(const std::array<double, 4>& readings) {
    distance_loc::DistanceReadings out;
    const auto to_mm = [](double v) -> std::optional<int32_t> {
        if (v < 0.0 || !std::isfinite(v)) return std::nullopt;
        return static_cast<int32_t>(std::lround(v * kInToMm));
    };
    out.left  = to_mm(readings[0]);
    out.right = to_mm(readings[1]);
    out.front = to_mm(readings[2]);
    out.back  = to_mm(readings[3]);
    return out;
}

state::MCLSnapshot SimHarness::to_state_snapshot(const mcl::PhaseSnapshot& snap) {
    state::MCLSnapshot out;
    out.particles = snap.particles;
    out.estimate = snap.estimate;
    out.n_eff = snap.n_eff;
    out.spread = snap.spread;
    out.radius_90 = snap.radius_90;
    return out;
}

state::TickState SimHarness::tick(Action action) {
    return process_step(chassis_.step(action));
}

state::TickState SimHarness::tick(double linear_vel, double angular_vel_deg) {
    return process_step(chassis_.step(linear_vel, angular_vel_deg));
}

state::TickState SimHarness::process_step(const StepResult& step) {
    // --- 1. Kidnap check (teleport physics, odom tracks new position) ---
    auto kidnap_target = sensor_provider_.failure_injector().pending_kidnap(tick_);
    if (kidnap_target.has_value()) {
        kidnap_target->heading_deg = chassis_.ground_truth().heading_deg;
        chassis_.teleport(*kidnap_target);
    }

    // --- 2. Odom noise pipeline: stall + noise ---
    const RobotState truth_before_noise = chassis_.ground_truth();
    MotionDelta stalled = sensor_model_.apply_collision_stall(step.delta, step.colliding);
    MotionDelta noisy   = sensor_model_.apply_odom_noise(stalled, truth_before_noise.heading_deg);
    double heading      = sensor_model_.observe_heading(truth_before_noise.heading_deg);

    // --- 3. Odom failures BEFORE integration (FIX: spike/bias applied here) ---
    sensor_provider_.failure_injector().applyOdom(tick_, noisy, heading);
    heading = wrap_heading(heading);

    // --- 4. Integrate modified delta into chassis odom ---
    chassis_.applyOdomNoise(noisy, heading);

    // --- 5. Advance sensor provider tick counter ---
    sensor_provider_.advanceTick();

    // --- 6. MCL correction (internally reads sensors via getReadings()) ---
    const mcl::CorrectionResult result = correction_controller_.update();

    // --- 7. Build TickState ---
    const RobotState truth = chassis_.ground_truth();

    state::TickState out;
    out.tick            = tick_;
    out.ground_truth    = truth;
    out.chassis_pose    = result.chassis_pose;
    out.observed_heading = heading;
    out.active_failures = sensor_provider_.failure_injector().active_failures(tick_);

    // MCL phase snapshots from correction result
    const mcl::TickOutput& mcl = result.mcl_output;
    out.post_predict   = to_state_snapshot(mcl.post_predict);
    out.post_update    = to_state_snapshot(mcl.post_update);
    out.post_resample  = to_state_snapshot(mcl.post_resample);

    // Error metrics
    out.mcl_error      = euclidean(mcl.raw_estimate.x, mcl.raw_estimate.y, truth.x, truth.y);
    out.odom_error     = euclidean(result.raw_odom.x, result.raw_odom.y, truth.x, truth.y);
    out.accepted_error = euclidean(result.chassis_pose.x, result.chassis_pose.y, truth.x, truth.y);

    // Pose snapshots
    out.raw_odom = result.raw_odom;
    out.raw_estimate = {
        static_cast<float>(mcl.raw_estimate.x),
        static_cast<float>(mcl.raw_estimate.y)
    };
    out.accepted_estimate = {
        static_cast<float>(result.chassis_pose.x),
        static_cast<float>(result.chassis_pose.y)
    };

    // Gate + MCL diagnostics
    out.gate_decision      = mcl.gate;
    out.pose_gated         = !mcl.gate.accepted;
    out.valid_sensor_count = mcl.valid_sensor_count;
    out.update_skipped     = mcl.update_skipped;
    out.timestamp_iso      = mcl.timestamp_iso;
    out.cluster_stats      = mcl.cluster_stats;

    // Sensor residuals and predicted readings
    for (size_t i = 0; i < 4; ++i) {
        out.mcl_predicted_readings[i] = mcl.mcl_predicted_readings[i];
        out.sensor_residuals[i]       = mcl.mcl_sensor_residuals[i];
    }

    // Observed sensor readings (raw inches, from odom state used by MCL)
    // Reconstruct from mcl output observed_readings
    for (size_t i = 0; i < 4; ++i) {
        out.observed_readings[i] = mcl.observed_readings[i];
    }

    history_.push_back(out);
    tick_++;
    return out;
}

const SimHarness::Config& SimHarness::config() const {
    return config_;
}

const std::vector<state::TickState>& SimHarness::history() const {
    return history_;
}

int SimHarness::current_tick() const {
    return tick_;
}

void SimHarness::schedule_failure(const noise::FailureEvent& event) {
    sensor_provider_.schedule_failure(event);
}

const noise::FailureInjector& SimHarness::failure_injector() const {
    return sensor_provider_.failure_injector();
}

mcl::GateDecision SimHarness::gate_estimate_for_control(
    const mcl::Estimate& prev_accepted,
    const state::TickState& tick_state) const {
    const mcl::MCLController* ctrl = correction_controller_.controller().mcl_controller();
    if (!ctrl) {
        mcl::GateDecision d;
        d.accepted = true;
        return d;
    }
    return ctrl->gate_estimate(
        config_.controller_config.field,
        tick_state.observed_readings,
        tick_state.observed_heading,
        prev_accepted,
        config_.physics_config.dt,
        config_.controller_config.gate_enables);
}

mcl::LocalizationController& SimHarness::controller() {
    return correction_controller_.controller();
}

const mcl::LocalizationController& SimHarness::controller() const {
    return correction_controller_.controller();
}

} // namespace sim
