#include "sim/sim_harness.hpp"

#include "distance_localization.hpp"

#include <cmath>

namespace sim {

namespace {

constexpr double kInToMm = 25.4;

} // namespace

SimHarness::SimHarness(const Config& config)
    : config_(config),
      physics_(config_.controller_config.field, config_.physics_config),
      sensor_model_(config_.controller_config.seed, config_.odom_noise, config_.sensor_noise),
      controller_([&]() {
          Config cfg = config_;
          cfg.controller_config.tick_dt_sec = cfg.physics_config.dt;
          cfg.controller_config.initial_pose = {
              cfg.initial_state.x,
              cfg.initial_state.y,
              cfg.initial_state.heading_deg
          };
          return cfg.controller_config;
      }()) {
    physics_.set_state(config_.initial_state);
    odom_state_ = physics_.state();
}

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
    out.left = to_mm(readings[0]);
    out.right = to_mm(readings[1]);
    out.front = to_mm(readings[2]);
    out.back = to_mm(readings[3]);
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

std::array<double, 4> SimHarness::read_sensors(const RobotState& state) {
    std::array<double, 4> readings{ -1.0, -1.0, -1.0, -1.0 };
    const distance_loc::Vec2 robot_pos{ state.x, state.y };
    for (int i = 0; i < 4; ++i) {
        readings[static_cast<size_t>(i)] = sensor_model_.observe_distance_sensor(
            robot_pos,
            state.heading_deg,
            config_.controller_config.mcl_config.sensors[i].offset,
            config_.controller_config.mcl_config.sensors[i].angle_deg,
            i,
            config_.controller_config.field.obstacles);
    }
    return readings;
}

state::TickState SimHarness::tick(Action action) {
    return process_step(physics_.step(action));
}

state::TickState SimHarness::tick(double linear_vel, double angular_vel_deg) {
    return process_step(physics_.step_continuous(linear_vel, angular_vel_deg));
}

state::TickState SimHarness::process_step(const StepResult& step) {
    auto kidnap_target = failure_injector_.pending_kidnap(tick_);
    if (kidnap_target.has_value()) {
        kidnap_target->heading_deg = physics_.state().heading_deg;
        physics_.set_state(*kidnap_target);
    }
    const RobotState truth = physics_.state();

    MotionDelta stalled = sensor_model_.apply_collision_stall(step.delta, step.colliding);
    MotionDelta noisy_odom = sensor_model_.apply_odom_noise(stalled, truth.heading_deg);

    const double heading_rad = distance_loc::deg2rad(odom_state_.heading_deg);
    const double s = std::sin(heading_rad);
    const double c = std::cos(heading_rad);
    odom_state_.x += noisy_odom.forward_in * s + noisy_odom.lateral_in * c;
    odom_state_.y += noisy_odom.forward_in * c - noisy_odom.lateral_in * s;
    odom_state_.heading_deg = wrap_heading(odom_state_.heading_deg + noisy_odom.rotation_deg);
    config_.controller_config.field.clamp(odom_state_.x, odom_state_.y);

    std::array<double, 4> readings = read_sensors(truth);
    double observed_heading = sensor_model_.observe_heading(truth.heading_deg);
    failure_injector_.apply(tick_, readings, noisy_odom, observed_heading);
    observed_heading = wrap_heading(observed_heading);

    mcl::TickInput input;
    input.sensors = readings_to_mm(readings);
    input.odom_pose = { odom_state_.x, odom_state_.y, odom_state_.heading_deg };
    input.imu_heading_deg = observed_heading;
    const mcl::TickOutput controller_out = controller_.tick(input);

    state::TickState out;
    out.tick = tick_;
    out.ground_truth = truth;
    out.observed_readings = readings;
    out.observed_heading = observed_heading;
    out.active_failures = failure_injector_.active_failures(tick_);
    out.post_predict = to_state_snapshot(controller_out.post_predict);
    out.post_update = to_state_snapshot(controller_out.post_update);
    out.post_resample = to_state_snapshot(controller_out.post_resample);
    out.mcl_error = euclidean(controller_out.raw_estimate.x, controller_out.raw_estimate.y, truth.x, truth.y);
    out.odom_error = euclidean(odom_state_.x, odom_state_.y, truth.x, truth.y);
    out.valid_sensor_count = controller_out.valid_sensor_count;
    out.update_skipped = controller_out.update_skipped;
    out.pose_gated = !controller_out.gate.accepted;
    out.timestamp_iso = controller_out.timestamp_iso;
    out.raw_estimate = { static_cast<float>(controller_out.raw_estimate.x), static_cast<float>(controller_out.raw_estimate.y) };
    out.accepted_estimate = { static_cast<float>(controller_out.accepted_pose.x), static_cast<float>(controller_out.accepted_pose.y) };
    out.gate_decision = controller_out.gate;

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
    failure_injector_.schedule(event);
}

const noise::FailureInjector& SimHarness::failure_injector() const {
    return failure_injector_;
}

mcl::GateDecision SimHarness::gate_estimate_for_control(
    const mcl::Estimate& prev_accepted,
    const state::TickState& tick) const {
    const mcl::MCLController* ctrl = controller_.mcl_controller();
    if (!ctrl) {
        mcl::GateDecision d;
        d.accepted = true;
        return d;
    }
    return ctrl->gate_estimate(
        config_.controller_config.field,
        tick.observed_readings,
        tick.observed_heading,
        prev_accepted,
        config_.physics_config.dt,
        config_.controller_config.gate_enables);
}

mcl::LocalizationController& SimHarness::controller() {
    return controller_;
}

const mcl::LocalizationController& SimHarness::controller() const {
    return controller_;
}

} // namespace sim
