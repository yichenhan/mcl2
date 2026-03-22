#include "state/sim_session.hpp"

#include "distance_localization.hpp"

#include <cmath>

namespace state {

SimSession::SimSession(const SimSessionConfig& config)
    : config_(config),
      physics_(config_.field, config_.physics_config),
      sensor_model_(config_.seed, config_.odom_noise_config, config_.sensor_noise_config),
      mcl_(config_.replay_config, config_.mcl_config, config_.mcl_gate_config) {
    physics_.set_state(config_.initial_state);
    odom_state_ = physics_.state();
    mcl_.initialize_uniform(config_.seed);
}

double SimSession::wrap_heading(double deg) {
    if (!std::isfinite(deg)) return 0.0;
    deg = std::fmod(deg, 360.0);
    if (deg < 0.0) deg += 360.0;
    return deg;
}

double SimSession::euclidean(double x0, double y0, double x1, double y1) {
    const double dx = x0 - x1;
    const double dy = y0 - y1;
    return std::sqrt(dx * dx + dy * dy);
}

std::array<double, 4> SimSession::read_sensors(const sim::RobotState& s) {
    std::array<double, 4> readings{ -1.0, -1.0, -1.0, -1.0 };
    const distance_loc::Vec2 robot_pos{ s.x, s.y };
    for (int i = 0; i < 4; ++i) {
        readings[static_cast<size_t>(i)] = sensor_model_.observe_distance_sensor(
            robot_pos,
            s.heading_deg,
            config_.mcl_config.sensors[i].offset,
            config_.mcl_config.sensors[i].angle_deg,
            i,
            config_.field.obstacles);
    }
    return readings;
}

MCLSnapshot SimSession::snapshot_from_mcl() const {
    MCLSnapshot snap;
    snap.particles = mcl_.particles();
    const auto cs = mcl_.cluster_stats();
    snap.estimate = cs.centroid;
    snap.n_eff = mcl_.n_eff();
    snap.spread = cs.spread;
    snap.radius_90 = cs.radius_90;
    return snap;
}

mcl::GateDecision SimSession::gate_estimate_for_control(
    const mcl::Estimate& prev_accepted,
    const TickState& tick) const {
    return mcl_.gate_estimate(
        config_.field,
        tick.observed_readings,
        tick.observed_heading,
        prev_accepted);
}

TickState SimSession::tick(sim::Action action) {
    return process_step(physics_.step(action));
}

TickState SimSession::tick(double linear_vel, double angular_vel_deg) {
    return process_step(physics_.step_continuous(linear_vel, angular_vel_deg));
}

TickState SimSession::process_step(const sim::StepResult& step) {
    auto kidnap_target = failure_injector_.pending_kidnap(tick_);
    if (kidnap_target.has_value()) {
        kidnap_target->heading_deg = physics_.state().heading_deg;
        physics_.set_state(*kidnap_target);
    }
    const sim::RobotState truth = physics_.state();

    sim::MotionDelta stalled = sensor_model_.apply_collision_stall(step.delta, step.colliding);
    sim::MotionDelta noisy_odom = sensor_model_.apply_odom_noise(stalled, truth.heading_deg);

    const double heading_rad = distance_loc::deg2rad(odom_state_.heading_deg);
    const double s = std::sin(heading_rad);
    const double c = std::cos(heading_rad);
    odom_state_.x += noisy_odom.forward_in * s + noisy_odom.lateral_in * c;
    odom_state_.y += noisy_odom.forward_in * c - noisy_odom.lateral_in * s;
    odom_state_.heading_deg = wrap_heading(odom_state_.heading_deg + noisy_odom.rotation_deg);
    config_.field.clamp(odom_state_.x, odom_state_.y);

    std::array<double, 4> readings = read_sensors(truth);
    double observed_heading = sensor_model_.observe_heading(truth.heading_deg);
    failure_injector_.apply(tick_, readings, noisy_odom, observed_heading);
    observed_heading = wrap_heading(observed_heading);

    mcl_.predict(noisy_odom.forward_in, noisy_odom.rotation_deg, observed_heading, noisy_odom.lateral_in);
    MCLSnapshot post_predict = snapshot_from_mcl();

    int valid_count = 0;
    for (double r : readings) {
        if (r >= 0.0) valid_count++;
    }

    const bool skip_update = (valid_count < config_.min_sensors_for_update);

    if (!skip_update) {
        mcl_.update(readings.data(), observed_heading);
    }
    MCLSnapshot post_update = snapshot_from_mcl();

    mcl_.resample();
    MCLSnapshot post_resample = snapshot_from_mcl();

    TickState out;
    out.tick = tick_;
    out.ground_truth = truth;
    out.observed_readings = readings;
    out.observed_heading = observed_heading;
    out.active_failures = failure_injector_.active_failures(tick_);
    out.post_predict = std::move(post_predict);
    out.post_update = std::move(post_update);
    out.post_resample = std::move(post_resample);
    out.mcl_error = euclidean(out.post_resample.estimate.x, out.post_resample.estimate.y, truth.x, truth.y);
    out.odom_error = euclidean(odom_state_.x, odom_state_.y, truth.x, truth.y);
    out.valid_sensor_count = valid_count;
    out.update_skipped = skip_update;
    out.pose_gated = (out.post_resample.radius_90 > config_.pose_gate_radius_90);
    const mcl::Estimate prev_accepted = has_replay_prev_accepted_ ? replay_prev_accepted_ : out.post_resample.estimate;
    const auto replay_gate = mcl_.record_tick(
        out.tick,
        config_.field,
        out.observed_readings,
        out.observed_heading,
        out.valid_sensor_count,
        out.update_skipped,
        prev_accepted);
    if (replay_gate.accepted || !has_replay_prev_accepted_) {
        replay_prev_accepted_ = out.post_resample.estimate;
        has_replay_prev_accepted_ = true;
    }

    history_.push_back(out);
    tick_++;
    return out;
}

const SimSessionConfig& SimSession::config() const {
    return config_;
}

const std::vector<TickState>& SimSession::history() const {
    return history_;
}

int SimSession::current_tick() const {
    return tick_;
}

void SimSession::schedule_failure(const noise::FailureEvent& event) {
    failure_injector_.schedule(event);
}

const noise::FailureInjector& SimSession::failure_injector() const {
    return failure_injector_;
}

mcl::MCLController& SimSession::mcl_controller() {
    return mcl_;
}

std::string action_to_string(sim::Action action) {
    switch (action) {
        case sim::Action::FORWARD: return "forward";
        case sim::Action::BACKWARD: return "backward";
        case sim::Action::ROTATE_CW: return "cw";
        case sim::Action::ROTATE_CCW: return "ccw";
        case sim::Action::NONE: return "none";
    }
    return "none";
}

bool action_from_string(const std::string& value, sim::Action& action) {
    if (value == "forward") {
        action = sim::Action::FORWARD;
        return true;
    }
    if (value == "backward") {
        action = sim::Action::BACKWARD;
        return true;
    }
    if (value == "cw") {
        action = sim::Action::ROTATE_CW;
        return true;
    }
    if (value == "ccw") {
        action = sim::Action::ROTATE_CCW;
        return true;
    }
    if (value == "none") {
        action = sim::Action::NONE;
        return true;
    }
    return false;
}

} // namespace state
