#include "mcl/localization_controller.hpp"

#include "distance_localization.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <sstream>

namespace mcl {

namespace {

constexpr double kMmPerIn = 25.4;

double euclidean(double x0, double y0, double x1, double y1) {
    const double dx = x0 - x1;
    const double dy = y0 - y1;
    return std::sqrt(dx * dx + dy * dy);
}

} // namespace

LocalizationController::LocalizationController(const ControllerConfig& config)
    : config_(config),
      accepted_pose_(config.initial_pose),
      raw_estimate_(config.initial_pose),
      prev_odom_pose_(config.initial_pose),
      mcl_(std::make_unique<MCLController>(config.replay_config, config.mcl_config, config.gate_config, config.log_fn)) {
    if (mcl_) {
        const bool has_known_pose =
            config.initial_pose.x != 0.0 || config.initial_pose.y != 0.0;
        if (has_known_pose) {
            mcl_->initialize_at_pose(
                config.seed,
                static_cast<float>(config.initial_pose.x),
                static_cast<float>(config.initial_pose.y));
        } else {
            mcl_->initialize_uniform(config.seed);
        }
    }
}

TickOutput LocalizationController::tick(const TickInput& input) {
    if (config_.algorithm == LocAlgorithm::RayWall) {
        return tick_raywall(input);
    }
    return tick_mcl(input);
}

Pose LocalizationController::get_accepted_pose() const {
    return accepted_pose_;
}

Pose LocalizationController::get_raw_estimate() const {
    return raw_estimate_;
}

void LocalizationController::set_pose(const Pose& pose) {
    accepted_pose_ = pose;
    raw_estimate_ = pose;
}

void LocalizationController::set_algorithm(LocAlgorithm algorithm) {
    config_.algorithm = algorithm;
}

const ControllerConfig& LocalizationController::config() const {
    return config_;
}

const MCLController* LocalizationController::mcl_controller() const {
    return mcl_.get();
}

double LocalizationController::wrap_heading(double deg) {
    if (!std::isfinite(deg)) return 0.0;
    deg = std::fmod(deg, 360.0);
    if (deg < 0.0) deg += 360.0;
    return deg;
}

double LocalizationController::heading_delta(double cur_deg, double prev_deg) {
    double d = wrap_heading(cur_deg) - wrap_heading(prev_deg);
    while (d > 180.0) d -= 360.0;
    while (d < -180.0) d += 360.0;
    return d;
}

std::string LocalizationController::iso_timestamp_utc() {
    const auto now = std::chrono::system_clock::now();
    const auto secs = std::chrono::time_point_cast<std::chrono::seconds>(now);
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - secs).count();
    const std::time_t t = std::chrono::system_clock::to_time_t(now);
#if defined(_WIN32)
    std::tm tm;
    gmtime_s(&tm, &t);
#else
    std::tm tm;
    gmtime_r(&t, &tm);
#endif
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%dT%H:%M:%S")
        << '.' << std::setw(3) << std::setfill('0') << ms << 'Z';
    return oss.str();
}

void LocalizationController::fill_snapshot(const MCLController& ctrl, PhaseSnapshot& out) {
    out.particles = ctrl.particles();
    const ClusterStats cs = ctrl.cluster_stats();
    out.estimate = cs.centroid;
    out.n_eff = ctrl.n_eff();
    out.spread = cs.spread;
    out.radius_90 = cs.radius_90;
}

void LocalizationController::compute_odom_deltas(
    const TickInput& input,
    double& delta_forward_in,
    double& delta_lateral_in,
    double& delta_rotation_deg) {
    if (!has_prev_odom_) {
        prev_odom_pose_.x = input.odom_pose.x;
        prev_odom_pose_.y = input.odom_pose.y;
        prev_odom_pose_.theta = input.odom_pose.theta;
        has_prev_odom_ = true;
        delta_forward_in = 0.0;
        delta_lateral_in = 0.0;
        delta_rotation_deg = 0.0;
        return;
    }

    const double dx = input.odom_pose.x - prev_odom_pose_.x;
    const double dy = input.odom_pose.y - prev_odom_pose_.y;
    const double h = distance_loc::deg2rad(prev_odom_pose_.theta);
    const double s = std::sin(h);
    const double c = std::cos(h);

    delta_forward_in = dx * s + dy * c;
    delta_lateral_in = dx * c - dy * s;
    delta_rotation_deg = heading_delta(input.odom_pose.theta, prev_odom_pose_.theta);

    prev_odom_pose_.x = input.odom_pose.x;
    prev_odom_pose_.y = input.odom_pose.y;
    prev_odom_pose_.theta = input.odom_pose.theta;
}

std::array<double, 4> LocalizationController::readings_to_inches(const distance_loc::DistanceReadings& readings) const {
    std::array<double, 4> out{ -1.0, -1.0, -1.0, -1.0 };
    const auto convert = [this](const std::optional<int32_t>& v) -> double {
        if (!v.has_value()) return -1.0;
        if (*v < 0 || *v >= config_.invalid_reading_mm) return -1.0;
        return static_cast<double>(*v) / kMmPerIn;
    };
    out[0] = convert(readings.left);
    out[1] = convert(readings.right);
    out[2] = convert(readings.front);
    out[3] = convert(readings.back);
    return out;
}

int LocalizationController::count_valid(const std::array<double, 4>& readings) const {
    int valid = 0;
    for (double r : readings) {
        if (r >= 0.0) valid++;
    }
    return valid;
}

TickOutput LocalizationController::tick_mcl(const TickInput& input) {
    TickOutput out;
    out.timestamp_iso = iso_timestamp_utc();
    out.algorithm_used = LocAlgorithm::MCL;

    double delta_forward = 0.0;
    double delta_lateral = 0.0;
    double delta_rotation = 0.0;
    compute_odom_deltas(input, delta_forward, delta_lateral, delta_rotation);

    const double heading_deg = wrap_heading(input.imu_heading_deg);
    const std::array<double, 4> readings_in = readings_to_inches(input.sensors);
    out.valid_sensor_count = count_valid(readings_in);

    mcl_->predict(delta_forward, delta_rotation, heading_deg, delta_lateral);
    fill_snapshot(*mcl_, out.post_predict);

    out.update_skipped = (out.valid_sensor_count < config_.min_sensors_for_update);
    if (!out.update_skipped) {
        mcl_->update(readings_in.data(), heading_deg);
    }
    fill_snapshot(*mcl_, out.post_update);

    mcl_->resample();
    fill_snapshot(*mcl_, out.post_resample);

    // Always propagate the controller pose with odom + IMU first.
    // MCL is then treated as a confidence-gated correction.
    {
        const double h = distance_loc::deg2rad(accepted_pose_.theta);
        const double s = std::sin(h);
        const double c = std::cos(h);
        accepted_pose_.x += delta_forward * s + delta_lateral * c;
        accepted_pose_.y += delta_forward * c - delta_lateral * s;
        accepted_pose_.theta = heading_deg;
    }

    const Estimate est = mcl_->estimate();
    raw_estimate_.x = est.x;
    raw_estimate_.y = est.y;
    raw_estimate_.theta = heading_deg;
    out.raw_estimate = raw_estimate_;
    out.cluster_stats = mcl_->cluster_stats();
    out.n_eff = mcl_->n_eff();

    const Estimate prev{ static_cast<float>(accepted_pose_.x), static_cast<float>(accepted_pose_.y) };
    out.gate = mcl_->gate_estimate(
        config_.field,
        readings_in,
        heading_deg,
        prev);

    out.correction_distance_in = euclidean(raw_estimate_.x, raw_estimate_.y, accepted_pose_.x, accepted_pose_.y);
    out.correction_applied = out.gate.accepted;
    if (out.gate.accepted) {
        accepted_pose_ = raw_estimate_;
    }
    out.accepted_pose = accepted_pose_;
    mcl_->record_tick(
        replay_tick_,
        config_.field,
        readings_in,
        heading_deg,
        out.valid_sensor_count,
        out.update_skipped,
        prev);
    replay_tick_++;
    return out;
}

TickOutput LocalizationController::tick_raywall(const TickInput& input) {
    TickOutput out;
    out.timestamp_iso = iso_timestamp_utc();
    out.algorithm_used = LocAlgorithm::RayWall;
    double delta_forward = 0.0;
    double delta_lateral = 0.0;
    double delta_rotation = 0.0;
    compute_odom_deltas(input, delta_forward, delta_lateral, delta_rotation);

    raw_estimate_.x = input.odom_pose.x;
    raw_estimate_.y = input.odom_pose.y;
    raw_estimate_.theta = wrap_heading(input.imu_heading_deg);
    out.raw_estimate = raw_estimate_;

    const std::array<double, 4> readings_in = readings_to_inches(input.sensors);
    out.valid_sensor_count = count_valid(readings_in);
    out.update_skipped = (out.valid_sensor_count < config_.min_sensors_for_update);

    if (out.update_skipped) {
        out.gate.accepted = false;
        out.gate.reason = "insufficient valid sensors";
        accepted_pose_.x = input.odom_pose.x;
        accepted_pose_.y = input.odom_pose.y;
        accepted_pose_.theta = wrap_heading(input.imu_heading_deg);
        out.accepted_pose = accepted_pose_;
        out.correction_distance_in = 0.0;
        return out;
    }

    out.raywall_result = distance_loc::computePositionRayCastLocal(
        input.sensors,
        static_cast<float>(wrap_heading(input.imu_heading_deg)),
        config_.sensor_offsets,
        config_.raywall_loc_config,
        input.odom_pose.x,
        input.odom_pose.y,
        config_.raywall_window_half,
        config_.raywall_coarse_step,
        config_.raywall_fine_step,
        config_.raywall_max_error_in);

    out.gate.accepted = (out.raywall_result.status == distance_loc::DistanceLocStatus::Good);
    if (!out.gate.accepted) {
        out.gate.reason = out.raywall_result.message.empty() ? "raywall invalid" : out.raywall_result.message;
        accepted_pose_.x = input.odom_pose.x;
        accepted_pose_.y = input.odom_pose.y;
        accepted_pose_.theta = wrap_heading(input.imu_heading_deg);
        out.accepted_pose = accepted_pose_;
        out.correction_distance_in = 0.0;
        return out;
    }

    raw_estimate_.x = out.raywall_result.x;
    raw_estimate_.y = out.raywall_result.y;
    raw_estimate_.theta = wrap_heading(input.imu_heading_deg);
    out.raw_estimate = raw_estimate_;

    const double dx = raw_estimate_.x - input.odom_pose.x;
    const double dy = raw_estimate_.y - input.odom_pose.y;
    const double err = std::sqrt(dx * dx + dy * dy);
    out.correction_distance_in = err;

    if (err > config_.min_odom_raywall_error_in &&
        (std::fabs(dx) > config_.min_correction_delta_in || std::fabs(dy) > config_.min_correction_delta_in)) {
        accepted_pose_ = raw_estimate_;
        out.correction_applied = true;
    } else {
        accepted_pose_.x = input.odom_pose.x;
        accepted_pose_.y = input.odom_pose.y;
        accepted_pose_.theta = wrap_heading(input.imu_heading_deg);
        out.correction_applied = false;
    }
    out.accepted_pose = accepted_pose_;

    return out;
}

} // namespace mcl
