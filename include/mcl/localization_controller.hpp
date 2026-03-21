#pragma once

#include "distance_localization.hpp"
#include "mcl/mcl_controller.hpp"
#include "sim/field.hpp"

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace mcl {

enum class LocAlgorithm { MCL, RayWall };

struct Pose {
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
};

struct TickInput {
    distance_loc::DistanceReadings sensors{};
    struct OdomPose {
        double x = 0.0;
        double y = 0.0;
        double theta = 0.0;
    } odom_pose{};
    double imu_heading_deg = 0.0;
};

struct PhaseSnapshot {
    std::vector<Particle> particles;
    Estimate estimate{ 0.0f, 0.0f };
    double n_eff = 0.0;
    double spread = 0.0;
    double radius_90 = 0.0;
};

struct TickOutput {
    Pose accepted_pose{};
    Pose raw_estimate{};
    GateDecision gate{};
    std::string timestamp_iso;
    LocAlgorithm algorithm_used = LocAlgorithm::MCL;
    double correction_distance_in = 0.0;
    bool correction_applied = false;
    int valid_sensor_count = 0;
    bool update_skipped = false;
    ClusterStats cluster_stats{ 0.0, 0.0, Estimate{ 0.0f, 0.0f } };
    double n_eff = 0.0;
    distance_loc::DistanceLocResult raywall_result{};
    PhaseSnapshot post_predict{};
    PhaseSnapshot post_update{};
    PhaseSnapshot post_resample{};
};

struct ControllerConfig {
    LocAlgorithm algorithm = LocAlgorithm::MCL;

    // MCL settings
    MCLConfig mcl_config{};
    GateConfig gate_config{};
    GateEnables gate_enables{};
    MCLController::LogFn log_fn{};
    uint64_t seed = 42;

    // Ray-wall settings
    distance_loc::DistanceLocConfig raywall_loc_config{};
    distance_loc::SensorOffsets sensor_offsets{};
    double raywall_window_half = 6.0;
    double raywall_coarse_step = 0.5;
    double raywall_fine_step = 0.25;
    double raywall_max_error_in = 2.5;

    sim::Field field{};
    int min_sensors_for_update = 2;
    int32_t invalid_reading_mm = 9999;
    double tick_dt_sec = 0.05;

    // Legacy correction thresholds for application.
    double min_correction_delta_in = 0.5;
    double min_odom_raywall_error_in = 2.0;

    Pose initial_pose{};
};

class LocalizationController {
public:
    explicit LocalizationController(const ControllerConfig& config = {});

    TickOutput tick(const TickInput& input);

    Pose get_accepted_pose() const;
    Pose get_raw_estimate() const;
    void set_pose(const Pose& pose);
    void set_algorithm(LocAlgorithm algorithm);
    const ControllerConfig& config() const;

    const MCLController* mcl_controller() const;

private:
    TickOutput tick_mcl(const TickInput& input);
    TickOutput tick_raywall(const TickInput& input);

    static double wrap_heading(double deg);
    static double heading_delta(double cur_deg, double prev_deg);
    static std::string iso_timestamp_utc();
    static void fill_snapshot(const MCLController& ctrl, PhaseSnapshot& out);

    void compute_odom_deltas(
        const TickInput& input,
        double& delta_forward_in,
        double& delta_lateral_in,
        double& delta_rotation_deg
    );
    std::array<double, 4> readings_to_inches(const distance_loc::DistanceReadings& readings) const;
    int count_valid(const std::array<double, 4>& readings) const;

    ControllerConfig config_{};
    Pose accepted_pose_{};
    Pose raw_estimate_{};
    Pose prev_odom_pose_{};
    bool has_prev_odom_ = false;
    std::unique_ptr<MCLController> mcl_;
};

} // namespace mcl
