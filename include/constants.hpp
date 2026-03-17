#pragma once

#include <cstdint>

namespace constants {

// Hardware port mapping
struct Ports {
    int left_drive_m1_port;
    int left_drive_m2_port;
    int left_drive_m3_port;
    int right_drive_m1_port;
    int right_drive_m2_port;
    int right_drive_m3_port;

    int intake1_motor_port;
    int intake2_motor_port;

    int imu_port;
    int rotation_port;

    int distance_right_port;
    int distance_left_port;
    int distance_front_port;
    int distance_back_port;

    char matchloader_adi_port;
    char middle_scoring_1_adi_port;
    char trapdoor_adi_port;
    char wing_adi_port;
    char mdescoreLfunnel_adi_port;
};

struct DriverCurves {
    int joystick_deadband;
    int min_output;
    double expo_gain;
};

struct DrivetrainConstants {
    double track_width_in;
    double wheel_diameter_in;
    double drivetrain_rpm;
    double horizontal_drift;
};

struct LemlibControllers {
    double lat_kP;
    double lat_kI;
    double lat_kD;
    double lat_anti_windup;
    double lat_small_err_in;
    double lat_small_err_timeout_ms;
    double lat_large_err_in;
    double lat_large_err_timeout_ms;
    double lat_slew;

    double ang_kP;
    double ang_kI;
    double ang_kD;
    double ang_anti_windup;
    double ang_small_err_deg;
    double ang_small_err_timeout_ms;
    double ang_large_err_deg;
    double ang_large_err_timeout_ms;
    double ang_slew;
};

struct TrackingWheelConstants {
    double wheel_diameter_in;
    double y_offset_in;
    double gear_ratio;
};

struct DistanceLocalizationConstants {
    float left_x_in;
    float left_y_in;
    float right_x_in;
    float right_y_in;
    float front_x_in;
    float front_y_in;
    float back_x_in;
    float back_y_in;

    float max_velocity_in_per_s;
    float max_jump_in;
    float wall_sum_tolerance_in;
    int32_t invalid_reading_mm;

    float field_size_in;
    float mm_to_inches;

    int debug_log_every_n;
};

struct RayWallConstants {
    double field_half_in;
    double eps;

    double sensor_angle_front_deg;
    double sensor_angle_right_deg;
    double sensor_angle_back_deg;
    double sensor_angle_left_deg;

    double max_per_sensor_err_in;

    double global_grid_step_in;
    double local_window_half_in;
    double local_coarse_step_in;
    double local_fine_step_in;
    double local_max_error_in;
    double local_max_candidate_spread_in;
    double wall_sum_tolerance_in;

    int global_log_every_n;
    int local_log_every_n;
};

struct LocalizationTaskConstants {
    int dist_loc_task_delay_ms;
    int heading_comp_task_delay_ms;
    int pose_log_task_delay_ms;
    int min_valid_sensors_for_correction;

    float heading_comp_k_cw;
    float heading_comp_k_ccw;

    float imu_heading_offset_deg;

    float angle_equal_eps_deg;
    float min_correction_delta_in;
    float min_odom_raywall_error_in;

    float corner_distance_tolerance_in;

    float max_wall_incidence_deg;
};

struct MotionConstants {
    double pi;
    double drive_wheel_diameter_in;
    double motor_cartridge_rpm;
    double drivetrain_rpm;
};

struct KalmanClampConstants {
    double max_dt_s;
    double min_innov_cov;
    double min_weight;
    double min_R;
    double max_R;
    double min_P_diag;
    int debug_log_every_n;
};

struct KalmanTuningConstants {
    double R_odom;
    double R_dist;
    double q_pos;
    double q_vel;
    double gate_sigma;
    bool use_dist_weight;
    double dist_weight;
    bool use_gate_2d;
};

extern const Ports kPorts;
extern const DriverCurves kThrottleCurve;
extern const DriverCurves kSteerCurve;
extern const DrivetrainConstants kDrivetrain;
extern const LemlibControllers kLemlibControllers;
extern const TrackingWheelConstants kTrackingWheel;
extern const DistanceLocalizationConstants kDistLoc;
extern const RayWallConstants kRayWall;
extern const LocalizationTaskConstants kLocTasks;
extern const MotionConstants kMotion;
extern const KalmanClampConstants kKalmanClamps;
extern const KalmanTuningConstants kKalmanTuning;

} // namespace constants
