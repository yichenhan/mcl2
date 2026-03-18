#include "constants.hpp"

#include "lemlib/chassis/trackingWheel.hpp"

namespace constants {

// Hardware mapping -----------------------------------------------------------
// kPorts: All physical hardware mappings (motor ports, sensor ports, and ADI ports).
// Change these when you rewire the robot or need to reverse a device via negative port numbers.
const Ports kPorts{
    // Drivetrain motors (negative = reversed in PROS)
    -13,   // left_drive_m1_port: Left drivetrain motor 1 port
    -14,   // left_drive_m2_port: Left drivetrain motor 2 port
    -12,   // left_drive_m3_port: Left drivetrain motor 3 port
    17,    // right_drive_m1_port: Right drivetrain motor 1 port
    6,    // right_drive_m2_port: Right drivetrain motor 2 port
    18,    // right_drive_m3_port: Right drivetrain motor 3 port

    -21,     // intake1_motor_port: Intake motor 1 port
    11,    // intake2_motor_port: Intake motor 2 port (reversed)

    20,     // imu_port: IMU port
    -19,    // rotation_port: Rotation sensor port

    10,     // distance_right_port: Right distance sensor port
    3,     // distance_left_port: Left distance sensor port
    9,    // distance_front_port: Front distance sensor port
    2,    // distance_back_port: Back distance sensor port

    'B',   // matchloader_adi_port: Matchloader solenoid ADI port
    'A',   // middle_scoring_1_adi_port: Middle scoring 1 solenoid ADI port
    'D',   // trapdoor_adi_port: Trapdoor solenoid ADI port
    'C',   // wing_adi_port: Wing piston ADI port
    'E'    // mdescoreLfunnel_adi_port: Middle descore left funnel piston ADI port
};

// Driver input shaping -------------------------------------------------------
// kThrottleCurve: Shapes driver throttle joystick input (deadband + minimum movement + expo).
// Tune these to make forward/back control feel smooth and predictable.
const DriverCurves kThrottleCurve{
    3,       // joystick_deadband: out of 127
    10,      // min_output: out of 127
    1.019    // expo_gain: unitless
};

// kSteerCurve: Shapes driver steering joystick input (deadband + minimum movement + expo).
// Tune these to control turning sensitivity around center and at full stick.
const DriverCurves kSteerCurve{
    3,       // joystick_deadband: out of 127
    10,      // min_output: out of 127
    1.019    // expo_gain: unitless
};

// Drivetrain / odometry ------------------------------------------------------
// kDrivetrain: Drivetrain geometry + wheel model used by LemLib for motion/odom math.
// Tune these if you change wheel size, track width, gearing RPM, or drift behavior.
const DrivetrainConstants kDrivetrain{
    11.75,                  // track_width_in: inches
    lemlib::Omniwheel::NEW_325, // wheel_diameter_in: inches (3.25" omni)
    450,                    // drivetrain_rpm: rpm (post-gear)
    9.5                     // horizontal_drift: boomerang drift compensation
};

// kLemlibControllers: LemLib PID + slew tuning for lateral (drive) and angular (turn) controllers.
// Tune these to improve path following accuracy and reduce oscillation/overshoot.


const LemlibControllers kLemlibControllers{
    // Lateral controller
    13.3233,         // lat_kP
    0,            // lat_kI
    78,           // lat_kD
    0,              // lat_anti_windup
    0,              // lat_small_err_in
    0,            // lat_small_err_timeout_ms
    0,              // lat_large_err_in
    0,            // lat_large_err_timeout_ms
    0,                  // lat_slew

    // Angular controller
    2.825,          // ang_kP
    0,  // ang_kI
    20,        // ang_kD
    0,              // ang_anti_windup
    0,              // ang_small_err_deg
    0,            // ang_small_err_timeout_ms
    0,              // ang_large_err_deg
    0,            // ang_large_err_timeout_ms
    0,            // ang_slew
};
// kTrackingWheel: Tracking wheel mounting parameters used by LemLib odometry.
// Tune these if the tracking wheel diameter/gear ratio changes or you remount its position.
const TrackingWheelConstants kTrackingWheel{
    lemlib::Omniwheel::NEW_275, // wheel_diameter_in: inches (2.75" omni)
    -0.95,                     // y_offset_in: inches (negative = behind)
    1.0                       // gear_ratio
};

// Distance sensor localization ----------------------------------------------
// kDistLoc: Tuning + geometry for distance-sensor-based localization and validation.
// Update offsets if you move sensors; tune thresholds to reject bad sensor solutions.
const DistanceLocalizationConstants kDistLoc{
    // Sensor offsets from robot center (inches)
    -6.043f,  -1.81102f,   // left_x_in, left_y_in :)
    6.004f,  -2.382f,   // right_x_in, right_y_in :)
    3.268f,   5.512f,   // front_x_in, front_y_in
    -3.878f,   -4.055118f,   // back_x_in, back_y_in :)

    // Validation thresholds
    170.0f,          // max_velocity_in_per_s
    6.0f,            // max_jump_in
    4.0f,            // wall_sum_tolerance_in
    9999,            // invalid_reading_mm

    // Field geometry and unit conversion
    144.0f,          // field_size_in
    (1.0f / 25.4f),  // mm_to_inches

    // Debug cadence
    50               // debug_log_every_n
};

// Ray-wall localization ------------------------------------------------------
// kRayWall: Tuning for the ray-casting / grid-search localization solver.
// Adjust gates/steps if you need more robustness (looser gate) or more precision (smaller step).
const RayWallConstants kRayWall{
    72.0,   // field_half_in
    1e-9,   // eps

    0.0,    // sensor_angle_front_deg
    90.0,   // sensor_angle_right_deg
    180.0,  // sensor_angle_back_deg
    -90.0,  // sensor_angle_left_deg

    1.0,    // max_per_sensor_err_in

    0.5,    // global_grid_step_in
    6.0,   // local_window_half_in
    0.5,    // local_coarse_step_in
    0.25,    // local_fine_step_in
    2.5,    // local_max_error_in
    6.5,    // local_max_candidate_spread_in
    4.0,    // wall_sum_tolerance_in

    50,     // global_log_every_n
    25    // local_log_every_n
};

// Background tasks / correction behavior ------------------------------------
// kLocTasks: Background task rates and gating thresholds for when to apply pose corrections.
// Tune these to balance CPU usage, correction aggressiveness, and log spam.
const LocalizationTaskConstants kLocTasks{
    50,     // dist_loc_task_delay_ms
    1000,   // heading_comp_task_delay_ms
    1000,   // pose_log_task_delay_ms
    2,     // min_valid_sensors_for_correction

    0.0f,   // heading_comp_k_cw  (disabled for testing)
    0.0f,  // heading_comp_k_ccw  (negative to ADD heading for CCW underturn)

    180.0f,   // imu_heading_offset_deg: static offset added to imu.get_heading()

    0.0f,   // angle_equal_eps_deg
    0.5f,   // min_correction_delta_in
    2.0f,   // min_odom_raywall_error_in

    0.1f,   // corner_distance_tolerance_in: reject sensor if ray hits within ±x inches of a field corner

    90.0f   // max_wall_incidence_deg: reject sensor if hitting wall at > 60° from perpendicular
};

// Encoder-based motion helper ------------------------------------------------
// kMotion: Constants used by the simple encoder-based `moveDistance()` helper (not LemLib motion).
// Update if you change drive wheel diameter or drivetrain gearing/cartridge.
const MotionConstants kMotion{
    3.14159,  // pi
    3.25,     // drive_wheel_diameter_in
    600.0,    // motor_cartridge_rpm
    450.0     // drivetrain_rpm
};

// Kalman defaults / clamps ---------------------------------------------------
// kKalmanClamps: Numerical safety limits to prevent filter blow-ups (dt cap, min variances, etc.).
// Generally leave these alone unless you understand the stability implications.
const KalmanClampConstants kKalmanClamps{
    0.5,   // max_dt_s
    1e-9,  // min_innov_cov
    1e-3,  // min_weight
    1e-9,  // min_R
    1e6,   // max_R
    1e-6,  // min_P_diag
    50     // debug_log_every_n
};

// kKalmanTuning: Default PoseKF tuning values (measurement/process noise and gating behavior).
// Tune these to control how strongly the filter trusts odom vs distance localization.
const KalmanTuningConstants kKalmanTuning{
    1.0,    // R_odom
    9.0,    // R_dist
    0.05,   // q_pos
    1.0,    // q_vel
    3.0,    // gate_sigma
    false,  // use_dist_weight
    0.9,    // dist_weight
    false   // use_gate_2d
};

}  // namespace constants


