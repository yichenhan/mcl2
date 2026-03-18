// ============================================================
// Ray-casting + Grid Search Localization (12ft x 12ft field)
// Units: inches
// Field coordinate system: centered at (0,0)
// Walls: x = ±72, y = ±72
// ============================================================

#include "distance_localization.hpp"
#include "constants.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <limits>
#include <vector>

namespace distance_loc {

// #region agent log
static int g_raywall_debug_tick_counter = 0;
// #endregion

// Vec2 and WallId are defined in distance_localization.hpp

// ============================================================================
// Tunable constants live in `src/constants.cpp` (see `include/constants.hpp`).
// ============================================================================

// ============================================================================
// Helper Functions
// ============================================================================

/**
 * @brief Convert degrees to radians
 */
double deg2rad(double deg) {
    return deg * M_PI / 180.0;
}

/**
 * @brief Rotate a local offset by robot heading θ
 * 
 * The sensor offset (ox, oy) is defined in the robot's local frame:
 *   +x = robot-right, +y = robot-forward (typical convention)
 * When the robot rotates by θ, that offset rotates with it.
 * 
 * @param offset_robot  Sensor offset in robot-local coordinates
 * @param theta_deg     Robot heading in degrees (field frame)
 * @return              Offset expressed in the FIELD frame
 */
Vec2 rotateOffsetByHeading(const Vec2& offset_robot, double theta_deg) {
    // In VEX convention angles increase CLOCKWISE.
    // Standard rotation matrix expects CCW-positive, so rotate by (-theta).
    const double t = deg2rad(-theta_deg);
    const double c = std::cos(t);
    const double s = std::sin(t);

    // Standard 2D rotation:
    // [x'] = [ c -s ] [x]
    // [y']   [ s  c ] [y]
    return Vec2{
        offset_robot.x * c - offset_robot.y * s,
        offset_robot.x * s + offset_robot.y * c
    };
}

/**
 * @brief Compute distance from a ray to the first wall it hits (square field)
 * 
 * Ray equation: point(t) = origin + t * direction
 * We solve for t where the ray crosses each wall, then return the smallest positive t.
 * 
 * @param origin     Ray starting point (sensor position in field coordinates)
 * @param direction  Ray direction (unit vector)
 * @return           Distance to the nearest wall (smallest positive t)
 */
double rayDistanceToSquareWalls(const Vec2& point, const Vec2& direction) {
    const double FIELD_HALF = constants::kRayWall.field_half_in;
    const double EPS = constants::kRayWall.eps;
    double nearest_distance = std::numeric_limits<double>::infinity();
    
    // ---------- Check RIGHT wall (x = +72) ----------
    if (direction.x > EPS) {  // Ray is moving right
        double t = (FIELD_HALF - point.x) / direction.x;
        double y_at_hit = point.y + t * direction.y;
        
        // Check if hit point is within wall bounds
        if (t > EPS && y_at_hit >= -FIELD_HALF && y_at_hit <= FIELD_HALF) {
            nearest_distance = std::min(nearest_distance, t);
        }
    }
    
    // ---------- Check LEFT wall (x = -72) ----------
    if (direction.x < -EPS) {  // Ray is moving left
        double t = (-FIELD_HALF - point.x) / direction.x;
        double y_at_hit = point.y + t * direction.y;
        
        if (t > EPS && y_at_hit >= -FIELD_HALF && y_at_hit <= FIELD_HALF) {
            nearest_distance = std::min(nearest_distance, t);
        }
    }
    
    // ---------- Check TOP wall (y = +72) ----------
    if (direction.y > EPS) {  // Ray is moving up
        double t = (FIELD_HALF - point.y) / direction.y;
        double x_at_hit = point.x + t * direction.x;
        
        if (t > EPS && x_at_hit >= -FIELD_HALF && x_at_hit <= FIELD_HALF) {
            nearest_distance = std::min(nearest_distance, t);
        }
    }
    
    // ---------- Check BOTTOM wall (y = -72) ----------
    if (direction.y < -EPS) {  // Ray is moving down
        double t = (-FIELD_HALF - point.y) / direction.y;
        double x_at_hit = point.x + t * direction.x;
        
        if (t > EPS && x_at_hit >= -FIELD_HALF && x_at_hit <= FIELD_HALF) {
            nearest_distance = std::min(nearest_distance, t);
        }
    }
    
    return nearest_distance;
}

// WallId enum is defined in distance_localization.hpp

/**
 * @brief Determine which wall a ray hits first (square field)
 *
 * Same logic as rayDistanceToSquareWalls but returns the wall ID instead of distance.
 *
 * @param point      Ray starting point (sensor position in field coordinates)
 * @param direction  Ray direction (unit vector)
 * @return           The wall that the ray hits first
 */
WallId rayWallHit(const Vec2& point, const Vec2& direction) {
    const double FIELD_HALF = constants::kRayWall.field_half_in;
    const double EPS = constants::kRayWall.eps;
    double nearest_t = std::numeric_limits<double>::infinity();
    WallId nearest_wall = WallId::None;
    
    // ---------- Check RIGHT wall (x = +72) ----------
    if (direction.x > EPS) {
        double t = (FIELD_HALF - point.x) / direction.x;
        double y_at_hit = point.y + t * direction.y;
        
        if (t > EPS && y_at_hit >= -FIELD_HALF && y_at_hit <= FIELD_HALF) {
            if (t < nearest_t) {
                nearest_t = t;
                nearest_wall = WallId::Right;
            }
        }
    }
    
    // ---------- Check LEFT wall (x = -72) ----------
    if (direction.x < -EPS) {
        double t = (-FIELD_HALF - point.x) / direction.x;
        double y_at_hit = point.y + t * direction.y;
        
        if (t > EPS && y_at_hit >= -FIELD_HALF && y_at_hit <= FIELD_HALF) {
            if (t < nearest_t) {
                nearest_t = t;
                nearest_wall = WallId::Left;
            }
        }
    }
    
    // ---------- Check TOP wall (y = +72) ----------
    if (direction.y > EPS) {
        double t = (FIELD_HALF - point.y) / direction.y;
        double x_at_hit = point.x + t * direction.x;
        
        if (t > EPS && x_at_hit >= -FIELD_HALF && x_at_hit <= FIELD_HALF) {
            if (t < nearest_t) {
                nearest_t = t;
                nearest_wall = WallId::Top;
            }
        }
    }
    
    // ---------- Check BOTTOM wall (y = -72) ----------
    if (direction.y < -EPS) {
        double t = (-FIELD_HALF - point.y) / direction.y;
        double x_at_hit = point.x + t * direction.x;
        
        if (t > EPS && x_at_hit >= -FIELD_HALF && x_at_hit <= FIELD_HALF) {
            if (t < nearest_t) {
                nearest_t = t;
                nearest_wall = WallId::Bottom;
            }
        }
    }
    
    return nearest_wall;
}

/**
 * @brief Determine which wall a sensor ray hits at a given robot pose
 * 
 * @param robot_pos        Robot center in FIELD coordinates
 * @param theta_deg        Robot heading in degrees (FIELD frame)
 * @param sensor_offset_rb Sensor offset in ROBOT coordinates (inches)
 * @param sensor_rel_deg   Sensor direction relative to robot heading (degrees)
 * @return                 The wall that the sensor ray hits
 */
WallId sensorWallHit(
    const Vec2& robot_pos,
    double theta_deg,
    const Vec2& sensor_offset_rb,
    double sensor_rel_deg
) {
    // 1) Rotate the offset into the FIELD frame
    const Vec2 offset_field = rotateOffsetByHeading(sensor_offset_rb, theta_deg);

    // 2) Sensor origin in FIELD frame
    const Vec2 sensor_pos{
        robot_pos.x + offset_field.x,
        robot_pos.y + offset_field.y
    };

    // 3) Sensor ray direction in FIELD frame
    const double ray_angle_deg = theta_deg + sensor_rel_deg;
    const double a = deg2rad(ray_angle_deg);
    // With VEX heading convention (0°=+Y, +CW), the unit direction is:
    // dx = sin(theta), dy = cos(theta)
    const Vec2 v{ std::sin(a), std::cos(a) };  // unit direction

    // 4) Determine which wall the ray hits
    return rayWallHit(sensor_pos, v);
}

/**
 * @brief Predict the distance a specific sensor should read at a given robot pose
 * 
 * @param robot_pos        Robot center in FIELD coordinates
 * @param theta_deg        Robot heading in degrees (FIELD frame)
 * @param sensor_offset_rb Sensor offset in ROBOT coordinates (inches)
 * @param sensor_rel_deg   Sensor direction relative to robot heading (degrees)
 * @return                 Predicted distance to the first wall along sensor's ray
 */
double ray_distance_with_offset(
    const Vec2& robot_pos,
    double theta_deg,
    const Vec2& sensor_offset_rb,
    double sensor_rel_deg
) {
    // 1) Rotate the offset into the FIELD frame
    const Vec2 offset_field = rotateOffsetByHeading(sensor_offset_rb, theta_deg);

    // 2) Sensor origin in FIELD frame
    const Vec2 sensor_pos{
        robot_pos.x + offset_field.x,
        robot_pos.y + offset_field.y
    };

    // 3) Sensor ray direction in FIELD frame
    const double ray_angle_deg = theta_deg + sensor_rel_deg;
    const double a = deg2rad(ray_angle_deg);
    // With VEX heading convention (0°=+Y, +CW), the unit direction is:
    // dx = sin(theta), dy = cos(theta)
    const Vec2 v{ std::sin(a), std::cos(a) };  // unit direction

    // 4) Raycast to square walls
    return rayDistanceToSquareWalls(sensor_pos, v);
}

// ============================================================================
// Sensor Mount Configuration
// ============================================================================

/**
 * @brief Sensor mounting information (offset + direction)
 */
struct SensorMount {
    Vec2 offset;        // Position in robot frame (inches)
    double rel_angle;   // Direction relative to robot heading (degrees)
};

/**
 * @brief Check if a sensor reading is valid
 */
static inline bool isValidReading(std::optional<int32_t> reading, int32_t invalid_value) {
    if (!reading.has_value()) return false;
    int32_t val = reading.value();
    return val >= 0 && val < invalid_value;
}

/**
 * @brief Convert millimeters to inches
 */
static inline double mmToInches(int32_t mm) {
    return static_cast<double>(mm) / 25.4;
}

static inline double clampd(double v, double lo, double hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

// ============================================================================
// Main Grid Search Function
// ============================================================================

DistanceLocResult computePositionRayCast(
    const DistanceReadings& readings,
    float heading_deg,
    const SensorOffsets& offsets,
    const DistanceLocConfig& config,
    double grid_step
) {
    DistanceLocResult result;
    result.status = DistanceLocStatus::Good;
    
    // #region agent log
    g_raywall_debug_tick_counter++;
    bool shouldLog = (constants::kRayWall.global_log_every_n > 0) &&
                     (g_raywall_debug_tick_counter % constants::kRayWall.global_log_every_n == 0);
    // #endregion

    // ========================================================================
    // Step 1: Validate that we have at least one sensor reading
    // ========================================================================
    
    bool has_left  = isValidReading(readings.left,  config.invalid_reading);
    bool has_right = isValidReading(readings.right, config.invalid_reading);
    bool has_front = isValidReading(readings.front, config.invalid_reading);
    bool has_back  = isValidReading(readings.back,  config.invalid_reading);
    
    int valid_count = (has_left ? 1 : 0) + (has_right ? 1 : 0) + 
                      (has_front ? 1 : 0) + (has_back ? 1 : 0);
    
    if (valid_count < 2) {
        result.status = DistanceLocStatus::bounds_impossible;
        result.message = "Need at least 2 valid sensor readings for grid search";
        // #region agent log
        if (shouldLog) {
            printf("\n=== RAY_WALL DEBUG (tick %d) ===\n", g_raywall_debug_tick_counter);
            printf("FAIL: %s (valid_count=%d)\n", result.message.c_str(), valid_count);
            printf("RAW(mm): L=%d R=%d F=%d B=%d | heading_deg=%.2f\n",
                   readings.left.value_or(-1), readings.right.value_or(-1),
                   readings.front.value_or(-1), readings.back.value_or(-1),
                   heading_deg);
            printf("================================\n");
        }
        // #endregion
        return result;
    }

    // ========================================================================
    // Step 2: Build sensor data array (bundles validity, measurement, and mount)
    // ========================================================================
    
    struct SensorData {
        bool is_valid;
        double measured;
        SensorMount mount;
    };
    
    SensorData sensors[] = {
        { has_left,  has_left  ? mmToInches(readings.left.value())  : 0.0,
          { {offsets.left.x,  offsets.left.y},  constants::kRayWall.sensor_angle_left_deg  } },
        { has_right, has_right ? mmToInches(readings.right.value()) : 0.0,
          { {offsets.right.x, offsets.right.y}, constants::kRayWall.sensor_angle_right_deg } },
        { has_front, has_front ? mmToInches(readings.front.value()) : 0.0,
          { {offsets.front.x, offsets.front.y}, constants::kRayWall.sensor_angle_front_deg } },
        { has_back,  has_back  ? mmToInches(readings.back.value())  : 0.0,
          { {offsets.back.x,  offsets.back.y},  constants::kRayWall.sensor_angle_back_deg  } }
    };

    // ========================================================================
    // Step 3: Grid search
    // ========================================================================
    
    double best_error = std::numeric_limits<double>::infinity();
    double best_x = 0.0;
    double best_y = 0.0;
    
    const double theta = static_cast<double>(heading_deg);
    const double MAX_PER_SENSOR_ERR_IN = constants::kRayWall.max_per_sensor_err_in;
    
    // #region agent log
    // Summary stats for debugging (avoid per-candidate spam)
    long rejected_abs_err = 0;
    long rejected_best_error = 0;
    bool found_any_within_gate = false;
    
    // Track the "closest to passing" candidate (smallest amount over the gate)
    double closest_over_by = std::numeric_limits<double>::infinity(); // abs_err - MAX_PER_SENSOR_ERR_IN
    double closest_over_x = 0.0;
    double closest_over_y = 0.0;
    int closest_over_sensor = -1; // 0=L,1=R,2=F,3=B
    double closest_over_abs_err = 0.0;
    // #endregion
    
    // Loop over all candidate positions
    for (double x = -constants::kRayWall.field_half_in; x <= constants::kRayWall.field_half_in; x += grid_step) {
        for (double y = -constants::kRayWall.field_half_in; y <= constants::kRayWall.field_half_in; y += grid_step) {
            Vec2 candidate{ x, y };
            double error = 0.0;
            bool skip = false;
            
            // Accumulate squared error for each valid sensor
            for (int i = 0; i < 4; i++) {
                if (!sensors[i].is_valid) continue;
                
                double predicted = ray_distance_with_offset(
                    candidate, theta, 
                    sensors[i].mount.offset, 
                    sensors[i].mount.rel_angle);
                
                // Hard constraint: each sensor must match within ±3 inches
                double abs_err = std::fabs(predicted - sensors[i].measured);
                if (abs_err > MAX_PER_SENSOR_ERR_IN) {
                    // #region agent log
                    rejected_abs_err++;
                    double over_by = abs_err - MAX_PER_SENSOR_ERR_IN;
                    if (over_by < closest_over_by) {
                        closest_over_by = over_by;
                        closest_over_x = x;
                        closest_over_y = y;
                        closest_over_sensor = i;
                        closest_over_abs_err = abs_err;
                    }
                    // #endregion
                    skip = true;
                    break;
                }

                double diff = predicted - sensors[i].measured;
                error += diff * diff;
                
                // Early exit optimization
                if (error >= best_error) {
                    // #region agent log
                    rejected_best_error++;
                    // #endregion
                    skip = true;
                    break;
                }
            }
            
            if (skip) continue;
            // #region agent log
            found_any_within_gate = true;
            // #endregion
            
            // This position is better than our current best
            if (error < best_error) {
                best_error = error;
                best_x = x;
                best_y = y;
            }
        }
    }

    // ========================================================================
    // Step 5: Validate result
    // ========================================================================
    
    if (best_error == std::numeric_limits<double>::infinity()) {
        result.status = DistanceLocStatus::bounds_impossible;
        result.message = "Grid search failed: no (x,y) fits all sensors within ±3 inches";
        // #region agent log
        if (shouldLog) {
            printf("\n=== RAY_WALL DEBUG (tick %d) ===\n", g_raywall_debug_tick_counter);
            printf("FAIL: %s\n", result.message.c_str());
            printf("Gate: MAX_PER_SENSOR_ERR_IN=%.2f in | grid_step=%.2f in\n",
                   MAX_PER_SENSOR_ERR_IN, grid_step);
            printf("RAW(mm): L=%d R=%d F=%d B=%d | heading_deg=%.2f\n",
                   readings.left.value_or(-1), readings.right.value_or(-1),
                   readings.front.value_or(-1), readings.back.value_or(-1),
                   heading_deg);
            printf("Stats: rejected_abs_err=%ld, rejected_best_error=%ld, found_any_within_gate=%d\n",
                   rejected_abs_err, rejected_best_error, found_any_within_gate ? 1 : 0);
            if (closest_over_sensor != -1) {
                const char* sensor_name = (closest_over_sensor == 0) ? "LEFT" :
                                          (closest_over_sensor == 1) ? "RIGHT" :
                                          (closest_over_sensor == 2) ? "FRONT" : "BACK";
                printf("Closest-to-passing: (x=%.2f,y=%.2f) sensor=%s abs_err=%.2f (over_by=%.2f)\n",
                       closest_over_x, closest_over_y, sensor_name, closest_over_abs_err, closest_over_by);
            } else {
                printf("Closest-to-passing: n/a\n");
            }
            printf("================================\n");
        }
        // #endregion
        return result;
    }
    
    // Check if error is unreasonably high (sensors don't match any position well)
    double rms_error = std::sqrt(best_error / valid_count);

    // ========================================================================
    // Step 6: Return best position
    // ========================================================================
    
    result.x = static_cast<float>(best_x);
    result.y = static_cast<float>(best_y);
    result.status = DistanceLocStatus::Good;
    result.message = "Position found via grid search (RMS error: " + 
                     std::to_string(rms_error) + " inches)";
    
    // #region agent log
    if (shouldLog) {
        printf("\n=== RAY_WALL DEBUG (tick %d) ===\n", g_raywall_debug_tick_counter);
        printf("SELECTED: x=%.2f, y=%.2f | best_error(SSE)=%.4f | rms_error=%.3f | valid_sensors=%d\n",
               best_x, best_y, best_error, rms_error, valid_count);
        printf("Gate: MAX_PER_SENSOR_ERR_IN=%.2f in | grid_step=%.2f in\n",
               MAX_PER_SENSOR_ERR_IN, grid_step);
        printf("Stats: rejected_abs_err=%ld, rejected_best_error=%ld\n",
               rejected_abs_err, rejected_best_error);
        printf("================================\n");
    }
    // #endregion
    
    // Store predicted distances at best position for debugging
    // sensors[] order: [0]=left, [1]=right, [2]=front, [3]=back
    Vec2 best_pos{ best_x, best_y };
    float* proj_outputs[] = { &result.left_proj, &result.right_proj, 
                              &result.front_proj, &result.back_proj };
    
    for (int i = 0; i < 4; i++) {
        if (sensors[i].is_valid) {
            *proj_outputs[i] = static_cast<float>(ray_distance_with_offset(
                best_pos, theta, sensors[i].mount.offset, sensors[i].mount.rel_angle));
        }
    }

    // #region agent log
    // Print measured vs predicted error at the selected pose (every 50 ticks)
    if (shouldLog) {
        const char* names[] = { "LEFT", "RIGHT", "FRONT", "BACK" };
        printf("Measured vs Predicted (inches) at selected pose:\n");
        for (int i = 0; i < 4; i++) {
            if (!sensors[i].is_valid) continue;
            double predicted = ray_distance_with_offset(
                best_pos, theta, sensors[i].mount.offset, sensors[i].mount.rel_angle);
            double abs_err = std::fabs(predicted - sensors[i].measured);
            printf("  %s: meas=%.2f pred=%.2f abs_err=%.2f\n",
                   names[i], sensors[i].measured, predicted, abs_err);
        }
    }
    // #endregion
    
    return result;
}

// ============================================================================
// Local-Window Grid Search (fast for high-rate loops)
// ============================================================================

DistanceLocResult computePositionRayCastLocal(
    const DistanceReadings& readings,
    float heading_deg,
    const SensorOffsets& offsets,
    const DistanceLocConfig& config,
    double seed_x,
    double seed_y,
    double window_half, //VERY DANGEROUS TO CHANGE!!!!!!
    double coarse_step,
    double fine_step,
    double max_error_in
) {
    DistanceLocResult result;
    result.status = DistanceLocStatus::Good;

    // #region agent log
    g_raywall_debug_tick_counter++;
    bool shouldLog = (constants::kRayWall.local_log_every_n > 0) &&
                     (g_raywall_debug_tick_counter % constants::kRayWall.local_log_every_n == 0);
    // #endregion

    // Validate basic inputs
    if (window_half <= 0.0 || coarse_step <= 0.0 || fine_step <= 0.0) {
        result.status = DistanceLocStatus::bounds_impossible;
        result.message = "Local grid search invalid params (window/step must be > 0)";
        return result;
    }

    // Validate that we have at least 2 valid sensor readings (same as global ray-wall)
    bool has_left  = isValidReading(readings.left,  config.invalid_reading);
    bool has_right = isValidReading(readings.right, config.invalid_reading);
    bool has_front = isValidReading(readings.front, config.invalid_reading);
    bool has_back  = isValidReading(readings.back,  config.invalid_reading);

    int valid_count = (has_left ? 1 : 0) + (has_right ? 1 : 0) +
                      (has_front ? 1 : 0) + (has_back ? 1 : 0);

    // Require at least one sensor per axis to constrain both X and Y.
    // Otherwise the search can "fit" along a line using only X-axis or only Y-axis sensors.
    const bool has_x_axis = has_left || has_right;
    const bool has_y_axis = has_front || has_back;
    if (!has_x_axis || !has_y_axis) {
        result.status = DistanceLocStatus::bounds_impossible;
        result.message = "Need at least one X-axis (LEFT/RIGHT) AND one Y-axis (FRONT/BACK) sensor for local grid search";
        // #region agent log
        if (shouldLog) {
            printf("\n=== RAY_WALL_LOCAL DEBUG (tick %d) ===\n", g_raywall_debug_tick_counter);
            printf("FAIL: %s\n", result.message.c_str());
            printf("Axis sensors: X=%d (L=%d R=%d) | Y=%d (F=%d B=%d) | valid_count=%d\n",
                   has_x_axis ? 1 : 0, has_left ? 1 : 0, has_right ? 1 : 0,
                   has_y_axis ? 1 : 0, has_front ? 1 : 0, has_back ? 1 : 0,
                   valid_count);
            printf("RAW(mm): L=%d R=%d F=%d B=%d | heading_deg=%.2f\n",
                   readings.left.value_or(-1), readings.right.value_or(-1),
                   readings.front.value_or(-1), readings.back.value_or(-1),
                   heading_deg);
            printf("======================================\n");
        }
        // #endregion
        return result;
    }

    if (valid_count < 2) {
        result.status = DistanceLocStatus::bounds_impossible;
        result.message = "Need at least 2 valid sensor readings for local grid search";
        return result;
    }

    // Bundle sensor measurement + mount info
    struct SensorData {
        bool is_valid;
        double measured;
        SensorMount mount;
    };

    SensorData sensors[] = {
        { has_left,  has_left  ? mmToInches(readings.left.value())  : 0.0,
          { {offsets.left.x,  offsets.left.y},  constants::kRayWall.sensor_angle_left_deg  } },
        { has_right, has_right ? mmToInches(readings.right.value()) : 0.0,
          { {offsets.right.x, offsets.right.y}, constants::kRayWall.sensor_angle_right_deg } },
        { has_front, has_front ? mmToInches(readings.front.value()) : 0.0,
          { {offsets.front.x, offsets.front.y}, constants::kRayWall.sensor_angle_front_deg } },
        { has_back,  has_back  ? mmToInches(readings.back.value())  : 0.0,
          { {offsets.back.x,  offsets.back.y},  constants::kRayWall.sensor_angle_back_deg  } }
    };

    const double theta = static_cast<double>(heading_deg);

    // Search radius constraint: only consider candidates within `window_half` inches of the seed.
    // This turns the square window into a circular "12-inch radius" (when window_half=12).
    const double center_x = seed_x;
    const double center_y = seed_y;
    const double radius = window_half;
    const double radius_sq = radius * radius;

    // Candidate scoring:
    // - Compute abs error for ALL valid sensors.
    // - For each axis pair (LEFT/RIGHT and FRONT/BACK), require at least ONE sensor in that pair
    //   to be within ±max_error_in. The other sensor in the pair may be wildly off and is ignored.
    // - Total error is normalized by the number of sensors actually used (inliers), so ignoring a
    //   sensor doesn't artificially lower the score.

    // Vectors to collect all passing candidates' x and y values (for spread gate).
    std::vector<double> passing_x_vals;
    std::vector<double> passing_y_vals;
    std::vector<double> passing_scores;
    std::vector<int> passing_used_counts;
    std::vector<int> passing_inlier_masks;  // 4-bit mask: bit0=left, bit1=right, bit2=front, bit3=back

    auto findBestInWindow = [&](double min_x, double max_x, double min_y, double max_y, double step,
                                double& out_x, double& out_y, double& out_best_score, int& out_used_count,
                                bool collect_passing = false) -> bool {
        out_best_score = std::numeric_limits<double>::infinity();
        out_used_count = 0;
        bool found_any = false;

        for (double x = min_x; x <= max_x; x += step) {
            for (double y = min_y; y <= max_y; y += step) {
                // Radius gate around the seed (odom) pose
                const double dx_seed = x - center_x;
                const double dy_seed = y - center_y;
                if ((dx_seed * dx_seed + dy_seed * dy_seed) > radius_sq) {
                    continue;
                }
                Vec2 candidate{ x, y };
                bool skip = false;

                // Compute predicted + errors for all sensors (even if some will be ignored).
                std::array<double, 4> diffs{0, 0, 0, 0};
                std::array<double, 4> abs_errs{0, 0, 0, 0};
                for (int i = 0; i < 4; i++) {
                    if (!sensors[i].is_valid) continue;
                    const double predicted = ray_distance_with_offset(
                        candidate, theta,
                        sensors[i].mount.offset,
                        sensors[i].mount.rel_angle
                    );
                    const double diff = predicted - sensors[i].measured;
                    diffs[i] = diff;
                    abs_errs[i] = std::fabs(diff);
                }

                // Axis-pair gate: at least one inlier on X-axis and one inlier on Y-axis.
                const bool left_in  = sensors[0].is_valid && (abs_errs[0] <= max_error_in);
                const bool right_in = sensors[1].is_valid && (abs_errs[1] <= max_error_in);
                const bool front_in = sensors[2].is_valid && (abs_errs[2] <= max_error_in);
                const bool back_in  = sensors[3].is_valid && (abs_errs[3] <= max_error_in);

                if (!(left_in || right_in)) {
                    skip = true;
                }
                if (!(front_in || back_in)) {
                    skip = true;
                }
                if (skip) continue;

                // Accumulate SSE over *inlier* sensors only, normalize by used_count.
                double sse = 0.0;
                int used_count = 0;
                for (int i = 0; i < 4; i++) {
                    if (!sensors[i].is_valid) continue;
                    if (abs_errs[i] > max_error_in) continue; // ignored outlier
                    const double diff = diffs[i];
                    sse += diff * diff;
                    used_count++;
                }

                // Should be at least 2 (one from each axis) if we passed the gate, but guard anyway.
                if (used_count <= 0) continue;

                const double score = sse / static_cast<double>(used_count); // MSE (RMS = sqrt(score))

                // Collect passing candidates for spread gate and weighted mean (coarse pass only).
                if (collect_passing) {
                    passing_x_vals.push_back(x);
                    passing_y_vals.push_back(y);
                    passing_scores.push_back(score);
                    passing_used_counts.push_back(used_count);

                    int inlier_mask = 0;
                    if (left_in)  inlier_mask |= (1 << 0);
                    if (right_in) inlier_mask |= (1 << 1);
                    if (front_in) inlier_mask |= (1 << 2);
                    if (back_in)  inlier_mask |= (1 << 3);
                    passing_inlier_masks.push_back(inlier_mask);
                }

                found_any = true;
                // Primary: prefer MORE inlier sensors. Secondary: lower MSE as tie-breaker.
                bool dominated_by_best = 
                    (used_count < out_used_count) ||
                    (used_count == out_used_count && score >= out_best_score);
                if (dominated_by_best) continue;

                out_best_score = score;
                out_used_count = used_count;
                out_x = x;
                out_y = y;
            }
        }

        return found_any && out_best_score != std::numeric_limits<double>::infinity();
    };

    // Clamp seed to field bounds
    seed_x = clampd(seed_x, -constants::kRayWall.field_half_in, constants::kRayWall.field_half_in);
    seed_y = clampd(seed_y, -constants::kRayWall.field_half_in, constants::kRayWall.field_half_in);

    // Coarse pass over local window
    const double coarse_min_x =
        clampd(seed_x - window_half, -constants::kRayWall.field_half_in, constants::kRayWall.field_half_in);
    const double coarse_max_x =
        clampd(seed_x + window_half, -constants::kRayWall.field_half_in, constants::kRayWall.field_half_in);
    const double coarse_min_y =
        clampd(seed_y - window_half, -constants::kRayWall.field_half_in, constants::kRayWall.field_half_in);
    const double coarse_max_y =
        clampd(seed_y + window_half, -constants::kRayWall.field_half_in, constants::kRayWall.field_half_in);

    double best_x = seed_x;
    double best_y = seed_y;
    double best_score = std::numeric_limits<double>::infinity();
    int best_used_count = 0;

    bool found = findBestInWindow(coarse_min_x, coarse_max_x, coarse_min_y, coarse_max_y,
                                  coarse_step, best_x, best_y, best_score, best_used_count,
                                  true /* collect_passing */);

    // Use best candidate (lowest error) directly - no averaging

    // ========================================================================
    // Spread gate: reject if valid candidates span too large a range.
    // When global spread fails (coaxial sensors disagree), fall back to
    // per-inlier-set grouping: group candidates by which sensors were
    // inliers, apply the spread gate per group, and pick the best group.
    // ========================================================================
    if (found && !passing_x_vals.empty() && !passing_y_vals.empty()) {
        const double x_min = *std::min_element(passing_x_vals.begin(), passing_x_vals.end());
        const double x_max = *std::max_element(passing_x_vals.begin(), passing_x_vals.end());
        const double y_min = *std::min_element(passing_y_vals.begin(), passing_y_vals.end());
        const double y_max = *std::max_element(passing_y_vals.begin(), passing_y_vals.end());
        const double x_range = x_max - x_min;
        const double y_range = y_max - y_min;
        const double max_spread = constants::kRayWall.local_max_candidate_spread_in;

        if (x_range > max_spread || y_range > max_spread) {
            // Global spread gate failed -- try per-inlier-set grouping.
            // 4-bit mask => 16 possible groups. Fixed-size array avoids heap allocation.
            struct GroupInfo {
                double x_min, x_max, y_min, y_max;
                double best_score;
                int best_used;
                double best_x, best_y;
                int count;
            };
            GroupInfo groups[16];
            for (int g = 0; g < 16; g++) {
                groups[g].x_min =  std::numeric_limits<double>::infinity();
                groups[g].x_max = -std::numeric_limits<double>::infinity();
                groups[g].y_min =  std::numeric_limits<double>::infinity();
                groups[g].y_max = -std::numeric_limits<double>::infinity();
                groups[g].best_score = std::numeric_limits<double>::infinity();
                groups[g].best_used = 0;
                groups[g].best_x = 0.0;
                groups[g].best_y = 0.0;
                groups[g].count = 0;
            }

            // Single O(N) pass: bucket each candidate into its group.
            for (size_t i = 0; i < passing_inlier_masks.size(); i++) {
                const int mask = passing_inlier_masks[i];
                GroupInfo& g = groups[mask];
                const double cx = passing_x_vals[i];
                const double cy = passing_y_vals[i];
                g.count++;
                if (cx < g.x_min) g.x_min = cx;
                if (cx > g.x_max) g.x_max = cx;
                if (cy < g.y_min) g.y_min = cy;
                if (cy > g.y_max) g.y_max = cy;

                const int used = passing_used_counts[i];
                const double sc = passing_scores[i];
                bool better = (used > g.best_used) ||
                              (used == g.best_used && sc < g.best_score);
                if (better) {
                    g.best_score = sc;
                    g.best_used = used;
                    g.best_x = cx;
                    g.best_y = cy;
                }
            }

            // Find the best group that passes the per-group spread gate.
            int winning_mask = -1;
            double winning_score = std::numeric_limits<double>::infinity();
            int winning_used = 0;
            for (int g = 0; g < 16; g++) {
                if (groups[g].count == 0) continue;
                const double gx_range = groups[g].x_max - groups[g].x_min;
                const double gy_range = groups[g].y_max - groups[g].y_min;
                if (gx_range > max_spread || gy_range > max_spread) continue;

                bool better = (groups[g].best_used > winning_used) ||
                              (groups[g].best_used == winning_used && groups[g].best_score < winning_score);
                if (better) {
                    winning_mask = g;
                    winning_score = groups[g].best_score;
                    winning_used = groups[g].best_used;
                }
            }

            if (winning_mask >= 0) {
                // A consistent sensor subset resolved the conflict.
                best_x = groups[winning_mask].best_x;
                best_y = groups[winning_mask].best_y;
                best_score = groups[winning_mask].best_score;
                best_used_count = groups[winning_mask].best_used;

                // Replace passing candidate lists with only the winning group's
                // candidates so that diagnostics and visualization are correct.
                std::vector<double> filtered_x, filtered_y;
                std::vector<double> filtered_scores;
                std::vector<int> filtered_used;
                for (size_t i = 0; i < passing_inlier_masks.size(); i++) {
                    if (passing_inlier_masks[i] == winning_mask) {
                        filtered_x.push_back(passing_x_vals[i]);
                        filtered_y.push_back(passing_y_vals[i]);
                        filtered_scores.push_back(passing_scores[i]);
                        filtered_used.push_back(passing_used_counts[i]);
                    }
                }
                passing_x_vals = std::move(filtered_x);
                passing_y_vals = std::move(filtered_y);
                passing_scores = std::move(filtered_scores);
                passing_used_counts = std::move(filtered_used);

                // #region agent log
                if (shouldLog) {
                    const char* sensor_names[] = {"L","R","F","B"};
                    printf("\n=== RAY_WALL_LOCAL DEBUG (tick %d) ===\n", g_raywall_debug_tick_counter);
                    printf("Spread gate: GLOBAL FAILED (x_range=%.2f y_range=%.2f max=%.2f) -> per-group fallback\n",
                           x_range, y_range, max_spread);
                    printf("Winning group mask=0x%X (sensors:", winning_mask);
                    for (int b = 0; b < 4; b++) {
                        if (winning_mask & (1 << b)) printf(" %s", sensor_names[b]);
                    }
                    printf(") | best=(%.2f,%.2f) score=%.4f used=%d candidates=%zu\n",
                           best_x, best_y, best_score, best_used_count, passing_x_vals.size());
                    printf("======================================\n");
                }
                // #endregion
            } else {
                // No inlier group passes the spread gate -- reject.
                result.status = DistanceLocStatus::bounds_impossible;
                result.message = "Local grid search rejected: candidate spread too large and no "
                                 "per-sensor-group passes spread gate (x_range=" +
                                 std::to_string(x_range) + ", y_range=" + std::to_string(y_range) +
                                 ", max=" + std::to_string(max_spread) + ")";
                // #region agent log
                if (shouldLog) {
                    printf("\n=== RAY_WALL_LOCAL DEBUG (tick %d) ===\n", g_raywall_debug_tick_counter);
                    printf("FAIL: %s\n", result.message.c_str());
                    printf("Spread gate: GLOBAL FAILED (x_range=%.2f y_range=%.2f max=%.2f) "
                           "-> per-group fallback also FAILED (no group passes)\n",
                           x_range, y_range, max_spread);
                    printf("Groups present:");
                    for (int g = 0; g < 16; g++) {
                        if (groups[g].count == 0) continue;
                        printf(" [0x%X: n=%d xr=%.2f yr=%.2f]", g, groups[g].count,
                               groups[g].x_max - groups[g].x_min,
                               groups[g].y_max - groups[g].y_min);
                    }
                    printf("\n");
                    printf("Seed: (%.2f, %.2f) window_half=%.2f coarse_step=%.2f\n",
                           seed_x, seed_y, window_half, coarse_step);
                    printf("RAW(mm): L=%d R=%d F=%d B=%d | heading_deg=%.2f\n",
                           readings.left.value_or(-1), readings.right.value_or(-1),
                           readings.front.value_or(-1), readings.back.value_or(-1),
                           heading_deg);
                    printf("======================================\n");
                }
                // #endregion
                return result;
            }
        }
    }

    if (!found) {
        result.status = DistanceLocStatus::bounds_impossible;
        result.message = "Local grid search failed: no (x,y) has at least one X-axis and one Y-axis sensor within ±" +
                         std::to_string(max_error_in) + " inches";
        // #region agent log
        if (shouldLog) {
            printf("\n=== RAY_WALL_LOCAL DEBUG (tick %d) ===\n", g_raywall_debug_tick_counter);
            printf("FAIL: %s\n", result.message.c_str());
            printf("Valid candidates: %zu\n", passing_x_vals.size());
            printf("Seed: (%.2f, %.2f) window_half=%.2f coarse_step=%.2f fine_step=%.2f\n",
                   seed_x, seed_y, window_half, coarse_step, fine_step);
            printf("RAW(mm): L=%d R=%d F=%d B=%d | heading_deg=%.2f\n",
                   readings.left.value_or(-1), readings.right.value_or(-1),
                   readings.front.value_or(-1), readings.back.value_or(-1),
                   heading_deg);
            printf("======================================\n");
        }
        // #endregion
        return result;
    }

/*Commenting out fine pass for now to see if it helps with accuracy
    // Fine pass around best coarse candidate
    const double fine_half = coarse_step; // refine within ±coarse_step
    const double fine_min_x =
        clampd(best_x - fine_half, -constants::kRayWall.field_half_in, constants::kRayWall.field_half_in);
    const double fine_max_x =
        clampd(best_x + fine_half, -constants::kRayWall.field_half_in, constants::kRayWall.field_half_in);
    const double fine_min_y =
        clampd(best_y - fine_half, -constants::kRayWall.field_half_in, constants::kRayWall.field_half_in);
    const double fine_max_y =
        clampd(best_y + fine_half, -constants::kRayWall.field_half_in, constants::kRayWall.field_half_in);

    double fine_x = best_x;
    double fine_y = best_y;
    double fine_score = std::numeric_limits<double>::infinity();
    int fine_used_count = 0;
    bool found_fine = findBestInWindow(fine_min_x, fine_max_x, fine_min_y, fine_max_y,
                                       fine_step, fine_x, fine_y, fine_score, fine_used_count);
    if (found_fine && (fine_score < best_score - 1e-12 ||
                       (std::fabs(fine_score - best_score) <= 1e-12 && fine_used_count > best_used_count))) {
        best_x = fine_x;
        best_y = fine_y;
        best_score = fine_score;
        best_used_count = fine_used_count;
    }
*/
    // Return pose + diagnostics
    result.x = static_cast<float>(best_x);
    result.y = static_cast<float>(best_y);
    result.status = DistanceLocStatus::Good;
    result.valid_candidates = static_cast<int>(passing_x_vals.size());
    if (!passing_x_vals.empty() && !passing_y_vals.empty()) {
        result.candidate_x_range = static_cast<float>(
            *std::max_element(passing_x_vals.begin(), passing_x_vals.end()) -
            *std::min_element(passing_x_vals.begin(), passing_x_vals.end()));
        result.candidate_y_range = static_cast<float>(
            *std::max_element(passing_y_vals.begin(), passing_y_vals.end()) -
            *std::min_element(passing_y_vals.begin(), passing_y_vals.end()));
        
        // Store all passing candidates for visualization
        result.candidate_xs.reserve(passing_x_vals.size());
        result.candidate_ys.reserve(passing_y_vals.size());
        for (size_t i = 0; i < passing_x_vals.size(); i++) {
            result.candidate_xs.push_back(static_cast<float>(passing_x_vals[i]));
            result.candidate_ys.push_back(static_cast<float>(passing_y_vals[i]));
        }
    }

    const int ignored_count = valid_count - best_used_count;
    const double rms_error = std::sqrt(best_score); // best_score is MSE over used sensors
    result.best_mse = static_cast<float>(best_score);
    result.message = "Position found via local grid search (RMS error: " +
                     std::to_string(rms_error) + " inches, used=" +
                     std::to_string(best_used_count) + ", ignored=" +
                     std::to_string(ignored_count) + ")";

    // Store predicted distances at best position for debugging (same order as computePositionRayCast)
    Vec2 best_pos{ best_x, best_y };
    float* proj_outputs[] = { &result.left_proj, &result.right_proj,
                              &result.front_proj, &result.back_proj };
    for (int i = 0; i < 4; i++) {
        if (sensors[i].is_valid) {
            *proj_outputs[i] = static_cast<float>(ray_distance_with_offset(
                best_pos, theta, sensors[i].mount.offset, sensors[i].mount.rel_angle));
        }
    }

    // #region agent log
    if (shouldLog) {
        printf("\n=== RAY_WALL_LOCAL DEBUG (tick %d) ===\n", g_raywall_debug_tick_counter);
        printf("SELECTED: x=%.2f, y=%.2f | rms_error=%.3f | used_sensors=%d | ignored_sensors=%d | valid_sensors=%d\n",
               best_x, best_y, rms_error, best_used_count, ignored_count, valid_count);
        printf("Seed: (%.2f, %.2f) window_half=%.2f coarse_step=%.2f fine_step=%.2f\n",
               seed_x, seed_y, window_half, coarse_step, fine_step);
        printf("Gate: per-axis (L/R and F/B) require >=1 inlier within ±%.2f in; outliers ignored in score\n",
               max_error_in);
        // Log spread gate diagnostics
        if (!passing_x_vals.empty() && !passing_y_vals.empty()) {
            const double x_range = *std::max_element(passing_x_vals.begin(), passing_x_vals.end()) -
                                   *std::min_element(passing_x_vals.begin(), passing_x_vals.end());
            const double y_range = *std::max_element(passing_y_vals.begin(), passing_y_vals.end()) -
                                   *std::min_element(passing_y_vals.begin(), passing_y_vals.end());
            printf("Spread gate: x_range=%.2f y_range=%.2f max_spread=%.2f | candidates=%zu\n",
                   x_range, y_range, constants::kRayWall.local_max_candidate_spread_in, passing_x_vals.size());
        }
        printf("======================================\n");
    }
    // #endregion

    return result;
}

} // namespace distance_loc

