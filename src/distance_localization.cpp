#include "distance_localization.hpp"
#include "constants.hpp"
#include <cmath>
#include <sstream>
#include <cstdio>

// #region agent log
static int g_debug_tick_counter = 0;
// #endregion

namespace distance_loc {

// ============================================================================
// Helper Functions
// ============================================================================

/**
 * @brief Check if a sensor reading is valid (not out of range or negative)
 */
static bool isValidReading(std::optional<int32_t> reading, int32_t invalid_value) {
    if (!reading.has_value()) {
        return false;
    }
    int32_t val = reading.value();
    return val >= 0 && val < invalid_value;
}

/**
 * @brief Convert millimeters to inches
 */
static float mmToInches(int32_t mm) {
    return static_cast<float>(mm) * constants::kDistLoc.mm_to_inches;
}

/**
 * @brief Normalize angle to [-PI, PI] range
 */
static float normalizeAngle(float angle) {
    while (angle > M_PI) angle -= 2.0f * M_PI;
    while (angle < -M_PI) angle += 2.0f * M_PI;
    return angle;
}

/**
 * @brief Compute Euclidean distance between two points
 */
static float distance(float x1, float y1, float x2, float y2) {
    float dx = x2 - x1;
    float dy = y2 - y1;
    return std::sqrt(dx * dx + dy * dy);
}

// ============================================================================
// Status String Conversion
// ============================================================================

const char* statusToString(DistanceLocStatus status) {
    switch (status) {
        case DistanceLocStatus::Good:
            return "Good";
        case DistanceLocStatus::velocity_impossible:
            return "Velocity Impossible";
        case DistanceLocStatus::max_jump_impossible:
            return "Max Jump Impossible";
        case DistanceLocStatus::bounds_impossible:
            return "Bounds Impossible";
        default:
            return "Unknown";
    }
}

// ============================================================================
// Main Localization Function
// ============================================================================

DistanceLocResult computePosition(
    const DistanceReadings& readings,
    float heading,
    const SensorOffsets& offsets,
    const PreviousPosition& prev_pos,
    float dt,
    const DistanceLocConfig& config
) {
    DistanceLocResult result;
    result.status = DistanceLocStatus::Good;
    const float FIELD_SIZE = constants::kDistLoc.field_size_in;
    const float HALF_FIELD = FIELD_SIZE / 2.0f;
    
    // #region agent log
    g_debug_tick_counter++;
    bool shouldLog = (constants::kDistLoc.debug_log_every_n > 0) &&
                     (g_debug_tick_counter % constants::kDistLoc.debug_log_every_n == 0);
    // #endregion
    
    // ========================================================================
    // Step 1: Validate that we have required sensor readings
    // ========================================================================
    
    // Check X-axis sensors (left and right)
    bool has_left = isValidReading(readings.left, config.invalid_reading);
    bool has_right = isValidReading(readings.right, config.invalid_reading);
    bool can_compute_x = has_left || has_right;
    
    // Check Y-axis sensors (front and back)
    bool has_front = isValidReading(readings.front, config.invalid_reading);
    bool has_back = isValidReading(readings.back, config.invalid_reading);
    bool can_compute_y = has_front || has_back;
    
    // We need at least one sensor per axis to compute position
    if (!can_compute_x || !can_compute_y) {
        result.status = DistanceLocStatus::bounds_impossible;
        std::ostringstream oss;
        oss << "Missing required sensors: ";
        if (!can_compute_x) oss << "X-axis (need LEFT or RIGHT) ";
        if (!can_compute_y) oss << "Y-axis (need FRONT or BACK)";
        result.message = oss.str();
        return result;
    }
    
    // ========================================================================
    // Step 2: Convert readings to inches and apply heading correction
    // ========================================================================
    
    // Normalize heading to [-PI, PI]
    float h = normalizeAngle(heading);
    
    // Cosine correction factors for projecting sensor readings onto axes
    // When heading = 0 (facing +Y), left/right sensors measure X directly
    // When heading = PI/2 (facing +X), left/right sensors measure Y
    float cos_h = std::cos(h);
    float sin_h = std::sin(h);
    
    // For perpendicular sensors:
    // - LEFT/RIGHT sensors point along the robot's local X-axis
    //   In field coords: direction = (cos(h + PI/2), sin(h + PI/2)) = (-sin(h), cos(h))
    //   Projects onto field X-axis with factor |cos(h)|
    //   
    // - FRONT/BACK sensors point along the robot's local Y-axis
    //   In field coords: direction = (cos(h), sin(h))
    //   Projects onto field Y-axis with factor |cos(h)|
    //
    // We use the absolute value of cosine for projection since we care about
    // the component along the measurement axis regardless of direction.
    
    float abs_cos_h = std::fabs(cos_h);
    float abs_sin_h = std::fabs(sin_h);
    
    // #region agent log
    if (shouldLog) {
        printf("\n=== DISTANCE LOC DEBUG (tick %d) ===\n", g_debug_tick_counter);
        printf("HEADING: raw=%.4f rad, norm=%.4f rad (%.2f deg)\n", heading, h, h * 180.0f / M_PI);
        printf("TRIG: cos=%.4f, sin=%.4f, |cos|=%.4f, |sin|=%.4f\n", cos_h, sin_h, abs_cos_h, abs_sin_h);
        printf("RAW SENSORS (mm): L=%d, R=%d, F=%d, B=%d\n",
            readings.left.value_or(-1), readings.right.value_or(-1),
            readings.front.value_or(-1), readings.back.value_or(-1));
    }
    // #endregion
    
    // ========================================================================
    // Step 3: Compute X position from LEFT and RIGHT sensors
    // ========================================================================
    
    float distance_from_left = 0.0f;
    float distance_from_right = 0.0f;
    float computed_x = 0.0f;
    
    if (has_left && has_right) {
        // Both sensors available - compute from both and average
        float left_dist_in = mmToInches(readings.left.value());
        float right_dist_in = mmToInches(readings.right.value());
        
        // Apply heading projection: distance along field X-axis
        // LEFT sensor: robot's left side points in direction perpendicular to heading
        // At heading=0, left sensor points toward -X (left wall)
        float left_proj = left_dist_in * abs_cos_h;
        float right_proj = right_dist_in * abs_cos_h;
        
        // Apply sensor offset correction
        // Offset.x is sensor position relative to robot center (+ = right)
        // Offset.y affects X position when robot is rotated
        float left_offset_x = offsets.left.x * cos_h - offsets.left.y * sin_h;
        float right_offset_x = offsets.right.x * cos_h - offsets.right.y * sin_h;
        
        // Distance from robot center to left wall (at X = -72)
        // Left sensor has negative offset, subtracting adds the magnitude
        distance_from_left = left_proj - left_offset_x;
        
        // Distance from robot center to right wall (at X = +72)
        // Right sensor has positive offset, need to add to get center distance
        distance_from_right = right_proj + right_offset_x;

        // Store intermediate values in result for debugging
        result.left_proj = left_proj;
        result.right_proj = right_proj;
        
        // #region agent log
        if (shouldLog) {
            printf("X-AXIS:\n");
            printf("  Measured: L=%.2f in, R=%.2f in\n", left_dist_in, right_dist_in);
            printf("  Projected (cos corrected): L=%.2f in, R=%.2f in\n", left_proj, right_proj);
            printf("  Offsets: L(%.2f,%.2f) R(%.2f,%.2f)\n", offsets.left.x, offsets.left.y, offsets.right.x, offsets.right.y);
            printf("  Transformed offset X: L=%.2f, R=%.2f\n", left_offset_x, right_offset_x);
            printf("  Dist from walls: left=%.2f, right=%.2f, sum=%.2f\n", distance_from_left, distance_from_right, distance_from_left + distance_from_right);
        }
        // #endregion
        
        // Wall sum validation for X-axis
        // distance_from_left + distance_from_right should equal FIELD_SIZE
        float x_wall_sum = distance_from_left + distance_from_right;
        
        if (std::fabs(x_wall_sum - FIELD_SIZE) > config.wall_sum_tolerance) {
            // #region agent log
            printf("!!! X-AXIS VIOLATION !!!\n");
            printf("  Wall sum: %.2f (expected %.2f, diff=%.2f, tol=%.2f)\n", 
                x_wall_sum, FIELD_SIZE, std::fabs(x_wall_sum - FIELD_SIZE), config.wall_sum_tolerance);
            printf("  dist_from_left=%.2f, dist_from_right=%.2f\n", distance_from_left, distance_from_right);
            printf("  Projections: L=%.2f, R=%.2f (|cos|=%.4f at %.2f deg)\n", left_proj, right_proj, abs_cos_h, h * 180.0f / M_PI);
            // #endregion
            
            result.status = DistanceLocStatus::bounds_impossible;
            std::ostringstream oss;
            oss << "X-axis wall sum invalid: distance_from_left (" << distance_from_left 
                << ") + distance_from_right (" << distance_from_right << ") = " << x_wall_sum 
                << " vs expected " << FIELD_SIZE << " (tolerance: " << config.wall_sum_tolerance << ")";
            result.message = oss.str();
            return result;
        }
        
        // Compute X position in center-origin coordinates
        // Position from left wall: distance_from_left - HALF_FIELD
        // Position from right wall: HALF_FIELD - distance_from_right
        // Average: (distance_from_left - distance_from_right) / 2
        computed_x = ((HALF_FIELD - distance_from_right) + (distance_from_left - HALF_FIELD)) / 2.0f;
        
    } else if (has_left) {
        // Only left sensor available
        float left_dist_in = mmToInches(readings.left.value());
        float left_proj = left_dist_in * abs_cos_h;
        float left_offset_x = offsets.left.x * cos_h - offsets.left.y * sin_h;
        distance_from_left = left_proj - left_offset_x;
        result.left_proj = left_proj;
        // X position from left wall (at X = -72): distance_from_left - HALF_FIELD
        computed_x = distance_from_left - HALF_FIELD;
        
    } else {  // has_right must be true
        float right_dist_in = mmToInches(readings.right.value());
        float right_proj = right_dist_in * abs_cos_h;
        float right_offset_x = offsets.right.x * cos_h - offsets.right.y * sin_h;
        distance_from_right = right_proj + right_offset_x;
        result.right_proj = right_proj;
        // X position from right wall (at X = +72): HALF_FIELD - distance_from_right
        computed_x = HALF_FIELD - distance_from_right;
    }
    
    // ========================================================================
    // Step 4: Compute Y position from FRONT and BACK sensors
    // ========================================================================
    
    float distance_from_front = 0.0f;
    float distance_from_back = 0.0f;
    float computed_y = 0.0f;
    
    if (has_front && has_back) {
        // Both sensors available - compute from both and average
        float front_dist_in = mmToInches(readings.front.value());
        float back_dist_in = mmToInches(readings.back.value());
        
        // Apply heading projection: distance along field Y-axis
        // At heading=0, front sensor points toward +Y (front wall)
        float front_proj = front_dist_in * abs_cos_h;
        float back_proj = back_dist_in * abs_cos_h;
        
        // Apply sensor offset correction
        // Offset.y is sensor position along robot's forward axis
        float front_offset_y = offsets.front.x * sin_h + offsets.front.y * cos_h;
        float back_offset_y = offsets.back.x * sin_h + offsets.back.y * cos_h;
        
        // Distance from robot center to front wall (at Y = +72)
        // Front sensor has positive offset, need to add to get center distance
        distance_from_front = front_proj + front_offset_y;
        
        // Distance from robot center to back wall (at Y = -72)
        // Back sensor has negative offset, subtracting adds the magnitude
        distance_from_back = back_proj - back_offset_y;
        
        // Store intermediate values in result for debugging
        result.front_proj = front_proj;
        result.back_proj = back_proj;
        
        // #region agent log
        if (shouldLog) {
            printf("Y-AXIS:\n");
            printf("  Measured: F=%.2f in, B=%.2f in\n", front_dist_in, back_dist_in);
            printf("  Projected (cos corrected): F=%.2f in, B=%.2f in\n", front_proj, back_proj);
            printf("  Offsets: F(%.2f,%.2f) B(%.2f,%.2f)\n", offsets.front.x, offsets.front.y, offsets.back.x, offsets.back.y);
            printf("  Transformed offset Y: F=%.2f, B=%.2f\n", front_offset_y, back_offset_y);
            printf("  Dist from walls: front=%.2f, back=%.2f, sum=%.2f\n", distance_from_front, distance_from_back, distance_from_front + distance_from_back);
        }
        // #endregion
        
        // Wall sum validation for Y-axis
        // distance_from_front + distance_from_back should equal FIELD_SIZE
        float y_wall_sum = distance_from_front + distance_from_back;
        
        if (std::fabs(y_wall_sum - FIELD_SIZE) > config.wall_sum_tolerance) {
            // #region agent log
            printf("!!! Y-AXIS VIOLATION !!!\n");
            printf("  Wall sum: %.2f (expected %.2f, diff=%.2f, tol=%.2f)\n", 
                y_wall_sum, FIELD_SIZE, std::fabs(y_wall_sum - FIELD_SIZE), config.wall_sum_tolerance);
            printf("  dist_from_front=%.2f, dist_from_back=%.2f\n", distance_from_front, distance_from_back);
            printf("  Projections: F=%.2f, B=%.2f (|cos|=%.4f at %.2f deg)\n", front_proj, back_proj, abs_cos_h, h * 180.0f / M_PI);
            // #endregion
            
            result.status = DistanceLocStatus::bounds_impossible;
            std::ostringstream oss;
            oss << "Y-axis wall sum invalid: distance_from_front (" << distance_from_front 
                << ") + distance_from_back (" << distance_from_back << ") = " << y_wall_sum 
                << " vs expected " << FIELD_SIZE << " (tolerance: " << config.wall_sum_tolerance << ")";
            result.message = oss.str();
            return result;
        }
        
        // Compute Y position in center-origin coordinates
        // Position from back wall: distance_from_back - HALF_FIELD
        // Position from front wall: HALF_FIELD - distance_from_front
        // Average: (distance_from_back - distance_from_front) / 2
        computed_y = ((HALF_FIELD - distance_from_front) + (distance_from_back - HALF_FIELD)) / 2.0f;
        
    } else if (has_front) {
        // Only front sensor available
        float front_dist_in = mmToInches(readings.front.value());
        float front_proj = front_dist_in * abs_cos_h;
        float front_offset_y = offsets.front.x * sin_h + offsets.front.y * cos_h;
        distance_from_front = front_proj + front_offset_y;
        result.front_proj = front_proj;
        // Y position from front wall (at Y = +72): HALF_FIELD - distance_from_front
        computed_y = HALF_FIELD - distance_from_front;
        
    } else {  // has_back must be true
        float back_dist_in = mmToInches(readings.back.value());
        float back_proj = back_dist_in * abs_cos_h;
        float back_offset_y = offsets.back.x * sin_h + offsets.back.y * cos_h;
        distance_from_back = back_proj - back_offset_y;
        result.back_proj = back_proj;
        // Y position from back wall (at Y = -72): distance_from_back - HALF_FIELD
        computed_y = distance_from_back - HALF_FIELD;
    }
    
    // ========================================================================
    // Step 5: Bounds check - position should be within field
    // ========================================================================
    
    if (computed_x < -HALF_FIELD || computed_x > HALF_FIELD ||
        computed_y < -HALF_FIELD || computed_y > HALF_FIELD) {
        result.status = DistanceLocStatus::bounds_impossible;
        std::ostringstream oss;
        oss << "Position out of bounds: (" << computed_x << ", " << computed_y 
            << ") - field is [" << -HALF_FIELD << ", " << HALF_FIELD << "]";
        result.message = oss.str();
        return result;
    }
    
    // ========================================================================
    // Step 6: Velocity and jump validation (if previous position available)
    // ========================================================================
    
    if (prev_pos.valid && dt > 0.0f) {
        float jump_dist = distance(computed_x, computed_y, prev_pos.x, prev_pos.y);
        
        // Check position jump
        if (jump_dist > config.max_jump) {
            result.status = DistanceLocStatus::max_jump_impossible;
            std::ostringstream oss;
            oss << "Position jump too large: " << jump_dist << " inches (max: " 
                << config.max_jump << ")";
            result.message = oss.str();
            return result;
        }
        
        // Check velocity
        float velocity = jump_dist / dt;
        if (velocity > config.max_velocity) {
            result.status = DistanceLocStatus::velocity_impossible;
            std::ostringstream oss;
            oss << "Velocity too high: " << velocity << " in/s (max: " 
                << config.max_velocity << ")";
            result.message = oss.str();
            return result;
        }
    }
    
    // ========================================================================
    // Step 7: Success - return computed position
    // ========================================================================
    
    result.x = computed_x;
    result.y = computed_y;
    result.status = DistanceLocStatus::Good;
    result.message = "Position computed successfully";
    
    // #region agent log
    if (shouldLog) {
        printf("RESULT: x=%.2f, y=%.2f (heading %.2f deg) - OK\n", computed_x, computed_y, h * 180.0f / M_PI);
        printf("================================\n");
    }
    // #endregion
    
    return result;
}

} // namespace distance_loc

