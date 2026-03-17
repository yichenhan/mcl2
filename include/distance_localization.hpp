#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace distance_loc {

// ============================================================================
// Types shared by distance_localization.cpp and ray_wall.cpp
// ============================================================================

struct SensorOffset {
    float x;
    float y;
};

struct SensorOffsets {
    SensorOffset left;
    SensorOffset right;
    SensorOffset front;
    SensorOffset back;
};

struct DistanceReadings {
    std::optional<int32_t> left;
    std::optional<int32_t> right;
    std::optional<int32_t> front;
    std::optional<int32_t> back;
};

struct PreviousPosition {
    float x;
    float y;
    bool valid;
};

struct DistanceLocConfig {
    int32_t invalid_reading;
    float max_jump;
    float max_velocity;
    float wall_sum_tolerance;
};

enum class DistanceLocStatus {
    Good,
    velocity_impossible,
    max_jump_impossible,
    bounds_impossible
};

struct DistanceLocResult {
    float x = 0.0f;
    float y = 0.0f;
    DistanceLocStatus status = DistanceLocStatus::Good;
    std::string message;
    float left_proj = 0.0f;
    float right_proj = 0.0f;
    float front_proj = 0.0f;
    float back_proj = 0.0f;
    int valid_candidates = 0;
    float candidate_x_range = 0.0f;
    float candidate_y_range = 0.0f;
    std::vector<float> candidate_xs;
    std::vector<float> candidate_ys;
    float best_mse = 0.0f;
};

// ============================================================================
// Ray-cast types (used by ray_wall.cpp, exposed for MCL)
// ============================================================================

struct Vec2 {
    double x;
    double y;
};

enum class WallId { None, Left, Right, Top, Bottom };

// ============================================================================
// Function declarations — distance_localization.cpp
// ============================================================================

const char* statusToString(DistanceLocStatus status);

DistanceLocResult computePosition(
    const DistanceReadings& readings,
    float heading,
    const SensorOffsets& offsets,
    const PreviousPosition& prev_pos,
    float dt,
    const DistanceLocConfig& config
);

// ============================================================================
// Function declarations — ray_wall.cpp (grid search)
// ============================================================================

DistanceLocResult computePositionRayCast(
    const DistanceReadings& readings,
    float heading_deg,
    const SensorOffsets& offsets,
    const DistanceLocConfig& config,
    double grid_step
);

DistanceLocResult computePositionRayCastLocal(
    const DistanceReadings& readings,
    float heading_deg,
    const SensorOffsets& offsets,
    const DistanceLocConfig& config,
    double seed_x,
    double seed_y,
    double window_half,
    double coarse_step,
    double fine_step,
    double max_error_in
);

// ============================================================================
// Ray-cast helper functions — ray_wall.cpp (exposed for MCL particle scoring)
// ============================================================================

double deg2rad(double deg);

Vec2 rotateOffsetByHeading(const Vec2& offset_robot, double theta_deg);

double rayDistanceToSquareWalls(const Vec2& point, const Vec2& direction);

WallId rayWallHit(const Vec2& point, const Vec2& direction);

WallId sensorWallHit(
    const Vec2& robot_pos,
    double theta_deg,
    const Vec2& sensor_offset_rb,
    double sensor_rel_deg
);

double ray_distance_with_offset(
    const Vec2& robot_pos,
    double theta_deg,
    const Vec2& sensor_offset_rb,
    double sensor_rel_deg
);

} // namespace distance_loc
