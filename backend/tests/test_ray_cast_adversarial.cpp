// =============================================================================
// test_ray_cast_adversarial.cpp
//
// Antagonistic edge-case tests for ray casting and distance localization.
// Focuses on geometric degeneracies, boundary rays, obstacle edge cases,
// and numerical precision.
// =============================================================================

#include "doctest/doctest.h"
#include "distance_localization.hpp"
#include "ray/ray_cast_obstacles.hpp"
#include <cmath>
#include <limits>
#include <vector>

using namespace distance_loc;

namespace {

std::vector<sim::Obstacle> wrap_obs(const std::vector<sim::AABB>& obs) {
    std::vector<sim::Obstacle> out;
    out.reserve(obs.size());
    for (const auto& b : obs) {
        out.push_back(sim::Obstacle{b, ""});
    }
    return out;
}

} // namespace

// Sensor mounts
static const Vec2   kLeftOffset  = { -6.043, -1.811 };
static const double kLeftAngle   = -90.0;
static const Vec2   kRightOffset = {  6.004, -2.382 };
static const double kRightAngle  =  90.0;
static const Vec2   kFrontOffset = {  3.268,  5.512 };
static const double kFrontAngle  =   0.0;
static const Vec2   kBackOffset  = { -3.878, -4.055 };
static const double kBackAngle   = 180.0;

// ============================================================================
// 1. RAY PARALLEL TO WALL
//    A ray that's exactly parallel to a wall should hit a perpendicular wall
//    at full field width.
// ============================================================================

TEST_CASE("Ray adversarial: ray parallel to top/bottom walls from center") {
    // Heading 90° from center, front sensor angle 0° => ray direction = (1,0)
    // This ray is parallel to top/bottom walls, should hit right wall at x=72
    double d = rayDistanceToSquareWalls({0.0, 0.0}, {1.0, 0.0});
    CHECK(d == doctest::Approx(72.0).epsilon(1e-6));
}

TEST_CASE("Ray adversarial: ray parallel to left/right walls from center") {
    double d = rayDistanceToSquareWalls({0.0, 0.0}, {0.0, 1.0});
    CHECK(d == doctest::Approx(72.0).epsilon(1e-6));
}

// ============================================================================
// 2. RAY FROM OUTSIDE THE FIELD
//    Sensor offset may place the ray origin outside the field.
// ============================================================================

TEST_CASE("Ray adversarial: origin well outside field shooting inward") {
    // At (100, 0) shooting -X, should hit right wall at x=72 from outside
    double d = rayDistanceToSquareWalls({100.0, 0.0}, {-1.0, 0.0});
    // distance from 100 to 72 = 28 (hitting right wall from outside)
    // But since ray is from OUTSIDE, behavior is implementation-dependent
    // Just verify it's finite and non-negative or infinity
    CHECK((std::isfinite(d) || d == std::numeric_limits<double>::infinity()));
    if (std::isfinite(d)) CHECK(d >= 0.0);
}

TEST_CASE("Ray adversarial: origin outside field shooting outward returns infinity") {
    double d = rayDistanceToSquareWalls({100.0, 0.0}, {1.0, 0.0});
    CHECK(d == std::numeric_limits<double>::infinity());
}

// ============================================================================
// 3. RAY EXACTLY ALONG DIAGONAL
// ============================================================================

TEST_CASE("Ray adversarial: exact 45° diagonal from various positions") {
    double s = 1.0 / std::sqrt(2.0);

    // From (-50, -50) toward (+,+) corner
    double d = rayDistanceToSquareWalls({-50.0, -50.0}, {s, s});
    CHECK(std::isfinite(d));
    CHECK(d > 0.0);
    // Should hit the wall at (72, y) or (x, 72)
    double end_x = -50.0 + d * s;
    double end_y = -50.0 + d * s;
    CHECK((std::fabs(end_x - 72.0) < 0.01 || std::fabs(end_y - 72.0) < 0.01));
}

// ============================================================================
// 4. RAY_DISTANCE_WITH_OFFSET: ROBOT AT EXACT BOUNDARY
// ============================================================================

TEST_CASE("Ray adversarial: robot at effective boundary, sensors shoot in all directions") {
    double limit = 64.0;  // field_half - robot_radius

    // At top boundary, heading 0
    for (double heading : {0.0, 90.0, 180.0, 270.0}) {
        double d;

        d = ray_distance_with_offset({0.0, limit}, heading, kFrontOffset, kFrontAngle);
        INFO("top boundary, heading=" << heading << " front sensor");
        CHECK((std::isfinite(d) || d == std::numeric_limits<double>::infinity()));
        if (std::isfinite(d)) CHECK(d >= 0.0);

        d = ray_distance_with_offset({0.0, limit}, heading, kBackOffset, kBackAngle);
        INFO("top boundary, heading=" << heading << " back sensor");
        CHECK((std::isfinite(d) || d == std::numeric_limits<double>::infinity()));
        if (std::isfinite(d)) CHECK(d >= 0.0);

        d = ray_distance_with_offset({0.0, limit}, heading, kLeftOffset, kLeftAngle);
        CHECK((std::isfinite(d) || d == std::numeric_limits<double>::infinity()));

        d = ray_distance_with_offset({0.0, limit}, heading, kRightOffset, kRightAngle);
        CHECK((std::isfinite(d) || d == std::numeric_limits<double>::infinity()));
    }
}

TEST_CASE("Ray adversarial: robot at all 4 boundaries produces valid readings") {
    double limit = 64.0;
    double positions[][2] = {
        {0.0, limit}, {0.0, -limit}, {limit, 0.0}, {-limit, 0.0}
    };

    for (auto& pos : positions) {
        for (double heading : {0.0, 45.0, 90.0, 135.0, 180.0, 225.0, 270.0, 315.0}) {
            double d = ray_distance_with_offset({pos[0], pos[1]}, heading,
                                                kFrontOffset, kFrontAngle);
            INFO("pos=(" << pos[0] << "," << pos[1] << ") heading=" << heading);
            CHECK((std::isfinite(d) || d == std::numeric_limits<double>::infinity()));
            if (std::isfinite(d)) CHECK(d >= 0.0);
        }
    }
}

// ============================================================================
// 5. OBSTACLE RAY CAST: DEGENERATE OBSTACLES
// ============================================================================

TEST_CASE("Ray obstacle adversarial: zero-size obstacle") {
    Vec2 pos{0.0, 0.0};
    Vec2 offset{0.0, 0.0};
    std::vector<sim::AABB> obs = {{10.0, 10.0, 10.0, 10.0}};  // point obstacle

    double d = ray::ray_distance_with_obstacles(pos, 0.0, offset, 0.0, wrap_obs(obs));
    // Should just hit the wall since point obstacle has no surface
    CHECK(std::isfinite(d));
    CHECK(d > 0.0);
}

TEST_CASE("Ray obstacle adversarial: obstacle covering entire field") {
    Vec2 pos{0.0, 0.0};
    Vec2 offset{0.0, 0.0};
    std::vector<sim::AABB> obs = {{-72.0, -72.0, 72.0, 72.0}};

    double d = ray::ray_distance_with_obstacles(pos, 0.0, offset, 0.0, wrap_obs(obs));
    // Robot is inside the obstacle => should return 0
    CHECK(d == doctest::Approx(0.0).epsilon(1e-6));
}

TEST_CASE("Ray obstacle adversarial: obstacle behind robot") {
    Vec2 pos{0.0, 0.0};
    Vec2 offset{0.0, 0.0};
    // Ray shoots +Y (heading 0, angle 0), obstacle is at negative Y
    std::vector<sim::AABB> obs = {{-5.0, -30.0, 5.0, -20.0}};

    double d = ray::ray_distance_with_obstacles(pos, 0.0, offset, 0.0, wrap_obs(obs));
    // Should hit the wall, not the obstacle (behind the ray)
    CHECK(d == doctest::Approx(72.0).epsilon(1e-6));
}

TEST_CASE("Ray obstacle adversarial: obstacle exactly at ray origin") {
    Vec2 pos{0.0, 0.0};
    Vec2 offset{0.0, 0.0};
    // Obstacle edge is exactly at origin
    std::vector<sim::AABB> obs = {{0.0, 0.0, 10.0, 10.0}};

    double d = ray::ray_distance_with_obstacles(pos, 0.0, offset, 0.0, wrap_obs(obs));
    // Origin is on the edge of the obstacle
    CHECK(d == doctest::Approx(0.0).epsilon(1e-3));
}

TEST_CASE("Ray obstacle adversarial: many obstacles in a line") {
    Vec2 pos{0.0, 0.0};
    Vec2 offset{0.0, 0.0};
    // 10 obstacles along the +Y axis, each 2 inches wide
    std::vector<sim::AABB> obs;
    for (int i = 0; i < 10; i++) {
        double y_start = 10.0 + i * 5.0;
        obs.push_back({-1.0, y_start, 1.0, y_start + 2.0});
    }

    double d = ray::ray_distance_with_obstacles(pos, 0.0, offset, 0.0, wrap_obs(obs));
    // Should hit the first one at y=10
    CHECK(d == doctest::Approx(10.0).epsilon(1e-6));
}

// ============================================================================
// 6. ROTATED SENSOR OFFSET NUMERICAL PRECISION
//    At headings that produce near-zero sin/cos, verify offset rotation
//    doesn't introduce significant error.
// ============================================================================

TEST_CASE("Ray adversarial: rotateOffsetByHeading at heading 0 is exact identity") {
    auto r = rotateOffsetByHeading(kFrontOffset, 0.0);
    CHECK(r.x == doctest::Approx(kFrontOffset.x).epsilon(1e-10));
    CHECK(r.y == doctest::Approx(kFrontOffset.y).epsilon(1e-10));
}

TEST_CASE("Ray adversarial: rotateOffsetByHeading 360 equals heading 0") {
    auto r0 = rotateOffsetByHeading(kLeftOffset, 0.0);
    auto r360 = rotateOffsetByHeading(kLeftOffset, 360.0);
    CHECK(r0.x == doctest::Approx(r360.x).epsilon(1e-8));
    CHECK(r0.y == doctest::Approx(r360.y).epsilon(1e-8));
}

TEST_CASE("Ray adversarial: full rotation returns to original") {
    // Rotate by 90 four times should return to original
    Vec2 offset = {5.0, 3.0};
    Vec2 result = offset;
    for (int i = 0; i < 4; i++) {
        result = rotateOffsetByHeading(result, 90.0);
    }
    // After 4 rotations of 90° each... but note rotateOffsetByHeading
    // applies a SINGLE rotation, not cumulative. Test the math:
    auto r = rotateOffsetByHeading(offset, 360.0);
    CHECK(r.x == doctest::Approx(offset.x).epsilon(1e-8));
    CHECK(r.y == doctest::Approx(offset.y).epsilon(1e-8));
}

// ============================================================================
// 7. WALL HIT IDENTIFICATION AT EXACT DIAGONALS
// ============================================================================

TEST_CASE("Ray adversarial: rayWallHit at exact 45° diagonal") {
    // At 45° from center, should hit either Top or Right (implementation-dependent)
    double s = 1.0 / std::sqrt(2.0);
    auto wall = rayWallHit({0.0, 0.0}, {s, s});
    CHECK((wall == WallId::Top || wall == WallId::Right));
}

TEST_CASE("Ray adversarial: rayWallHit from near a corner") {
    // Near top-right corner, shooting +X should hit Right wall
    auto wall = rayWallHit({70.0, 70.0}, {1.0, 0.0});
    CHECK(wall == WallId::Right);

    // Near top-right corner, shooting +Y should hit Top wall
    wall = rayWallHit({70.0, 70.0}, {0.0, 1.0});
    CHECK(wall == WallId::Top);
}

// ============================================================================
// 8. SENSOR READINGS AT EVERY 1° HEADING INCREMENT
//    Exhaustive heading sweep to catch any trig discontinuities.
// ============================================================================

TEST_CASE("Ray adversarial: all sensors valid at every 1-degree heading from center") {
    for (int deg = 0; deg < 360; deg++) {
        double heading = static_cast<double>(deg);

        double dF = ray_distance_with_offset({0, 0}, heading, kFrontOffset, kFrontAngle);
        double dB = ray_distance_with_offset({0, 0}, heading, kBackOffset,  kBackAngle);
        double dL = ray_distance_with_offset({0, 0}, heading, kLeftOffset,  kLeftAngle);
        double dR = ray_distance_with_offset({0, 0}, heading, kRightOffset, kRightAngle);

        INFO("heading = " << deg);
        CHECK(std::isfinite(dF));
        CHECK(std::isfinite(dB));
        CHECK(std::isfinite(dL));
        CHECK(std::isfinite(dR));
        CHECK(dF > 0.0);
        CHECK(dB > 0.0);
        CHECK(dL > 0.0);
        CHECK(dR > 0.0);
    }
}

// ============================================================================
// 9. SENSOR DISTANCE CONSISTENCY: OPPOSITE SENSORS SUM CONSTRAINT
//    For a robot at center heading 0, front + back ~= field height (144 - offsets),
//    left + right ~= field width (144 - offsets). Verify this approximately holds
//    at various positions.
// ============================================================================

TEST_CASE("Ray adversarial: front+back distance reasonable at center heading 0") {
    double dF = ray_distance_with_offset({0, 0}, 0.0, kFrontOffset, kFrontAngle);
    double dB = ray_distance_with_offset({0, 0}, 0.0, kBackOffset,  kBackAngle);

    // Front + back should be roughly field height minus sensor offsets
    double sum = dF + dB;
    CHECK(sum > 120.0);
    CHECK(sum < 150.0);  // 144 total - various sensor offsets
}

TEST_CASE("Ray adversarial: left+right distance reasonable at center heading 0") {
    double dL = ray_distance_with_offset({0, 0}, 0.0, kLeftOffset,  kLeftAngle);
    double dR = ray_distance_with_offset({0, 0}, 0.0, kRightOffset, kRightAngle);

    double sum = dL + dR;
    CHECK(sum > 120.0);
    CHECK(sum < 150.0);
}

// ============================================================================
// 10. OBSTACLE AT EVERY WALL
// ============================================================================

TEST_CASE("Ray obstacle adversarial: obstacles on all 4 walls still return valid distances") {
    Vec2 pos{0.0, 0.0};
    Vec2 offset{0.0, 0.0};
    std::vector<sim::AABB> obs = {
        {-5, 60, 5, 72},    // top wall
        {-5, -72, 5, -60},  // bottom wall
        {60, -5, 72, 5},    // right wall
        {-72, -5, -60, 5},  // left wall
    };

    for (double heading : {0.0, 90.0, 180.0, 270.0}) {
        double d = ray::ray_distance_with_obstacles(pos, heading, offset, 0.0, wrap_obs(obs));
        INFO("heading=" << heading);
        CHECK(std::isfinite(d));
        CHECK(d > 0.0);
        CHECK(d <= 72.0);  // obstacle is closer than wall
    }
}
