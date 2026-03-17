#include "doctest/doctest.h"
#include "distance_localization.hpp"
#include <cmath>
#include <limits>

using namespace distance_loc;

// Sensor mounts from VEX robot (EDD spec)
static const Vec2   kLeftOffset  = { -6.043, -1.811 };
static const double kLeftAngle   = -90.0;

static const Vec2   kRightOffset = {  6.004, -2.382 };
static const double kRightAngle  =  90.0;

static const Vec2   kFrontOffset = {  3.268,  5.512 };
static const double kFrontAngle  =   0.0;

static const Vec2   kBackOffset  = { -3.878, -4.055 };
static const double kBackAngle   = 180.0;

// ============================================================================
// deg2rad
// ============================================================================

TEST_CASE("deg2rad: basic conversions") {
    CHECK(deg2rad(0.0)   == doctest::Approx(0.0));
    CHECK(deg2rad(90.0)  == doctest::Approx(M_PI / 2.0));
    CHECK(deg2rad(180.0) == doctest::Approx(M_PI));
    CHECK(deg2rad(360.0) == doctest::Approx(2.0 * M_PI));
    CHECK(deg2rad(-90.0) == doctest::Approx(-M_PI / 2.0));
}

// ============================================================================
// rotateOffsetByHeading
// ============================================================================

TEST_CASE("rotateOffsetByHeading: heading 0 is identity") {
    auto r = rotateOffsetByHeading({3.0, 4.0}, 0.0);
    CHECK(r.x == doctest::Approx(3.0).epsilon(1e-9));
    CHECK(r.y == doctest::Approx(4.0).epsilon(1e-9));
}

TEST_CASE("rotateOffsetByHeading: heading 90 CW") {
    // VEX CW: 90 deg CW. t = deg2rad(-90) = -pi/2, cos=0, sin=-1
    // (1,0) -> x'=1*0-0*(-1)=0, y'=1*(-1)+0*0=-1
    auto r = rotateOffsetByHeading({1.0, 0.0}, 90.0);
    CHECK(r.x == doctest::Approx(0.0).epsilon(1e-6));
    CHECK(r.y == doctest::Approx(-1.0).epsilon(1e-6));
}

TEST_CASE("rotateOffsetByHeading: heading 90 on forward offset") {
    // (0,1) at heading 90 CW: x'=0*0-1*(-1)=1, y'=0*(-1)+1*0=0
    auto r = rotateOffsetByHeading({0.0, 1.0}, 90.0);
    CHECK(r.x == doctest::Approx(1.0).epsilon(1e-6));
    CHECK(r.y == doctest::Approx(0.0).epsilon(1e-6));
}

// ============================================================================
// rayDistanceToSquareWalls
// ============================================================================

TEST_CASE("rayDistanceToSquareWalls: center to all 4 walls") {
    CHECK(rayDistanceToSquareWalls({0, 0}, {0,  1}) == doctest::Approx(72.0));
    CHECK(rayDistanceToSquareWalls({0, 0}, {0, -1}) == doctest::Approx(72.0));
    CHECK(rayDistanceToSquareWalls({0, 0}, {1,  0}) == doctest::Approx(72.0));
    CHECK(rayDistanceToSquareWalls({0, 0}, {-1, 0}) == doctest::Approx(72.0));
}

TEST_CASE("rayDistanceToSquareWalls: off-center") {
    CHECK(rayDistanceToSquareWalls({50, 0}, {1, 0}) == doctest::Approx(22.0));
}

TEST_CASE("rayDistanceToSquareWalls: diagonal from center") {
    double s = 1.0 / std::sqrt(2.0);
    double d = rayDistanceToSquareWalls({0, 0}, {s, s});
    CHECK(d == doctest::Approx(72.0 * std::sqrt(2.0)).epsilon(1e-6));
}

TEST_CASE("rayDistanceToSquareWalls: origin on wall shooting outward returns infinity") {
    double d = rayDistanceToSquareWalls({72.0, 0.0}, {1.0, 0.0});
    CHECK(d == std::numeric_limits<double>::infinity());
}

TEST_CASE("rayDistanceToSquareWalls: origin on wall shooting inward") {
    double d = rayDistanceToSquareWalls({-72.0, 0.0}, {1.0, 0.0});
    CHECK(d == doctest::Approx(144.0));
}

TEST_CASE("rayDistanceToSquareWalls: origin on wall shooting along wall") {
    double d = rayDistanceToSquareWalls({-72.0, 0.0}, {0.0, 1.0});
    CHECK(d == doctest::Approx(72.0));
}

// ============================================================================
// rayWallHit
// ============================================================================

TEST_CASE("rayWallHit: center to each cardinal direction") {
    CHECK(rayWallHit({0, 0}, {0,  1}) == WallId::Top);
    CHECK(rayWallHit({0, 0}, {0, -1}) == WallId::Bottom);
    CHECK(rayWallHit({0, 0}, {1,  0}) == WallId::Right);
    CHECK(rayWallHit({0, 0}, {-1, 0}) == WallId::Left);
}

TEST_CASE("rayWallHit: off-center") {
    CHECK(rayWallHit({50, 0}, {1, 0}) == WallId::Right);
    CHECK(rayWallHit({-50, 0}, {-1, 0}) == WallId::Left);
}

TEST_CASE("rayWallHit: no hit returns None") {
    CHECK(rayWallHit({72.0, 0.0}, {1.0, 0.0}) == WallId::None);
}

// ============================================================================
// sensorWallHit
// ============================================================================

TEST_CASE("sensorWallHit: all sensors at heading 0 hit expected walls") {
    CHECK(sensorWallHit({0, 0}, 0.0, kFrontOffset, kFrontAngle) == WallId::Top);
    CHECK(sensorWallHit({0, 0}, 0.0, kBackOffset,  kBackAngle)  == WallId::Bottom);
    CHECK(sensorWallHit({0, 0}, 0.0, kLeftOffset,  kLeftAngle)  == WallId::Left);
    CHECK(sensorWallHit({0, 0}, 0.0, kRightOffset, kRightAngle) == WallId::Right);
}

// ============================================================================
// ray_distance_with_offset
// ============================================================================

TEST_CASE("ray_distance_with_offset: center heading 0, front sensor") {
    // Front sensor at (3.268, 5.512), angle 0. At heading 0, shoots +Y.
    // Sensor origin y = 5.512. Distance to top wall = 72 - 5.512 = 66.488.
    double d = ray_distance_with_offset({0, 0}, 0.0, kFrontOffset, kFrontAngle);
    CHECK(d == doctest::Approx(72.0 - 5.512).epsilon(0.01));
}

TEST_CASE("ray_distance_with_offset: center heading 90, right sensor") {
    // At heading 90 CW, robot faces +X. Right sensor angle = 90.
    // Ray angle = 90 + 90 = 180 deg -> direction (sin(180), cos(180)) = (0, -1).
    // Right offset rotated: (-2.382, -6.004). Sensor origin y = -6.004.
    // Distance to bottom wall = 72 - 6.004 = 65.996.
    double d = ray_distance_with_offset({0, 0}, 90.0, kRightOffset, kRightAngle);
    CHECK(d == doctest::Approx(72.0 - 6.004).epsilon(0.01));
}

TEST_CASE("ray_distance_with_offset: all sensors at center heading 0") {
    double dF = ray_distance_with_offset({0, 0}, 0.0, kFrontOffset, kFrontAngle);
    double dB = ray_distance_with_offset({0, 0}, 0.0, kBackOffset,  kBackAngle);
    double dL = ray_distance_with_offset({0, 0}, 0.0, kLeftOffset,  kLeftAngle);
    double dR = ray_distance_with_offset({0, 0}, 0.0, kRightOffset, kRightAngle);

    CHECK(dF == doctest::Approx(72.0 - 5.512).epsilon(0.01));
    CHECK(dB == doctest::Approx(72.0 - 4.055).epsilon(0.01));
    CHECK(dL == doctest::Approx(72.0 - 6.043).epsilon(0.01));
    CHECK(dR == doctest::Approx(72.0 - 6.004).epsilon(0.01));
}

TEST_CASE("ray_distance_with_offset: all sensors at center headings 90/180/270") {
    for (double heading : {90.0, 180.0, 270.0}) {
        double dF = ray_distance_with_offset({0, 0}, heading, kFrontOffset, kFrontAngle);
        double dB = ray_distance_with_offset({0, 0}, heading, kBackOffset,  kBackAngle);
        double dL = ray_distance_with_offset({0, 0}, heading, kLeftOffset,  kLeftAngle);
        double dR = ray_distance_with_offset({0, 0}, heading, kRightOffset, kRightAngle);

        INFO("heading = " << heading);
        CHECK(dF > 0.0);
        CHECK(dB > 0.0);
        CHECK(dL > 0.0);
        CHECK(dR > 0.0);
        CHECK(dF < 144.0);
        CHECK(dB < 144.0);
        CHECK(dL < 144.0);
        CHECK(dR < 144.0);
    }
}

TEST_CASE("ray_distance_with_offset: 360 heading sweep at center") {
    for (double heading = 0.0; heading < 360.0; heading += 10.0) {
        double dF = ray_distance_with_offset({0, 0}, heading, kFrontOffset, kFrontAngle);
        double dB = ray_distance_with_offset({0, 0}, heading, kBackOffset,  kBackAngle);
        double dL = ray_distance_with_offset({0, 0}, heading, kLeftOffset,  kLeftAngle);
        double dR = ray_distance_with_offset({0, 0}, heading, kRightOffset, kRightAngle);

        INFO("heading = " << heading);
        CHECK(std::isfinite(dF));
        CHECK(std::isfinite(dB));
        CHECK(std::isfinite(dL));
        CHECK(std::isfinite(dR));
        CHECK(dF > 0.0);
        CHECK(dB > 0.0);
        CHECK(dL > 0.0);
        CHECK(dR > 0.0);
        // From center, max distance is ~72*sqrt(2) ≈ 101.8
        CHECK(dF < 110.0);
        CHECK(dB < 110.0);
        CHECK(dL < 110.0);
        CHECK(dR < 110.0);
    }
}

TEST_CASE("ray_distance_with_offset: robot near left wall, left sensor ~1 inch") {
    // Left sensor offset x = -6.043 at heading 0.
    // Sensor origin x = robot_x + (-6.043). Want ~1 inch from left wall (x=-72).
    // sensor_x = -71 => robot_x = -71 + 6.043 = -64.957
    double d = ray_distance_with_offset({-64.957, 0}, 0.0, kLeftOffset, kLeftAngle);
    CHECK(d == doctest::Approx(1.0).epsilon(0.1));
}

TEST_CASE("ray_distance_with_offset: sensor past wall returns infinity") {
    // Robot at (-71, 0) heading 0: left sensor origin at x = -71 + (-6.043) = -77.043
    // Past the left wall. Ray shoots further left. No intersection.
    double d = ray_distance_with_offset({-71, 0}, 0.0, kLeftOffset, kLeftAngle);
    CHECK(d == std::numeric_limits<double>::infinity());
}
