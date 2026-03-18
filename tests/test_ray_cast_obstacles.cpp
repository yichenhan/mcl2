#include "doctest/doctest.h"

#include "distance_localization.hpp"
#include "ray/ray_cast_obstacles.hpp"

#include <cmath>
#include <limits>
#include <vector>

using namespace distance_loc;

TEST_CASE("ray obstacle: empty obstacle list equals wall-only ray") {
    Vec2 pos{0.0, 0.0};
    Vec2 offset{0.0, 0.0};
    const double d_wall = ray_distance_with_offset(pos, 0.0, offset, 0.0);
    const double d_obs = ray::ray_distance_with_obstacles(pos, 0.0, offset, 0.0, {});
    CHECK(d_obs == doctest::Approx(d_wall).epsilon(1e-9));
}

TEST_CASE("ray obstacle: obstacle in front shortens measured distance") {
    Vec2 pos{0.0, 0.0};
    Vec2 offset{0.0, 0.0};
    std::vector<sim::AABB> obs = {
        { -5.0, 20.0, 5.0, 30.0 }
    };
    const double d = ray::ray_distance_with_obstacles(pos, 0.0, offset, 0.0, obs);
    CHECK(d == doctest::Approx(20.0).epsilon(1e-6));
}

TEST_CASE("ray obstacle: nearest of multiple obstacles is selected") {
    Vec2 pos{0.0, 0.0};
    Vec2 offset{0.0, 0.0};
    std::vector<sim::AABB> obs = {
        { -5.0, 35.0, 5.0, 45.0 },
        { -5.0, 10.0, 5.0, 15.0 }
    };
    const double d = ray::ray_distance_with_obstacles(pos, 0.0, offset, 0.0, obs);
    CHECK(d == doctest::Approx(10.0).epsilon(1e-6));
}

TEST_CASE("ray obstacle: off-axis obstacle does not affect ray") {
    Vec2 pos{0.0, 0.0};
    Vec2 offset{0.0, 0.0};
    std::vector<sim::AABB> obs = {
        { 10.0, 10.0, 20.0, 20.0 }  // ray along +Y at x=0 misses this
    };
    const double d = ray::ray_distance_with_obstacles(pos, 0.0, offset, 0.0, obs);
    CHECK(d == doctest::Approx(72.0).epsilon(1e-6));
}

TEST_CASE("ray obstacle: robot inside obstacle yields zero distance") {
    Vec2 pos{0.0, 0.0};
    Vec2 offset{0.0, 0.0};
    std::vector<sim::AABB> obs = {
        { -1.0, -1.0, 1.0, 1.0 }
    };
    const double d = ray::ray_distance_with_obstacles(pos, 0.0, offset, 0.0, obs);
    CHECK(d == doctest::Approx(0.0).epsilon(1e-9));
}

TEST_CASE("ray obstacle: diagonal ray intersects obstacle correctly") {
    Vec2 pos{0.0, 0.0};
    Vec2 offset{0.0, 0.0};
    std::vector<sim::AABB> obs = {
        { 20.0, 20.0, 30.0, 30.0 }
    };
    // heading 45 => dir=(sin45,cos45)=(s,s), first hit at (20,20)
    const double d = ray::ray_distance_with_obstacles(pos, 45.0, offset, 0.0, obs);
    CHECK(d == doctest::Approx(20.0 * std::sqrt(2.0)).epsilon(1e-6));
}

TEST_CASE("ray obstacle: sensor offset is applied before intersection") {
    Vec2 pos{0.0, 0.0};
    Vec2 offset{0.0, 10.0};  // start closer to obstacle along +Y
    std::vector<sim::AABB> obs = {
        { -5.0, 20.0, 5.0, 30.0 }
    };
    const double d = ray::ray_distance_with_obstacles(pos, 0.0, offset, 0.0, obs);
    CHECK(d == doctest::Approx(10.0).epsilon(1e-6));
}
