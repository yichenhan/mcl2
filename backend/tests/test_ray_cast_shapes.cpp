#include "doctest/doctest.h"

#include "distance_localization.hpp"
#include "ray/ray_cast_obstacles.hpp"
#include "sim/field.hpp"

#include "nlohmann/json.hpp"

#include <cmath>
#include <vector>

using namespace distance_loc;

namespace {

double ray_with_shapes(
    const Vec2& pos,
    double heading_deg,
    const std::vector<sim::Obstacle>& obstacles) {
    return ray::ray_distance_with_obstacles(pos, heading_deg, Vec2{0.0, 0.0}, 0.0, obstacles);
}

} // namespace

TEST_CASE("shape ray: circle dead-on hit") {
    const std::vector<sim::Obstacle> obs = {sim::Obstacle{sim::Circle{0.0, 30.0, 5.0}, ""}};
    const double d = ray_with_shapes(Vec2{0.0, 0.0}, 0.0, obs);
    CHECK(d == doctest::Approx(25.0).epsilon(1e-6));
}

TEST_CASE("shape ray: circle miss falls back to wall") {
    const std::vector<sim::Obstacle> obs = {sim::Obstacle{sim::Circle{20.0, 30.0, 3.0}, ""}};
    const double d = ray_with_shapes(Vec2{0.0, 0.0}, 0.0, obs);
    CHECK(d == doctest::Approx(72.0).epsilon(1e-6));
}

TEST_CASE("shape ray: circle tangent") {
    const std::vector<sim::Obstacle> obs = {sim::Obstacle{sim::Circle{5.0, 30.0, 5.0}, ""}};
    const double d = ray_with_shapes(Vec2{0.0, 0.0}, 0.0, obs);
    CHECK(d == doctest::Approx(30.0).epsilon(1e-6));
}

TEST_CASE("shape ray: inside circle is zero") {
    const std::vector<sim::Obstacle> obs = {sim::Obstacle{sim::Circle{0.0, 0.0, 10.0}, ""}};
    const double d = ray_with_shapes(Vec2{0.0, 0.0}, 0.0, obs);
    CHECK(d == doctest::Approx(0.0).epsilon(1e-9));
}

TEST_CASE("shape ray: triangle face hit") {
    const sim::Triangle t{-10.0, 20.0, 10.0, 20.0, 0.0, 30.0};
    const std::vector<sim::Obstacle> obs = {sim::Obstacle{t, ""}};
    const double d = ray_with_shapes(Vec2{0.0, 0.0}, 0.0, obs);
    CHECK(d == doctest::Approx(20.0).epsilon(1e-6));
}

TEST_CASE("shape ray: triangle winding order invariant") {
    const sim::Triangle cw{-10.0, 20.0, 10.0, 20.0, 0.0, 30.0};
    const sim::Triangle ccw{10.0, 20.0, -10.0, 20.0, 0.0, 30.0};
    const double d1 = ray_with_shapes(Vec2{0.0, 0.0}, 0.0, {sim::Obstacle{cw, ""}});
    const double d2 = ray_with_shapes(Vec2{0.0, 0.0}, 0.0, {sim::Obstacle{ccw, ""}});
    CHECK(d1 == doctest::Approx(d2).epsilon(1e-6));
}

TEST_CASE("shape ray: degenerate triangle is ignored") {
    const sim::Triangle deg{0.0, 10.0, 0.0, 20.0, 0.0, 30.0};
    const std::vector<sim::Obstacle> obs = {sim::Obstacle{deg, ""}};
    const double d = ray_with_shapes(Vec2{0.0, 0.0}, 0.0, obs);
    CHECK(d == doctest::Approx(72.0).epsilon(1e-6));
}

TEST_CASE("shape ray: nearest among mixed obstacle types") {
    const sim::AABB rect{-5.0, 40.0, 5.0, 45.0};
    const sim::Circle circle{0.0, 20.0, 3.0};
    const sim::Triangle tri{-10.0, 50.0, 10.0, 50.0, 0.0, 60.0};
    const std::vector<sim::Obstacle> obs = {
        sim::Obstacle{rect, ""},
        sim::Obstacle{circle, ""},
        sim::Obstacle{tri, ""},
    };
    const double d = ray_with_shapes(Vec2{0.0, 0.0}, 0.0, obs);
    CHECK(d == doctest::Approx(17.0).epsilon(1e-6));
}

TEST_CASE("field passability: circle and triangle block points") {
    sim::Field f;
    f.obstacles = {
        sim::Obstacle{sim::Circle{10.0, 10.0, 4.0}, ""},
        sim::Obstacle{sim::Triangle{-10.0, 20.0, -2.0, 20.0, -6.0, 28.0}, ""},
    };
    CHECK_FALSE(f.is_passable(10.0, 10.0));
    CHECK_FALSE(f.is_passable(-6.0, 22.0));
    CHECK(f.is_passable(30.0, 30.0));
}

TEST_CASE("field segment collision: circle and triangle") {
    sim::Field f;
    f.obstacles = {
        sim::Obstacle{sim::Circle{10.0, 0.0, 2.0}, ""},
        sim::Obstacle{sim::Triangle{20.0, -2.0, 24.0, -2.0, 22.0, 4.0}, ""},
    };
    CHECK(f.segment_hits_obstacle(0.0, 0.0, 12.0, 0.0));
    CHECK(f.segment_hits_obstacle(20.0, 0.0, 22.0, 3.0));
    CHECK_FALSE(f.segment_hits_obstacle(-20.0, -20.0, -10.0, -10.0));
}

TEST_CASE("obstacle json round-trip keeps shape and color") {
    const sim::Obstacle in_circle{sim::Circle{1.0, 2.0, 3.0}, "rgba(234, 160, 28, 0.55)"};
    const nlohmann::json j = in_circle;
    const sim::Obstacle out_circle = j.get<sim::Obstacle>();
    REQUIRE(std::holds_alternative<sim::Circle>(out_circle.shape));
    const auto c = std::get<sim::Circle>(out_circle.shape);
    CHECK(c.cx == doctest::Approx(1.0));
    CHECK(c.cy == doctest::Approx(2.0));
    CHECK(c.radius == doctest::Approx(3.0));
    CHECK(out_circle.color == "rgba(234, 160, 28, 0.55)");

    const nlohmann::json legacy_rect = {
        {"min_x", -5.0},
        {"min_y", -4.0},
        {"max_x", 5.0},
        {"max_y", 4.0},
    };
    const sim::Obstacle parsed_legacy = legacy_rect.get<sim::Obstacle>();
    REQUIRE(std::holds_alternative<sim::AABB>(parsed_legacy.shape));
    const auto r = std::get<sim::AABB>(parsed_legacy.shape);
    CHECK(r.min_x == doctest::Approx(-5.0));
    CHECK(r.max_y == doctest::Approx(4.0));
    CHECK(parsed_legacy.color.empty());
}
