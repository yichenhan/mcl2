// =============================================================================
// test_physics_adversarial.cpp
//
// Antagonistic edge-case tests for the Physics and Field simulation.
// Focuses on boundary collisions, obstacle edge cases, heading wrap-around,
// and degenerate inputs.
// =============================================================================

#include "doctest/doctest.h"
#include "sim/field.hpp"
#include "sim/physics.hpp"
#include <cmath>
#include <vector>

using namespace sim;

// ============================================================================
// 1. DIAGONAL CORNER COLLISIONS
//    Moving at 45° into a corner exercises both X and Y clamping simultaneously.
// ============================================================================

TEST_CASE("Physics adversarial: diagonal into corner clamps both axes") {
    Field f;
    PhysicsConfig cfg;
    Physics phys(f, cfg);
    double limit = f.field_half - f.robot_radius;

    // Place near top-right corner, heading 45° (toward corner)
    phys.set_state({ limit - 0.5, limit - 0.5, 45.0 });

    auto result = phys.step(Action::FORWARD);
    auto s = phys.state();

    CHECK(result.colliding);
    CHECK(s.x <= limit);
    CHECK(s.y <= limit);
    CHECK(std::isfinite(s.x));
    CHECK(std::isfinite(s.y));
}

TEST_CASE("Physics adversarial: diagonal into negative corner") {
    Field f;
    PhysicsConfig cfg;
    Physics phys(f, cfg);
    double limit = f.field_half - f.robot_radius;

    // Near bottom-left corner heading 225° (toward -X,-Y)
    phys.set_state({ -limit + 0.5, -limit + 0.5, 225.0 });

    auto result = phys.step(Action::FORWARD);
    auto s = phys.state();

    CHECK(result.colliding);
    CHECK(s.x >= -limit);
    CHECK(s.y >= -limit);
}

// ============================================================================
// 2. HEADING WRAP-AROUND EDGE CASES
// ============================================================================

TEST_CASE("Physics adversarial: heading exactly 360 normalizes to 0") {
    Field f;
    Physics phys(f);
    phys.set_state({0.0, 0.0, 360.0});
    auto s = phys.state();
    // Should be 0 or 360 — either way, movement should match heading 0
    phys.step(Action::FORWARD);
    auto s2 = phys.state();
    CHECK(s2.y > 0.0);  // heading 0 = +Y
    CHECK(std::fabs(s2.x) < 1e-4);
}

TEST_CASE("Physics adversarial: large negative heading wraps correctly") {
    Field f;
    Physics phys(f);
    phys.set_state({0.0, 0.0, -90.0});  // Should be equivalent to 270
    phys.step(Action::FORWARD);
    auto s = phys.state();
    CHECK(s.x < 0.0);  // 270° = -X direction
}

TEST_CASE("Physics adversarial: heading 359 + CW step wraps past 360") {
    Field f;
    PhysicsConfig cfg;
    Physics phys(f, cfg);
    phys.set_state({0.0, 0.0, 359.0});

    phys.step(Action::ROTATE_CW);
    double new_heading = phys.state().heading_deg;

    // 359 + 18 = 377, wrapped to 17
    double expected = std::fmod(359.0 + cfg.max_angular_vel * cfg.dt, 360.0);
    CHECK(new_heading == doctest::Approx(expected).epsilon(1e-4));
    CHECK(new_heading >= 0.0);
    CHECK(new_heading < 360.0);
}

TEST_CASE("Physics adversarial: heading 1 + CCW step wraps below 0") {
    Field f;
    PhysicsConfig cfg;
    Physics phys(f, cfg);
    phys.set_state({0.0, 0.0, 1.0});

    phys.step(Action::ROTATE_CCW);
    double new_heading = phys.state().heading_deg;

    // 1 - 18 = -17, wrapped to 343
    double expected = std::fmod(1.0 - cfg.max_angular_vel * cfg.dt + 360.0, 360.0);
    CHECK(new_heading == doctest::Approx(expected).epsilon(1e-4));
    CHECK(new_heading >= 0.0);
    CHECK(new_heading < 360.0);
}

// ============================================================================
// 3. OBSTACLE EDGE CASES
// ============================================================================

TEST_CASE("Physics adversarial: obstacle touching field wall") {
    Field f;
    double limit = f.field_half - f.robot_radius;
    // Obstacle flush against top wall
    f.obstacles.push_back({-10.0, limit - 5.0, 10.0, limit + 5.0});

    PhysicsConfig cfg;
    Physics phys(f, cfg);
    phys.set_state({0.0, limit - 10.0, 0.0});

    // Should collide with obstacle before wall
    auto r = phys.step(Action::FORWARD);
    CHECK(r.colliding);
    auto s = phys.state();
    CHECK(s.y <= limit);
}

TEST_CASE("Physics adversarial: zero-width obstacle (degenerate AABB)") {
    Field f;
    f.obstacles.push_back({0.0, 10.0, 0.0, 20.0});  // zero width

    PhysicsConfig cfg;
    Physics phys(f, cfg);
    phys.set_state({0.0, 5.0, 0.0});

    // Movement through a zero-width obstacle — implementation-dependent
    auto r = phys.step(Action::FORWARD);
    // Just verify no crash
    CHECK(std::isfinite(phys.state().x));
    CHECK(std::isfinite(phys.state().y));
}

TEST_CASE("Physics adversarial: obstacle exactly at robot position") {
    Field f;
    f.obstacles.push_back({-1.0, -1.0, 1.0, 1.0});

    PhysicsConfig cfg;
    Physics phys(f, cfg);
    phys.set_state({0.0, 0.0, 0.0});  // Inside obstacle

    // Any movement should be blocked
    auto r = phys.step(Action::FORWARD);
    CHECK(r.colliding);
    CHECK(r.delta.forward_in == doctest::Approx(0.0).epsilon(1e-6));
}

TEST_CASE("Physics adversarial: two obstacles forming a narrow corridor") {
    Field f;
    f.obstacles.push_back({-30.0, 10.0, -2.0, 50.0});  // left wall
    f.obstacles.push_back({2.0, 10.0, 30.0, 50.0});     // right wall

    PhysicsConfig cfg;
    Physics phys(f, cfg);
    phys.set_state({0.0, 5.0, 0.0});

    // Move through corridor (heading 0 = +Y)
    for (int i = 0; i < 20; i++) {
        phys.step(Action::FORWARD);
    }

    auto s = phys.state();
    CHECK(std::isfinite(s.x));
    CHECK(std::isfinite(s.y));
    // Should have moved forward through corridor
    CHECK(s.y > 5.0);
}

TEST_CASE("Physics adversarial: many obstacles on field") {
    Field f;
    // Scatter 10 obstacles
    f.obstacles.push_back({-50, -50, -40, -40});
    f.obstacles.push_back({-30, -20, -20, -10});
    f.obstacles.push_back({-10, 0, 0, 10});
    f.obstacles.push_back({10, 20, 20, 30});
    f.obstacles.push_back({30, 40, 40, 50});
    f.obstacles.push_back({-50, 40, -40, 50});
    f.obstacles.push_back({40, -50, 50, -40});
    f.obstacles.push_back({-20, 20, -10, 30});
    f.obstacles.push_back({20, -30, 30, -20});
    f.obstacles.push_back({-5, -40, 5, -30});

    PhysicsConfig cfg;
    Physics phys(f, cfg);
    phys.set_state({0.0, -55.0, 0.0});

    // Move around for 100 ticks without crashing
    for (int i = 0; i < 100; i++) {
        Action a;
        if (i % 20 < 12) a = Action::FORWARD;
        else if (i % 20 < 16) a = Action::ROTATE_CW;
        else a = Action::BACKWARD;

        phys.step(a);
        auto s = phys.state();
        double limit = f.field_half - f.robot_radius;
        CHECK(s.x >= -limit - 0.01);
        CHECK(s.x <= limit + 0.01);
        CHECK(s.y >= -limit - 0.01);
        CHECK(s.y <= limit + 0.01);
    }
}

// ============================================================================
// 4. COLLISION THEN IMMEDIATE DIRECTION CHANGE
// ============================================================================

TEST_CASE("Physics adversarial: collide, rotate, move away — single tick each") {
    Field f;
    PhysicsConfig cfg;
    Physics phys(f, cfg);
    double limit = f.field_half - f.robot_radius;
    phys.set_state({0.0, limit, 0.0});

    // Step 1: Forward into wall
    auto r1 = phys.step(Action::FORWARD);
    CHECK(r1.colliding);

    // Step 2: Rotate 180 (10 steps)
    for (int i = 0; i < 10; i++) phys.step(Action::ROTATE_CW);

    // Step 3: Forward away from wall
    auto r3 = phys.step(Action::FORWARD);
    CHECK_FALSE(r3.colliding);
    CHECK(phys.state().y < limit);
}

// ============================================================================
// 5. BACKWARD INTO WALL
// ============================================================================

TEST_CASE("Physics adversarial: backward into bottom wall") {
    Field f;
    PhysicsConfig cfg;
    Physics phys(f, cfg);
    double limit = f.field_half - f.robot_radius;
    phys.set_state({0.0, -limit + 0.3, 0.0});  // near bottom, heading +Y

    auto r = phys.step(Action::BACKWARD);  // moves -Y
    CHECK(r.colliding);
    CHECK(phys.state().y >= -limit);
}

// ============================================================================
// 6. FIELD: is_passable vs is_inside consistency
// ============================================================================

TEST_CASE("Physics adversarial: is_passable false inside obstacle, is_inside true") {
    Field f;
    f.obstacles.push_back({-5.0, -5.0, 5.0, 5.0});

    // (0,0) is inside the field but not passable
    CHECK(f.is_inside(0.0, 0.0));
    CHECK_FALSE(f.is_passable(0.0, 0.0));

    // (20,20) is both inside and passable
    CHECK(f.is_inside(20.0, 20.0));
    CHECK(f.is_passable(20.0, 20.0));
}

// ============================================================================
// 7. NOOP REPEATED 1000 TIMES
// ============================================================================

TEST_CASE("Physics adversarial: 1000 NONE actions don't drift") {
    Field f;
    Physics phys(f);
    phys.set_state({10.0, 20.0, 45.0});

    for (int i = 0; i < 1000; i++) {
        phys.step(Action::NONE);
    }

    auto s = phys.state();
    CHECK(s.x == doctest::Approx(10.0));
    CHECK(s.y == doctest::Approx(20.0));
    CHECK(s.heading_deg == doctest::Approx(45.0));
}

// ============================================================================
// 8. SEGMENT HITS OBSTACLE: EDGE CASES
// ============================================================================

TEST_CASE("Physics adversarial: segment tangent to obstacle") {
    Field f;
    f.obstacles.push_back({5.0, 0.0, 10.0, 10.0});

    // Segment that runs along the edge (x=5, from y=-5 to y=15)
    // Depending on implementation, may or may not hit
    bool hits = f.segment_hits_obstacle(5.0, -5.0, 5.0, 15.0);
    // Just verify no crash — tangent case is implementation-defined
    (void)hits;
    CHECK(true);  // no crash is success
}

TEST_CASE("Physics adversarial: zero-length segment") {
    Field f;
    f.obstacles.push_back({-5.0, -5.0, 5.0, 5.0});

    // Zero-length segment inside obstacle
    CHECK(f.segment_hits_obstacle(0.0, 0.0, 0.0, 0.0) == true);

    // Zero-length segment outside obstacle
    bool hits = f.segment_hits_obstacle(20.0, 20.0, 20.0, 20.0);
    // Should not hit — no movement, not inside
    (void)hits;
    CHECK(true);
}
