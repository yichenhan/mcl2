#include "doctest/doctest.h"
#include "sim/field.hpp"
#include "sim/physics.hpp"
#include <cmath>

using namespace sim;

// ============================================================================
// Field: is_inside
// ============================================================================

TEST_CASE("Field is_inside: center is inside") {
    Field f;
    CHECK(f.is_inside(0.0, 0.0));
}

TEST_CASE("Field is_inside: points within bounds are inside") {
    Field f;
    CHECK(f.is_inside(50.0, 50.0));
    CHECK(f.is_inside(-50.0, -50.0));
    CHECK(f.is_inside(63.0, 63.0));   // within field_half - robot_radius = 64
    CHECK(f.is_inside(-63.0, -63.0));
}

TEST_CASE("Field is_inside: points at effective boundary") {
    Field f;
    double limit = f.field_half - f.robot_radius;  // 72 - 8 = 64
    CHECK(f.is_inside(limit, 0.0));
    CHECK(f.is_inside(-limit, 0.0));
    CHECK(f.is_inside(0.0, limit));
    CHECK(f.is_inside(0.0, -limit));
}

TEST_CASE("Field is_inside: points outside effective boundary") {
    Field f;
    double outside = f.field_half - f.robot_radius + 0.1;  // 64.1
    CHECK_FALSE(f.is_inside(outside, 0.0));
    CHECK_FALSE(f.is_inside(-outside, 0.0));
    CHECK_FALSE(f.is_inside(0.0, outside));
    CHECK_FALSE(f.is_inside(0.0, -outside));
}

TEST_CASE("Field is_inside: corners outside") {
    Field f;
    CHECK_FALSE(f.is_inside(72.0, 72.0));
    CHECK_FALSE(f.is_inside(-72.0, -72.0));
}

// ============================================================================
// Field: clamp
// ============================================================================

TEST_CASE("Field clamp: point inside is unchanged") {
    Field f;
    double x = 10.0, y = -20.0;
    f.clamp(x, y);
    CHECK(x == doctest::Approx(10.0));
    CHECK(y == doctest::Approx(-20.0));
}

TEST_CASE("Field clamp: point outside is clamped to effective boundary") {
    Field f;
    double limit = f.field_half - f.robot_radius;  // 64
    double x = 100.0, y = -100.0;
    f.clamp(x, y);
    CHECK(x == doctest::Approx(limit));
    CHECK(y == doctest::Approx(-limit));
}

TEST_CASE("Field clamp: point at boundary stays at boundary") {
    Field f;
    double limit = f.field_half - f.robot_radius;
    double x = limit, y = -limit;
    f.clamp(x, y);
    CHECK(x == doctest::Approx(limit));
    CHECK(y == doctest::Approx(-limit));
}

TEST_CASE("Field clamp: each axis is clamped independently") {
    Field f;
    double x = 100.0, y = 5.0;
    f.clamp(x, y);
    double limit = f.field_half - f.robot_radius;
    CHECK(x == doctest::Approx(limit));
    CHECK(y == doctest::Approx(5.0));
}

// ============================================================================
// Physics: construction and initial state
// ============================================================================

TEST_CASE("Physics: default state is at origin heading 0") {
    Field f;
    Physics phys(f);
    auto s = phys.state();
    CHECK(s.x == doctest::Approx(0.0));
    CHECK(s.y == doctest::Approx(0.0));
    CHECK(s.heading_deg == doctest::Approx(0.0));
}

TEST_CASE("Physics: set_state changes state") {
    Field f;
    Physics phys(f);
    phys.set_state({ 10.0, 20.0, 90.0 });
    auto s = phys.state();
    CHECK(s.x == doctest::Approx(10.0));
    CHECK(s.y == doctest::Approx(20.0));
    CHECK(s.heading_deg == doctest::Approx(90.0));
}

// ============================================================================
// Physics: NONE action
// ============================================================================

TEST_CASE("Physics step NONE: state unchanged, delta is zero") {
    Field f;
    Physics phys(f);
    phys.set_state({ 10.0, 20.0, 45.0 });
    auto result = phys.step(Action::NONE);

    CHECK(result.delta.forward_in == doctest::Approx(0.0));
    CHECK(result.delta.rotation_deg == doctest::Approx(0.0));
    CHECK_FALSE(result.colliding);

    auto s = phys.state();
    CHECK(s.x == doctest::Approx(10.0));
    CHECK(s.y == doctest::Approx(20.0));
    CHECK(s.heading_deg == doctest::Approx(45.0));
}

// ============================================================================
// Physics: FORWARD movement
// ============================================================================

TEST_CASE("Physics step FORWARD heading 0: moves in +Y") {
    Field f;
    PhysicsConfig cfg;
    Physics phys(f, cfg);
    phys.set_state({ 0.0, 0.0, 0.0 });

    auto result = phys.step(Action::FORWARD);
    double expected_dist = cfg.max_velocity * cfg.dt;  // 36 * 0.05 = 1.8

    CHECK(result.delta.forward_in == doctest::Approx(expected_dist).epsilon(1e-6));
    CHECK(result.delta.rotation_deg == doctest::Approx(0.0));
    CHECK_FALSE(result.colliding);

    auto s = phys.state();
    CHECK(s.x == doctest::Approx(0.0).epsilon(1e-6));
    CHECK(s.y == doctest::Approx(expected_dist).epsilon(1e-6));
}

TEST_CASE("Physics step FORWARD heading 90: moves in +X") {
    Field f;
    PhysicsConfig cfg;
    Physics phys(f, cfg);
    phys.set_state({ 0.0, 0.0, 90.0 });

    phys.step(Action::FORWARD);
    double d = cfg.max_velocity * cfg.dt;

    auto s = phys.state();
    CHECK(s.x == doctest::Approx(d).epsilon(1e-6));
    CHECK(s.y == doctest::Approx(0.0).epsilon(1e-6));
}

TEST_CASE("Physics step FORWARD heading 45: moves diagonally") {
    Field f;
    PhysicsConfig cfg;
    Physics phys(f, cfg);
    phys.set_state({ 0.0, 0.0, 45.0 });

    phys.step(Action::FORWARD);
    double d = cfg.max_velocity * cfg.dt;
    double diag = d / std::sqrt(2.0);

    auto s = phys.state();
    CHECK(s.x == doctest::Approx(diag).epsilon(1e-4));
    CHECK(s.y == doctest::Approx(diag).epsilon(1e-4));
}

TEST_CASE("Physics step FORWARD heading 180: moves in -Y") {
    Field f;
    PhysicsConfig cfg;
    Physics phys(f, cfg);
    phys.set_state({ 0.0, 0.0, 180.0 });

    phys.step(Action::FORWARD);
    double d = cfg.max_velocity * cfg.dt;

    auto s = phys.state();
    CHECK(s.x == doctest::Approx(0.0).epsilon(1e-6));
    CHECK(s.y == doctest::Approx(-d).epsilon(1e-6));
}

// ============================================================================
// Physics: BACKWARD movement
// ============================================================================

TEST_CASE("Physics step BACKWARD heading 0: moves in -Y") {
    Field f;
    PhysicsConfig cfg;
    Physics phys(f, cfg);
    phys.set_state({ 0.0, 0.0, 0.0 });

    auto result = phys.step(Action::BACKWARD);
    double d = cfg.max_velocity * cfg.dt;

    auto s = phys.state();
    CHECK(s.x == doctest::Approx(0.0).epsilon(1e-6));
    CHECK(s.y == doctest::Approx(-d).epsilon(1e-6));
    CHECK(result.delta.forward_in == doctest::Approx(-d).epsilon(1e-6));
}

// ============================================================================
// Physics: ROTATION
// ============================================================================

TEST_CASE("Physics step ROTATE_CW: heading increases") {
    Field f;
    PhysicsConfig cfg;
    Physics phys(f, cfg);
    phys.set_state({ 0.0, 0.0, 0.0 });

    auto result = phys.step(Action::ROTATE_CW);
    double expected_rot = cfg.max_angular_vel * cfg.dt;  // 360 * 0.05 = 18

    CHECK(result.delta.rotation_deg == doctest::Approx(expected_rot).epsilon(1e-6));
    CHECK(result.delta.forward_in == doctest::Approx(0.0));

    auto s = phys.state();
    CHECK(s.heading_deg == doctest::Approx(expected_rot).epsilon(1e-6));
    CHECK(s.x == doctest::Approx(0.0));
    CHECK(s.y == doctest::Approx(0.0));
}

TEST_CASE("Physics step ROTATE_CCW: heading decreases (wraps)") {
    Field f;
    PhysicsConfig cfg;
    Physics phys(f, cfg);
    phys.set_state({ 0.0, 0.0, 0.0 });

    auto result = phys.step(Action::ROTATE_CCW);
    double expected_rot = -(cfg.max_angular_vel * cfg.dt);

    CHECK(result.delta.rotation_deg == doctest::Approx(expected_rot).epsilon(1e-6));

    auto s = phys.state();
    // 0 - 18 = -18, wrapped to 342
    double wrapped = std::fmod(expected_rot, 360.0);
    if (wrapped < 0) wrapped += 360.0;
    CHECK(s.heading_deg == doctest::Approx(wrapped).epsilon(1e-6));
}

TEST_CASE("Physics rotation: 20 CW steps = 360 degrees wraps to 0") {
    Field f;
    PhysicsConfig cfg;
    Physics phys(f, cfg);
    phys.set_state({ 0.0, 0.0, 0.0 });

    for (int i = 0; i < 20; i++) {
        phys.step(Action::ROTATE_CW);
    }
    // 20 * 18 = 360, should wrap to 0
    auto s = phys.state();
    CHECK(s.heading_deg == doctest::Approx(0.0).epsilon(1e-6));
}

// ============================================================================
// Physics: wall collision
// ============================================================================

TEST_CASE("Physics FORWARD into wall: clamped, collision flag true") {
    Field f;
    PhysicsConfig cfg;
    Physics phys(f, cfg);
    double limit = f.field_half - f.robot_radius;  // 64
    phys.set_state({ 0.0, limit - 0.5, 0.0 });  // 0.5" from wall, heading +Y

    auto result = phys.step(Action::FORWARD);

    CHECK(result.colliding);
    auto s = phys.state();
    CHECK(s.y <= limit);
    // forward_in should reflect actual distance moved, not requested
    CHECK(result.delta.forward_in < cfg.max_velocity * cfg.dt);
    CHECK(result.delta.forward_in >= 0.0);
}

TEST_CASE("Physics FORWARD at wall: stays clamped, collision flag true") {
    Field f;
    PhysicsConfig cfg;
    Physics phys(f, cfg);
    double limit = f.field_half - f.robot_radius;
    phys.set_state({ 0.0, limit, 0.0 });

    auto result = phys.step(Action::FORWARD);

    CHECK(result.colliding);
    auto s = phys.state();
    CHECK(s.y == doctest::Approx(limit));
    CHECK(result.delta.forward_in == doctest::Approx(0.0).epsilon(1e-6));
}

TEST_CASE("Physics FORWARD away from wall: no collision") {
    Field f;
    PhysicsConfig cfg;
    Physics phys(f, cfg);
    double limit = f.field_half - f.robot_radius;
    phys.set_state({ 0.0, limit, 180.0 });  // at wall, heading away

    auto result = phys.step(Action::FORWARD);

    CHECK_FALSE(result.colliding);
    auto s = phys.state();
    CHECK(s.y < limit);
}

TEST_CASE("Physics: collision on X-axis wall") {
    Field f;
    PhysicsConfig cfg;
    Physics phys(f, cfg);
    double limit = f.field_half - f.robot_radius;
    phys.set_state({ limit - 0.1, 0.0, 90.0 });  // near right wall, heading +X

    auto result = phys.step(Action::FORWARD);

    CHECK(result.colliding);
    auto s = phys.state();
    CHECK(s.x <= limit);
}

// ============================================================================
// Physics: multi-step movement
// ============================================================================

TEST_CASE("Physics: 10 FORWARD steps from center heading 0 accumulate correctly") {
    Field f;
    PhysicsConfig cfg;
    Physics phys(f, cfg);
    phys.set_state({ 0.0, 0.0, 0.0 });

    double total_forward = 0.0;
    for (int i = 0; i < 10; i++) {
        auto result = phys.step(Action::FORWARD);
        total_forward += result.delta.forward_in;
    }

    double expected = 10.0 * cfg.max_velocity * cfg.dt;  // 18.0
    auto s = phys.state();
    CHECK(s.y == doctest::Approx(expected).epsilon(1e-4));
    CHECK(total_forward == doctest::Approx(expected).epsilon(1e-4));
}

TEST_CASE("Physics: rotate then move produces correct position") {
    Field f;
    PhysicsConfig cfg;
    Physics phys(f, cfg);
    phys.set_state({ 0.0, 0.0, 0.0 });

    // Rotate 90 CW (5 steps of 18 deg each)
    for (int i = 0; i < 5; i++) {
        phys.step(Action::ROTATE_CW);
    }
    CHECK(phys.state().heading_deg == doctest::Approx(90.0).epsilon(1e-6));

    // Move forward (now heading +X)
    phys.step(Action::FORWARD);
    double d = cfg.max_velocity * cfg.dt;

    auto s = phys.state();
    CHECK(s.x == doctest::Approx(d).epsilon(1e-4));
    CHECK(s.y == doctest::Approx(0.0).epsilon(1e-4));
}

TEST_CASE("Physics: move in a square returns near origin") {
    Field f;
    PhysicsConfig cfg;
    Physics phys(f, cfg);
    phys.set_state({ 0.0, 0.0, 0.0 });

    int steps_per_side = 5;
    for (int side = 0; side < 4; side++) {
        for (int i = 0; i < steps_per_side; i++) {
            phys.step(Action::FORWARD);
        }
        // Rotate 90 CW (5 steps of 18 deg)
        for (int i = 0; i < 5; i++) {
            phys.step(Action::ROTATE_CW);
        }
    }

    auto s = phys.state();
    CHECK(s.x == doctest::Approx(0.0).epsilon(0.5));
    CHECK(s.y == doctest::Approx(0.0).epsilon(0.5));
    CHECK(s.heading_deg == doctest::Approx(0.0).epsilon(1e-4));
}

// ============================================================================
// Physics: heading convention
//   VEX: 0=+Y (forward), CW positive
//   dx = forward * sin(heading), dy = forward * cos(heading)
// ============================================================================

TEST_CASE("Physics heading convention: 0 deg = +Y") {
    Field f;
    Physics phys(f);
    phys.set_state({ 0.0, 0.0, 0.0 });
    phys.step(Action::FORWARD);
    CHECK(phys.state().y > 0.0);
    CHECK(std::fabs(phys.state().x) < 1e-6);
}

TEST_CASE("Physics heading convention: 90 deg = +X") {
    Field f;
    Physics phys(f);
    phys.set_state({ 0.0, 0.0, 90.0 });
    phys.step(Action::FORWARD);
    CHECK(phys.state().x > 0.0);
    CHECK(std::fabs(phys.state().y) < 1e-6);
}

TEST_CASE("Physics heading convention: 270 deg = -X") {
    Field f;
    Physics phys(f);
    phys.set_state({ 0.0, 0.0, 270.0 });
    phys.step(Action::FORWARD);
    CHECK(phys.state().x < 0.0);
    CHECK(std::fabs(phys.state().y) < 1e-6);
}

// ============================================================================
// Field + Physics with obstacles (Phase 6)
// ============================================================================

TEST_CASE("Field is_passable: inside obstacle is blocked") {
    Field f;
    f.obstacles.push_back({ -10.0, -5.0, 10.0, 5.0 });
    CHECK_FALSE(f.is_passable(0.0, 0.0));
    CHECK(f.is_passable(20.0, 20.0));
}

TEST_CASE("Field segment_hits_obstacle: detects crossing segment") {
    Field f;
    f.obstacles.push_back({ -2.0, 4.0, 2.0, 8.0 });
    CHECK(f.segment_hits_obstacle(0.0, 0.0, 0.0, 10.0));
    CHECK_FALSE(f.segment_hits_obstacle(10.0, 0.0, 10.0, 10.0));
}

TEST_CASE("Physics obstacle collision: moving into obstacle stops robot") {
    Field f;
    f.obstacles.push_back({ -2.0, 4.0, 2.0, 8.0 });
    PhysicsConfig cfg;
    Physics phys(f, cfg);
    phys.set_state({ 0.0, 3.0, 0.0 });  // heading +Y toward box

    auto r = phys.step(Action::FORWARD); // step would end at y=4.8 into obstacle
    CHECK(r.colliding);
    CHECK(r.delta.forward_in == doctest::Approx(0.0).epsilon(1e-6));
    CHECK(phys.state().y == doctest::Approx(3.0).epsilon(1e-6));
}

TEST_CASE("Physics obstacle collision: moving away from obstacle succeeds") {
    Field f;
    f.obstacles.push_back({ -2.0, 4.0, 2.0, 8.0 });
    PhysicsConfig cfg;
    Physics phys(f, cfg);
    phys.set_state({ 0.0, 3.0, 180.0 });  // heading -Y away from obstacle

    auto r = phys.step(Action::FORWARD);
    CHECK_FALSE(r.colliding);
    CHECK(r.delta.forward_in > 0.0);
    CHECK(phys.state().y < 3.0);
}
