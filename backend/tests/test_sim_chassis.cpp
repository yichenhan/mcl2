#include "doctest/doctest.h"

#include "sim/sim_chassis.hpp"
#include "sim/physics.hpp"

#include <cmath>

static sim::SimChassis make_chassis(double x = 0, double y = 0, double heading = 0,
                                     double field_half = 72.0) {
    sim::Field field;
    field.field_half = field_half;
    sim::PhysicsConfig phys;
    sim::RobotState initial{x, y, heading};
    return sim::SimChassis(field, phys, initial);
}

TEST_CASE("SimChassis getPose returns initial state") {
    auto chassis = make_chassis(10, 20, 45);
    const auto p = chassis.getPose();
    CHECK(p.x == doctest::Approx(10.0f));
    CHECK(p.y == doctest::Approx(20.0f));
    CHECK(p.theta == doctest::Approx(45.0f));
}

TEST_CASE("SimChassis setPose overwrites odom, ground_truth unchanged") {
    auto chassis = make_chassis(0, 0, 0);
    chassis.step(24.0, 0.0);  // advance ground truth

    chassis.setPose(5.0f, 7.0f, 90.0f);
    const auto p = chassis.getPose();
    CHECK(p.x == doctest::Approx(5.0f));
    CHECK(p.y == doctest::Approx(7.0f));
    CHECK(p.theta == doctest::Approx(90.0f));

    // Ground truth should not be affected by setPose
    const auto gt = chassis.ground_truth();
    CHECK(gt.y > 0.0);  // stepped forward before setPose
}

TEST_CASE("SimChassis step advances ground truth but not odom") {
    auto chassis = make_chassis(0, 0, 0);
    const auto before_gt = chassis.ground_truth();

    chassis.step(24.0, 0.0);
    const auto after_gt = chassis.ground_truth();
    CHECK(after_gt.y > before_gt.y);  // moved forward

    // odom unchanged until applyOdomNoise
    const auto p = chassis.getPose();
    CHECK(p.x == doctest::Approx(0.0f));
    CHECK(p.y == doctest::Approx(0.0f));
}

TEST_CASE("SimChassis applyOdomNoise integrates delta into odom") {
    auto chassis = make_chassis(0, 0, 0);
    const auto step = chassis.step(24.0, 0.0);

    chassis.applyOdomNoise(step.delta, 0.0);
    const auto p = chassis.getPose();
    // Should have moved (delta.forward_in is non-zero after stepping forward)
    CHECK(std::fabs(p.y) > 0.01f);
}

TEST_CASE("SimChassis ground_truth is independent of setPose") {
    auto chassis = make_chassis(0, 0, 0);
    chassis.step(24.0, 0.0);
    const auto gt_before = chassis.ground_truth();

    chassis.setPose(99.0f, 99.0f, 0.0f);
    const auto gt_after = chassis.ground_truth();

    CHECK(gt_after.x == doctest::Approx(gt_before.x));
    CHECK(gt_after.y == doctest::Approx(gt_before.y));
}

TEST_CASE("SimChassis odom drifts from truth over many steps") {
    auto chassis = make_chassis(0, 0, 0);
    // Apply slightly wrong deltas to simulate odom noise
    for (int i = 0; i < 100; ++i) {
        auto step = chassis.step(24.0, 5.0);
        // Introduce artificial drift: scale forward component
        sim::MotionDelta noisy = step.delta;
        noisy.forward_in *= 1.02;  // 2% over-reading
        const double hdg = chassis.ground_truth().heading_deg;
        chassis.applyOdomNoise(noisy, hdg);
    }
    const auto gt = chassis.ground_truth();
    const auto p = chassis.getPose();
    // odom and ground truth should have diverged
    const double dx = p.x - gt.x;
    const double dy = p.y - gt.y;
    const double drift = std::sqrt(dx * dx + dy * dy);
    CHECK(drift > 0.5);  // should have accumulated drift
}

TEST_CASE("SimChassis heading wraps at 360 boundary") {
    auto chassis = make_chassis(0, 0, 350);
    sim::MotionDelta delta{0, 0, 0};
    chassis.applyOdomNoise(delta, 370.0);  // 370 should wrap to 10
    const auto p = chassis.getPose();
    CHECK(p.theta >= 0.0f);
    CHECK(p.theta < 360.0f);
    CHECK(p.theta == doctest::Approx(10.0f).epsilon(0.01));
}

TEST_CASE("SimChassis field clamping prevents odom outside bounds") {
    const double half = 72.0;
    auto chassis = make_chassis(0, 0, 0, half);
    // Integrate a huge delta that would push far outside bounds
    sim::MotionDelta delta{999.0, 0.0, 0.0};
    chassis.applyOdomNoise(delta, 0.0);
    const auto p = chassis.getPose();
    CHECK(p.y <= static_cast<float>(half));
}

TEST_CASE("SimChassis step with Action enum works") {
    auto chassis = make_chassis(0, 0, 0);
    const auto result = chassis.step(sim::Action::FORWARD);
    CHECK(result.delta.forward_in > 0.0);
}

TEST_CASE("SimChassis multiple setPose calls are idempotent") {
    auto chassis = make_chassis(0, 0, 0);
    chassis.setPose(10.0f, 20.0f, 90.0f);
    chassis.setPose(5.0f, 15.0f, 45.0f);
    const auto p = chassis.getPose();
    CHECK(p.x == doctest::Approx(5.0f));
    CHECK(p.y == doctest::Approx(15.0f));
    CHECK(p.theta == doctest::Approx(45.0f));
}

TEST_CASE("SimChassis zero delta applyOdomNoise preserves position") {
    auto chassis = make_chassis(10, 20, 45);
    sim::MotionDelta zero{0, 0, 0};
    chassis.applyOdomNoise(zero, 45.0);
    const auto p = chassis.getPose();
    CHECK(p.x == doctest::Approx(10.0f));
    CHECK(p.y == doctest::Approx(20.0f));
    CHECK(p.theta == doctest::Approx(45.0f));
}

TEST_CASE("SimChassis negative velocity steps backward") {
    auto chassis = make_chassis(0, 0, 0);
    const auto gt_before = chassis.ground_truth();
    chassis.step(-24.0, 0.0);
    const auto gt_after = chassis.ground_truth();
    CHECK(gt_after.y < gt_before.y);
}

TEST_CASE("SimChassis teleport moves ground truth only") {
    auto chassis = make_chassis(0, 0, 0);
    const auto odom_before = chassis.getPose();

    sim::RobotState target{50, 50, 90};
    chassis.teleport(target);

    // Ground truth moved
    const auto gt = chassis.ground_truth();
    CHECK(gt.x == doctest::Approx(50.0));
    CHECK(gt.y == doctest::Approx(50.0));

    // Odom unchanged
    const auto p = chassis.getPose();
    CHECK(p.x == doctest::Approx(odom_before.x));
    CHECK(p.y == doctest::Approx(odom_before.y));
}
