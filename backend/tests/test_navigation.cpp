#include "doctest/doctest.h"

#include "pursuit/waypoint_follower.hpp"
#include "sim/sim_harness.hpp"

#include <cmath>

using pursuit::Waypoint;
using pursuit::WaypointFollower;
using pursuit::FollowerConfig;
using pursuit::FollowerResult;

// ============================================================================
// WaypointFollower unit tests
// ============================================================================

TEST_CASE("WaypointFollower computes forward velocity toward target directly ahead") {
    WaypointFollower follower;
    sim::RobotState pose{0, 0, 0};  // heading 0 = facing +y
    std::vector<Waypoint> wps{{0, 30}};
    const auto cmd = follower.compute(pose, wps, 0);
    CHECK(cmd.linear_velocity > 0.0);
    CHECK(!cmd.all_done);
    CHECK(!cmd.waypoint_reached);
}

TEST_CASE("WaypointFollower reports reached when within tolerance") {
    FollowerConfig cfg;
    cfg.waypoint_tolerance = 3.0;
    WaypointFollower follower(cfg);
    sim::RobotState pose{0, 19.5, 0};
    std::vector<Waypoint> wps{{0, 20}};
    const auto cmd = follower.compute(pose, wps, 0);
    CHECK(cmd.waypoint_reached);
}

TEST_CASE("WaypointFollower commands angular velocity toward off-axis target") {
    WaypointFollower follower;
    sim::RobotState pose{0, 0, 0};   // facing +y
    std::vector<Waypoint> wps{{20, 0}};  // target is to the right (+x)
    const auto cmd = follower.compute(pose, wps, 0);
    // Must turn to reach the target
    CHECK(std::fabs(cmd.angular_vel_deg) > 0.0);
}

TEST_CASE("WaypointFollower all_done when list is empty") {
    WaypointFollower follower;
    sim::RobotState pose{0, 0, 0};
    std::vector<Waypoint> empty;
    const auto cmd = follower.compute(pose, empty, 0);
    CHECK(cmd.all_done);
}

TEST_CASE("WaypointFollower all_done after last waypoint reached") {
    FollowerConfig cfg;
    cfg.waypoint_tolerance = 5.0;
    WaypointFollower follower(cfg);
    sim::RobotState pose{0, 0, 0};  // at origin, within 5 inches of waypoint at (0,0)
    std::vector<Waypoint> wps{{0, 0}};
    const auto cmd = follower.compute(pose, wps, 0);
    CHECK(cmd.waypoint_reached);
    CHECK(cmd.all_done);
}

TEST_CASE("WaypointFollower angular velocity is capped") {
    FollowerConfig cfg;
    cfg.max_angular_vel_deg = 90.0;
    WaypointFollower follower(cfg);
    sim::RobotState pose{0, 0, 0};   // facing +y
    std::vector<Waypoint> wps{{-40, 0}};  // hard left
    const auto cmd = follower.compute(pose, wps, 0);
    CHECK(std::fabs(cmd.angular_vel_deg) <= cfg.max_angular_vel_deg + 1e-9);
}

// ============================================================================
// NavigationState + SimHarness integration tests
// ============================================================================

static sim::SimHarness::Config make_nav_cfg() {
    sim::SimHarness::Config cfg;
    cfg.controller_config.seed = 42;
    cfg.controller_config.mcl_config.num_particles = 100;
    cfg.sensor_noise.dropout_probability = 0.0;
    cfg.sensor_noise.long_dropout_probability = 0.0;
    cfg.sensor_noise.gaussian_stddev_mm = 0.0;
    cfg.sensor_noise.range_noise_slope = 0.0;
    cfg.sensor_noise.spurious_reflection_probability = 0.0;
    cfg.odom_noise = {};  // zero odom noise for clean nav tests
    return cfg;
}

TEST_CASE("WaypointFollower drives robot toward waypoint over multiple ticks") {
    auto cfg = make_nav_cfg();
    sim::SimHarness harness(cfg);

    pursuit::WaypointFollower follower;
    std::vector<Waypoint> wps{{0, 30}};  // 30 inches ahead
    int waypoint_idx = 0;

    for (int i = 0; i < 60; ++i) {
        const state::TickState& last = harness.history().empty()
            ? (void)harness.tick(0.0, 0.0), harness.history().back()
            : harness.history().back();
        const sim::RobotState robot{last.chassis_pose.x, last.chassis_pose.y, last.chassis_pose.theta};
        const auto cmd = follower.compute(robot, wps, waypoint_idx);
        if (cmd.waypoint_reached || cmd.all_done) break;
        harness.tick(cmd.linear_velocity, cmd.angular_vel_deg);
    }
    // Robot should have moved forward significantly
    const auto& last = harness.history().back();
    CHECK(last.ground_truth.y > 5.0);
}

TEST_CASE("WaypointFollower second waypoint is targeted after first reached") {
    pursuit::FollowerConfig fcfg;
    fcfg.waypoint_tolerance = 4.0;
    pursuit::WaypointFollower follower(fcfg);

    // Robot at (0, 15) -- within tolerance of wp0=(0,15), so it should move to wp1
    sim::RobotState pose{0, 15, 0};
    std::vector<Waypoint> wps{{0, 15}, {0, 40}};

    const auto cmd0 = follower.compute(pose, wps, 0);
    CHECK(cmd0.waypoint_reached);
    CHECK(!cmd0.all_done);

    // Now index 1 -- robot far from wp1
    const auto cmd1 = follower.compute(pose, wps, 1);
    CHECK(!cmd1.waypoint_reached);
    CHECK(cmd1.linear_velocity > 0.0);
}

TEST_CASE("WaypointFollower current_index is preserved in result") {
    WaypointFollower follower;
    sim::RobotState pose{0, 0, 0};
    std::vector<Waypoint> wps{{0, 30}};
    const auto cmd = follower.compute(pose, wps, 0);
    CHECK(cmd.current_index == 0);
}

TEST_CASE("WaypointFollower out-of-range index returns all_done") {
    WaypointFollower follower;
    sim::RobotState pose{0, 0, 0};
    std::vector<Waypoint> wps{{0, 30}};
    // Index beyond waypoints -- should return all_done
    const auto cmd = follower.compute(pose, wps, 5);
    CHECK(cmd.all_done);
}

TEST_CASE("WaypointFollower zero waypoints returns all_done") {
    WaypointFollower follower;
    sim::RobotState pose{0, 0, 0};
    const auto cmd = follower.compute(pose, {}, 0);
    CHECK(cmd.all_done);
    CHECK(cmd.linear_velocity == doctest::Approx(0.0));
}

TEST_CASE("WaypointFollower negative current_index returns all_done") {
    WaypointFollower follower;
    sim::RobotState pose{0, 0, 0};
    std::vector<Waypoint> wps{{0, 30}};
    const auto cmd = follower.compute(pose, wps, -1);
    CHECK(cmd.all_done);
}
