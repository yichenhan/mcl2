#include "doctest/doctest.h"
#include "pursuit/waypoint_follower.hpp"

#include <vector>

using namespace pursuit;

TEST_CASE("WaypointFollower points forward for waypoint ahead") {
    WaypointFollower follower;
    sim::RobotState s{0.0, 0.0, 0.0};
    std::vector<distance_loc::Vec2> waypoints{{0.0, 24.0}};
    const auto cmd = follower.compute(s, waypoints);
    CHECK(cmd.linear_vel > 0.0);
    CHECK(cmd.angular_vel_deg == doctest::Approx(0.0).epsilon(1e-3));
}

TEST_CASE("WaypointFollower turns CW for waypoint to the right") {
    WaypointFollower follower;
    sim::RobotState s{0.0, 0.0, 0.0};
    std::vector<distance_loc::Vec2> waypoints{{24.0, 0.0}};
    const auto cmd = follower.compute(s, waypoints);
    CHECK(cmd.angular_vel_deg > 0.0);
}

TEST_CASE("WaypointFollower advances index when within tolerance") {
    FollowerConfig cfg;
    cfg.waypoint_tolerance = 3.0;
    WaypointFollower follower(cfg);
    std::vector<distance_loc::Vec2> waypoints{{0.0, 0.0}, {0.0, 30.0}};
    sim::RobotState s{0.0, 0.0, 0.0};
    (void)follower.compute(s, waypoints);
    CHECK(follower.current_waypoint_index() == 1);
}

TEST_CASE("WaypointFollower uses turnToHeading for large heading error") {
    FollowerConfig cfg;
    cfg.turn_in_place_threshold_deg = 45.0;
    WaypointFollower follower(cfg);
    sim::RobotState s{0.0, 0.0, 0.0};
    std::vector<distance_loc::Vec2> waypoints{{-24.0, 0.0}};
    const auto cmd = follower.compute(s, waypoints);
    CHECK(cmd.linear_vel == doctest::Approx(0.0));
    CHECK(cmd.angular_vel_deg < 0.0);
}

TEST_CASE("WaypointFollower does not drive forward with moderate heading error") {
    FollowerConfig cfg;
    cfg.turn_in_place_threshold_deg = 45.0;
    WaypointFollower follower(cfg);
    sim::RobotState s{0.0, 0.0, 0.0};
    // About 40 deg heading error: below turn threshold but still should not
    // translate forward until heading is better aligned.
    std::vector<distance_loc::Vec2> waypoints{{25.0, 30.0}};
    const auto cmd = follower.compute(s, waypoints);
    CHECK(cmd.linear_vel == doctest::Approx(0.0));
    CHECK(cmd.angular_vel_deg > 0.0);
}

TEST_CASE("WaypointFollower advances when waypoint is passed slightly beyond target") {
    FollowerConfig cfg;
    cfg.waypoint_tolerance = 3.0;
    WaypointFollower follower(cfg);
    std::vector<distance_loc::Vec2> waypoints{{0.0, 0.0}, {0.0, 30.0}};
    // First call consumes waypoint 0 at origin.
    (void)follower.compute(sim::RobotState{0.0, 0.0, 0.0}, waypoints);
    CHECK(follower.current_waypoint_index() == 1);

    // Robot has passed y=30 by a small amount but is just outside tolerance.
    (void)follower.compute(sim::RobotState{0.0, 34.0, 0.0}, waypoints);
    CHECK(follower.is_complete());
}

TEST_CASE("WaypointFollower marks completion when all waypoints reached") {
    FollowerConfig cfg;
    cfg.waypoint_tolerance = 1.0;
    WaypointFollower follower(cfg);
    std::vector<distance_loc::Vec2> waypoints{{0.0, 0.0}};
    sim::RobotState s{0.0, 0.0, 0.0};
    (void)follower.compute(s, waypoints);
    CHECK(follower.is_complete());
}

