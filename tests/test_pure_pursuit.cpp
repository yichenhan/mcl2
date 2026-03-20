#include "doctest/doctest.h"
#include "pursuit/pure_pursuit.hpp"

using namespace pursuit;

TEST_CASE("PurePursuit points forward for waypoint ahead") {
    PurePursuit pp;
    sim::RobotState s{0.0, 0.0, 0.0};
    std::vector<distance_loc::Vec2> waypoints{{0.0, 24.0}};
    const auto cmd = pp.compute(s, waypoints);
    CHECK(cmd.linear_vel > 0.0);
    CHECK(cmd.angular_vel_deg == doctest::Approx(0.0).epsilon(1e-3));
}

TEST_CASE("PurePursuit turns CW for waypoint to the right") {
    PurePursuit pp;
    sim::RobotState s{0.0, 0.0, 0.0};
    std::vector<distance_loc::Vec2> waypoints{{24.0, 0.0}};
    const auto cmd = pp.compute(s, waypoints);
    CHECK(cmd.angular_vel_deg > 0.0);
}

TEST_CASE("PurePursuit advances index when within tolerance") {
    PurePursuitConfig cfg;
    cfg.waypoint_tolerance = 3.0;
    PurePursuit pp(cfg);
    std::vector<distance_loc::Vec2> waypoints{{0.0, 0.0}, {0.0, 30.0}};
    sim::RobotState s{0.0, 0.0, 0.0};
    (void)pp.compute(s, waypoints);
    CHECK(pp.current_waypoint_index() == 1);
}

TEST_CASE("PurePursuit progresses even if robot passes a waypoint outside tolerance") {
    PurePursuitConfig cfg;
    cfg.lookahead_distance = 8.0;
    cfg.waypoint_tolerance = 0.5;
    PurePursuit pp(cfg);

    // L-shaped path: robot has passed the corner waypoint, but is not within tolerance of it.
    std::vector<distance_loc::Vec2> waypoints{{0.0, 0.0}, {0.0, 10.0}, {10.0, 10.0}};
    sim::RobotState s{2.0, 12.0, 90.0};
    const auto cmd = pp.compute(s, waypoints);

    CHECK(pp.current_waypoint_index() >= 2);
    CHECK(cmd.linear_vel > 0.0);
}

TEST_CASE("PurePursuit lookahead target stays forward along segment") {
    PurePursuitConfig cfg;
    cfg.lookahead_distance = 10.0;
    cfg.waypoint_tolerance = 1.0;
    PurePursuit pp(cfg);

    std::vector<distance_loc::Vec2> waypoints{{0.0, 0.0}, {0.0, 40.0}};
    sim::RobotState s{0.0, 5.0, 0.0};
    const auto cmd = pp.compute(s, waypoints);

    CHECK(pp.current_waypoint_index() == 1);
    CHECK(cmd.linear_vel > 0.0);
    CHECK(std::fabs(cmd.angular_vel_deg) < 20.0);
}

