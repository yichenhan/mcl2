#include "doctest/doctest.h"
#include "pursuit/route_runner.hpp"

TEST_CASE("RouteRunner executes a simple route and writes replay name") {
    pursuit::RouteDefinition route;
    route.name = "unit_simple";
    route.description = "unit route";
    route.waypoints = {{0.0, 0.0}, {0.0, 30.0}};
    route.max_ticks = 220;
    route.failure_seed = 99;
    route.failure_config = {};

    pursuit::RouteRunner runner;
    const auto result = runner.run(route, "/tmp");
    CHECK(result.route_name == "unit_simple");
    CHECK(result.total_ticks > 0);
    CHECK(!result.replay_file.empty());
}

TEST_CASE("RouteRunner velocity gate can throttle estimate updates") {
    pursuit::RouteDefinition fast_gate;
    fast_gate.name = "unit_velocity_fast";
    fast_gate.description = "high speed gate";
    fast_gate.waypoints = {{0.0, 0.0}, {20.0, 0.0}, {20.0, 20.0}};
    fast_gate.max_ticks = 220;
    fast_gate.failure_seed = 98;
    fast_gate.failure_config = {};
    fast_gate.max_estimate_speed_ft_per_s = 10.0;

    pursuit::RouteDefinition strict_gate = fast_gate;
    strict_gate.name = "unit_velocity_strict";
    strict_gate.max_estimate_speed_ft_per_s = 0.05;

    pursuit::RouteRunner runner;
    const auto fast = runner.run(fast_gate, "/tmp");
    const auto strict = runner.run(strict_gate, "/tmp");

    CHECK(fast.waypoints_reached >= strict.waypoints_reached);
}

