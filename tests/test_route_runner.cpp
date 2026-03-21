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

TEST_CASE("RouteRunner odom delta gate can throttle estimate updates") {
    pursuit::RouteDefinition loose_gate;
    loose_gate.name = "unit_odom_loose";
    loose_gate.description = "loose odom gate";
    loose_gate.waypoints = {{0.0, 0.0}, {20.0, 0.0}, {20.0, 20.0}};
    loose_gate.max_ticks = 220;
    loose_gate.failure_seed = 98;
    loose_gate.failure_config = {};
    loose_gate.max_inches_odom_delta_per_tick = 48.0;

    pursuit::RouteDefinition strict_gate = loose_gate;
    strict_gate.name = "unit_odom_strict";
    strict_gate.max_inches_odom_delta_per_tick = 0.5;

    pursuit::RouteRunner runner;
    const auto loose = runner.run(loose_gate, "/tmp");
    const auto strict = runner.run(strict_gate, "/tmp");

    CHECK(loose.waypoints_reached >= strict.waypoints_reached);
}

