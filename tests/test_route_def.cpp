#include "doctest/doctest.h"
#include "pursuit/route.hpp"

#include <fstream>
#include <string>

namespace {
std::string temp_path(const std::string& name) {
    return "/tmp/" + name;
}
}

TEST_CASE("load_route parses valid JSON file") {
    const std::string path = temp_path("route_valid_test.json");
    std::ofstream out(path, std::ios::trunc);
    out << R"({
      "name": "test_route",
      "waypoints": [{"x": 0, "y": 0}, {"x": 10, "y": 20}],
      "max_ticks": 120
    })";
    out.close();

    const auto route = pursuit::load_route(path);
    CHECK(route.name == "test_route");
    CHECK(route.waypoints.size() == 2);
    CHECK(route.max_ticks == 120);
}

TEST_CASE("load_route throws on missing file") {
    CHECK_THROWS_AS(pursuit::load_route("/tmp/no_such_route_file.json"), std::runtime_error);
}

TEST_CASE("validate_route rejects empty waypoints") {
    pursuit::RouteDefinition route;
    route.name = "empty";
    CHECK_THROWS_AS(pursuit::validate_route(route), std::runtime_error);
}

TEST_CASE("load_route parses max_inches_odom_delta_per_tick") {
    const std::string path = temp_path("route_odom_gate_test.json");
    std::ofstream out(path, std::ios::trunc);
    out << R"({
      "name": "odom_gate_route",
      "waypoints": [{"x": 0, "y": 0}, {"x": 10, "y": 0}, {"x": 10, "y": 10}],
      "max_inches_odom_delta_per_tick": 18.5,
      "max_ticks": 120
    })";
    out.close();

    const auto route = pursuit::load_route(path);
    CHECK(route.max_inches_odom_delta_per_tick == doctest::Approx(18.5));
}

TEST_CASE("validate_route rejects segment crossing obstacle") {
    pursuit::RouteDefinition route;
    route.name = "crossing";
    route.waypoints = {{-20.0, 0.0}, {20.0, 0.0}};
    route.obstacles = {{-5.0, -5.0, 5.0, 5.0}};
    route.initial_position = {-30.0, 0.0};
    CHECK_THROWS_AS(pursuit::validate_route(route), std::runtime_error);
}

