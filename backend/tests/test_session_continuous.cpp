#include "doctest/doctest.h"
#include "sim/sim_harness.hpp"

TEST_CASE("SimHarness tick(linear, angular) advances robot") {
    sim::SimHarness harness(sim::SimHarness::Config{});
    const auto t0 = harness.tick(24.0, 0.0);
    CHECK(t0.tick == 0);
    CHECK(t0.ground_truth.y > 0.0);
    CHECK(!t0.timestamp_iso.empty());
}

TEST_CASE("SimHarness continuous tick populates chassis_pose") {
    sim::SimHarness harness(sim::SimHarness::Config{});
    const auto t0 = harness.tick(24.0, 0.0);
    CHECK(std::isfinite(t0.chassis_pose.x));
    CHECK(std::isfinite(t0.chassis_pose.y));
    CHECK(std::isfinite(t0.chassis_pose.theta));
}

