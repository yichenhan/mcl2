#include "doctest/doctest.h"
#include "noise/failure_injector.hpp"

TEST_CASE("FailureInjector spurious reflection overrides sensor reading") {
    noise::FailureInjector injector;
    noise::FailureEvent e;
    e.type = noise::FailureType::SpuriousReflection;
    e.start_tick = 5;
    e.duration_ticks = 3;
    e.sensor_idx = 2;
    e.param = 4.5;
    injector.schedule(e);

    std::array<double, 4> readings{10.0, 11.0, 12.0, 13.0};
    sim::MotionDelta odom{};
    double heading = 0.0;
    injector.apply(5, readings, odom, heading);
    CHECK(readings[2] == doctest::Approx(4.5));
    CHECK(readings[0] == doctest::Approx(10.0));
}

