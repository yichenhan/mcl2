#include "doctest/doctest.h"

#include "noise/failure_injector.hpp"

TEST_CASE("FailureInjector sensor_dead invalidates reading in active window") {
    noise::FailureInjector inj;
    noise::FailureEvent e;
    e.start_tick = 2;
    e.duration_ticks = 3;
    e.type = noise::FailureType::SensorDead;
    e.sensor_idx = 1;
    inj.schedule(e);

    std::array<double, 4> readings{1.0, 2.0, 3.0, 4.0};
    sim::MotionDelta odom{1.0, 0.0, 0.0};
    double heading = 0.0;
    inj.apply(1, readings, odom, heading);
    CHECK(readings[1] == doctest::Approx(2.0));
    inj.apply(2, readings, odom, heading);
    CHECK(readings[1] == doctest::Approx(-1.0));
}

TEST_CASE("FailureInjector odom_spike and heading_bias apply") {
    noise::FailureInjector inj;
    noise::FailureEvent spike;
    spike.start_tick = 0;
    spike.duration_ticks = 1;
    spike.type = noise::FailureType::OdomSpike;
    spike.param = 3.0;
    inj.schedule(spike);

    noise::FailureEvent bias;
    bias.start_tick = 0;
    bias.duration_ticks = 1;
    bias.type = noise::FailureType::HeadingBias;
    bias.param = 5.0;
    inj.schedule(bias);

    std::array<double, 4> readings{1.0, 1.0, 1.0, 1.0};
    sim::MotionDelta odom{2.0, 1.0, 10.0};
    double heading = 90.0;
    inj.apply(0, readings, odom, heading);
    CHECK(odom.forward_in == doctest::Approx(6.0));
    CHECK(odom.lateral_in == doctest::Approx(3.0));
    CHECK(odom.rotation_deg == doctest::Approx(30.0));
    CHECK(heading == doctest::Approx(95.0));
}

TEST_CASE("FailureInjector sensor_stuck freezes value during active window") {
    noise::FailureInjector inj;
    noise::FailureEvent e;
    e.start_tick = 3;
    e.duration_ticks = 2;
    e.type = noise::FailureType::SensorStuck;
    e.sensor_idx = 0;
    inj.schedule(e);

    std::array<double, 4> readings{10.0, 2.0, 3.0, 4.0};
    sim::MotionDelta odom{0.0, 0.0, 0.0};
    double heading = 0.0;
    inj.apply(3, readings, odom, heading);
    CHECK(readings[0] == doctest::Approx(10.0));
    readings[0] = 99.0;
    inj.apply(4, readings, odom, heading);
    CHECK(readings[0] == doctest::Approx(10.0));
}
