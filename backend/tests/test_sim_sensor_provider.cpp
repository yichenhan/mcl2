#include "doctest/doctest.h"

#include "noise/failure_injector.hpp"
#include "sim/sim_chassis.hpp"
#include "sim/sim_sensor_provider.hpp"

#include <array>
#include <cmath>

// ============================================================================
// Helpers
// ============================================================================

static sim::SimChassis make_chassis(double x = 0, double y = 0, double heading = 0) {
    sim::Field field;
    field.field_half = 72.0;
    sim::PhysicsConfig phys;
    sim::RobotState init{x, y, heading};
    return sim::SimChassis(field, phys, init);
}

static sim::SimSensorProvider make_provider(const sim::SimChassis& chassis,
                                             uint64_t seed = 42) {
    sim::SensorNoiseConfig noise;
    noise.dropout_probability = 0.0;
    noise.long_dropout_probability = 0.0;
    noise.gaussian_stddev_mm = 0.0;
    noise.range_noise_slope = 0.0;
    noise.spurious_reflection_probability = 0.0;
    mcl::MCLConfig mcl_cfg;
    static std::vector<sim::Obstacle> empty_obstacles;
    return sim::SimSensorProvider(chassis, seed, noise, mcl_cfg, empty_obstacles);
}

// ============================================================================
// FailureInjector split API tests
// ============================================================================

TEST_CASE("FailureInjector applyOdom modifies odom spike but not readings") {
    noise::FailureInjector inj;
    noise::FailureEvent e;
    e.start_tick = 0;
    e.duration_ticks = 1;
    e.type = noise::FailureType::OdomSpike;
    e.param = 3.0;
    inj.schedule(e);

    std::array<double, 4> readings{1.0, 2.0, 3.0, 4.0};
    sim::MotionDelta odom{2.0, 1.0, 10.0};
    double heading = 90.0;

    inj.applyOdom(0, odom, heading);
    CHECK(odom.forward_in == doctest::Approx(6.0));
    CHECK(odom.lateral_in == doctest::Approx(3.0));
    CHECK(odom.rotation_deg == doctest::Approx(30.0));
    // Readings must be untouched
    CHECK(readings[0] == doctest::Approx(1.0));
    CHECK(readings[1] == doctest::Approx(2.0));
}

TEST_CASE("FailureInjector applySensors modifies readings but not odom") {
    noise::FailureInjector inj;
    noise::FailureEvent e;
    e.start_tick = 0;
    e.duration_ticks = 1;
    e.type = noise::FailureType::SensorDead;
    e.sensor_idx = 1;
    inj.schedule(e);

    std::array<double, 4> readings{1.0, 2.0, 3.0, 4.0};
    sim::MotionDelta odom{2.0, 1.0, 10.0};
    double heading = 90.0;

    inj.applySensors(0, readings);
    CHECK(readings[1] == doctest::Approx(-1.0));
    CHECK(readings[0] == doctest::Approx(1.0));
    // odom and heading untouched
    CHECK(odom.forward_in == doctest::Approx(2.0));
    CHECK(heading == doctest::Approx(90.0));
}

TEST_CASE("FailureInjector applyOdom handles heading bias") {
    noise::FailureInjector inj;
    noise::FailureEvent e;
    e.start_tick = 0;
    e.duration_ticks = 2;
    e.type = noise::FailureType::HeadingBias;
    e.param = 5.0;
    inj.schedule(e);

    sim::MotionDelta odom{0, 0, 0};
    double heading = 90.0;
    inj.applyOdom(0, odom, heading);
    CHECK(heading == doctest::Approx(95.0));
}

TEST_CASE("FailureInjector applySensors handles sensor stuck") {
    noise::FailureInjector inj;
    noise::FailureEvent e;
    e.start_tick = 0;
    e.duration_ticks = 3;
    e.type = noise::FailureType::SensorStuck;
    e.sensor_idx = 0;
    inj.schedule(e);

    std::array<double, 4> r1{10.0, 2.0, 3.0, 4.0};
    inj.applySensors(0, r1);
    CHECK(r1[0] == doctest::Approx(10.0));  // first tick -- stuck at initial value

    std::array<double, 4> r2{99.0, 2.0, 3.0, 4.0};  // value changed
    inj.applySensors(1, r2);
    CHECK(r2[0] == doctest::Approx(10.0));  // frozen at first seen value
}

TEST_CASE("FailureInjector split apply is equivalent to combined apply") {
    // Build two injectors with identical events
    noise::FailureEvent spike{0, 1, noise::FailureType::OdomSpike, -1, 2.0, 0.0, 0.0};
    noise::FailureEvent dead{0, 1, noise::FailureType::SensorDead, 2, 0.0, 0.0, 0.0};
    noise::FailureEvent bias{0, 1, noise::FailureType::HeadingBias, -1, 5.0, 0.0, 0.0};

    noise::FailureInjector combined;
    combined.schedule(spike); combined.schedule(dead); combined.schedule(bias);

    noise::FailureInjector split;
    split.schedule(spike); split.schedule(dead); split.schedule(bias);

    std::array<double, 4> r_c{1.0, 2.0, 3.0, 4.0};
    std::array<double, 4> r_s{1.0, 2.0, 3.0, 4.0};
    sim::MotionDelta o_c{1.0, 0.5, 5.0};
    sim::MotionDelta o_s{1.0, 0.5, 5.0};
    double h_c = 90.0, h_s = 90.0;

    combined.apply(0, r_c, o_c, h_c);
    split.applyOdom(0, o_s, h_s);
    split.applySensors(0, r_s);

    CHECK(r_c[2] == doctest::Approx(r_s[2]));
    CHECK(o_c.forward_in == doctest::Approx(o_s.forward_in));
    CHECK(h_c == doctest::Approx(h_s));
}

TEST_CASE("FailureInjector applyOdom with no active events is no-op") {
    noise::FailureInjector inj;
    sim::MotionDelta odom{1.0, 0.5, 10.0};
    double heading = 45.0;
    inj.applyOdom(0, odom, heading);
    CHECK(odom.forward_in == doctest::Approx(1.0));
    CHECK(heading == doctest::Approx(45.0));
}

TEST_CASE("FailureInjector applySensors with no active events is no-op") {
    noise::FailureInjector inj;
    std::array<double, 4> readings{10.0, 20.0, 30.0, 40.0};
    inj.applySensors(0, readings);
    CHECK(readings[0] == doctest::Approx(10.0));
    CHECK(readings[3] == doctest::Approx(40.0));
}

// ============================================================================
// SimSensorProvider tests
// ============================================================================

TEST_CASE("SimSensorProvider getReadings returns valid distances for centered robot") {
    auto chassis = make_chassis(0, 0, 0);
    auto provider = make_provider(chassis);
    const auto readings = provider.getReadings();
    // At (0,0,0) on a 72-inch half field, all 4 sensors should see walls
    CHECK(readings.left.has_value());
    CHECK(readings.right.has_value());
    CHECK(readings.front.has_value());
    CHECK(readings.back.has_value());
    CHECK(*readings.left > 0);
    CHECK(*readings.right > 0);
    CHECK(*readings.front > 0);
    CHECK(*readings.back > 0);
}

TEST_CASE("SimSensorProvider getReadings uses ground truth not odom") {
    auto chassis = make_chassis(0, 0, 0);
    // Move odom far from truth via setPose
    chassis.setPose(60.0f, 60.0f, 0.0f);
    // Ground truth is still at (0,0,0)

    auto provider = make_provider(chassis);
    const auto readings_from_truth = provider.getReadings();

    // Now build another provider with chassis actually at (60,60)
    auto chassis2 = make_chassis(60, 60, 0);
    auto provider2 = make_provider(chassis2);
    const auto readings_from_60 = provider2.getReadings();

    // Since truth is at (0,0), readings should match a robot at (0,0)
    // and NOT match a robot at (60,60)
    if (readings_from_truth.left && readings_from_60.left) {
        CHECK(*readings_from_truth.left != *readings_from_60.left);
    }
}

TEST_CASE("SimSensorProvider advanceTick increments counter") {
    auto chassis = make_chassis(0, 0, 0);
    auto provider = make_provider(chassis);
    CHECK(provider.current_tick() == 0);
    provider.advanceTick();
    provider.advanceTick();
    provider.advanceTick();
    CHECK(provider.current_tick() == 3);
}

TEST_CASE("SimSensorProvider sensor failure applies at correct tick") {
    auto chassis = make_chassis(0, 0, 0);
    auto provider = make_provider(chassis);

    noise::FailureEvent e;
    e.start_tick = 5;
    e.duration_ticks = 1;
    e.type = noise::FailureType::SensorDead;
    e.sensor_idx = 0;
    provider.schedule_failure(e);

    // At tick 0, sensor 0 should be valid
    const auto r0 = provider.getReadings();
    CHECK(r0.left.has_value());

    // Advance to tick 5
    for (int i = 0; i < 5; ++i) provider.advanceTick();

    // At tick 5, sensor 0 should be dead
    const auto r5 = provider.getReadings();
    CHECK_FALSE(r5.left.has_value());
}

TEST_CASE("SimSensorProvider sensor failure inactive before start tick") {
    auto chassis = make_chassis(0, 0, 0);
    auto provider = make_provider(chassis);

    noise::FailureEvent e;
    e.start_tick = 5;
    e.duration_ticks = 2;
    e.type = noise::FailureType::SensorDead;
    e.sensor_idx = 0;
    provider.schedule_failure(e);

    // Advance to tick 3 (before failure)
    for (int i = 0; i < 3; ++i) provider.advanceTick();
    const auto r3 = provider.getReadings();
    CHECK(r3.left.has_value());
}

TEST_CASE("SimSensorProvider multiple sensor failures compose") {
    auto chassis = make_chassis(0, 0, 0);
    auto provider = make_provider(chassis);

    noise::FailureEvent dead;
    dead.start_tick = 0; dead.duration_ticks = 5;
    dead.type = noise::FailureType::SensorDead; dead.sensor_idx = 0;

    noise::FailureEvent spurious;
    spurious.start_tick = 0; spurious.duration_ticks = 5;
    spurious.type = noise::FailureType::SpuriousReflection; spurious.sensor_idx = 1;
    spurious.param = 5.0;  // 5 inches

    provider.schedule_failure(dead);
    provider.schedule_failure(spurious);

    const auto r = provider.getReadings();
    CHECK_FALSE(r.left.has_value());  // dead
    // sensor 1 override = 5 inches = 127mm
    CHECK(r.right.has_value());
    CHECK(*r.right == static_cast<int32_t>(std::lround(5.0 * 25.4)));
}

TEST_CASE("SimSensorProvider failure_injector returns mutable reference") {
    auto chassis = make_chassis(0, 0, 0);
    auto provider = make_provider(chassis);

    noise::FailureEvent e;
    e.start_tick = 0; e.duration_ticks = 1;
    e.type = noise::FailureType::SensorDead; e.sensor_idx = 2;

    provider.failure_injector().schedule(e);
    const auto r = provider.getReadings();
    CHECK_FALSE(r.front.has_value());
}

TEST_CASE("SimSensorProvider getReadings consistent across multiple calls same tick") {
    // Without noise, same tick should produce same readings
    auto chassis = make_chassis(0, 0, 0);

    sim::SensorNoiseConfig noise;
    noise.dropout_probability = 0.0;
    noise.long_dropout_probability = 0.0;
    noise.gaussian_stddev_mm = 0.0;
    noise.range_noise_slope = 0.0;
    noise.spurious_reflection_probability = 0.0;
    mcl::MCLConfig mcl_cfg;
    static std::vector<sim::Obstacle> empty;
    sim::SimSensorProvider provider(chassis, 42, noise, mcl_cfg, empty);

    const auto r1 = provider.getReadings();
    const auto r2 = provider.getReadings();  // same tick

    CHECK(r1.left == r2.left);
    CHECK(r1.right == r2.right);
    CHECK(r1.front == r2.front);
    CHECK(r1.back == r2.back);
}
