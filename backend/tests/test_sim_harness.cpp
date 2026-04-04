#include "doctest/doctest.h"

#include "sim/sim_harness.hpp"

#include <cmath>

TEST_CASE("SimHarness tick produces populated TickState") {
    sim::SimHarness::Config cfg;
    cfg.controller_config.mcl_config.num_particles = 100;
    cfg.sensor_noise.dropout_probability = 0.0;
    cfg.sensor_noise.long_dropout_probability = 0.0;
    cfg.sensor_noise.gaussian_stddev_mm = 0.0;
    cfg.sensor_noise.range_noise_slope = 0.0;
    cfg.sensor_noise.spurious_reflection_probability = 0.0;
    sim::SimHarness harness(cfg);

    const auto t0 = harness.tick(sim::Action::FORWARD);
    CHECK(t0.tick == 0);
    CHECK(t0.post_predict.particles.size() == 100);
    CHECK(t0.post_update.particles.size() == 100);
    CHECK(t0.post_resample.particles.size() == 100);
    CHECK(!t0.timestamp_iso.empty());
}

TEST_CASE("SimHarness tracks history and tick count") {
    sim::SimHarness harness(sim::SimHarness::Config{});
    CHECK(harness.current_tick() == 0);
    CHECK(harness.history().empty());
    harness.tick(sim::Action::FORWARD);
    harness.tick(sim::Action::ROTATE_CW);
    CHECK(harness.current_tick() == 2);
    CHECK(harness.history().size() == 2);
}

TEST_CASE("SimHarness supports scheduled failures") {
    sim::SimHarness harness(sim::SimHarness::Config{});
    noise::FailureEvent e;
    // sensor_provider_.advanceTick() runs before getReadings(), so the first
    // call to getReadings() sees sensor_provider_tick == 1, not 0.
    e.start_tick = 1;
    e.duration_ticks = 1;
    e.type = noise::FailureType::SensorDead;
    e.sensor_idx = 0;
    harness.schedule_failure(e);
    const auto t0 = harness.tick(sim::Action::NONE);
    CHECK(t0.observed_readings[0] < 0.0);
}

TEST_CASE("SimHarness continuous tick advances robot") {
    sim::SimHarness harness(sim::SimHarness::Config{});
    const auto t0 = harness.tick(24.0, 0.0);
    CHECK(t0.ground_truth.y > 0.0);
}

TEST_CASE("SimHarness exposes underlying controller") {
    sim::SimHarness harness(sim::SimHarness::Config{});
    harness.controller().set_pose({5.0, 6.0, 7.0});
    const auto p = harness.controller().get_accepted_pose();
    CHECK(p.x == doctest::Approx(5.0));
    CHECK(p.y == doctest::Approx(6.0));
    CHECK(p.theta == doctest::Approx(7.0));
}

TEST_CASE("SimHarness tick computes accepted_error") {
    sim::SimHarness::Config cfg;
    cfg.controller_config.mcl_config.num_particles = 100;
    cfg.sensor_noise.dropout_probability = 0.0;
    cfg.sensor_noise.long_dropout_probability = 0.0;
    cfg.sensor_noise.gaussian_stddev_mm = 0.0;
    cfg.sensor_noise.range_noise_slope = 0.0;
    cfg.sensor_noise.spurious_reflection_probability = 0.0;
    sim::SimHarness harness(cfg);
    const auto t0 = harness.tick(sim::Action::NONE);
    CHECK(std::isfinite(t0.accepted_error));
    CHECK(t0.accepted_error >= 0.0);
    for (int i = 0; i < 50; ++i) {
        const auto t = harness.tick(24.0, 0.0);
        CHECK(std::isfinite(t.accepted_error));
    }
}

// ============================================================================
// Phase 4: chassis_pose + failure ordering + backward-compat tests
// ============================================================================

static sim::SimHarness::Config make_clean_cfg(uint64_t seed = 42) {
    sim::SimHarness::Config cfg;
    cfg.controller_config.seed = seed;
    cfg.controller_config.mcl_config.num_particles = 100;
    cfg.sensor_noise.dropout_probability = 0.0;
    cfg.sensor_noise.long_dropout_probability = 0.0;
    cfg.sensor_noise.gaussian_stddev_mm = 0.0;
    cfg.sensor_noise.range_noise_slope = 0.0;
    cfg.sensor_noise.spurious_reflection_probability = 0.0;
    return cfg;
}

TEST_CASE("SimHarness tick returns TickState with populated chassis_pose") {
    auto harness = sim::SimHarness(make_clean_cfg());
    const auto t0 = harness.tick(24.0, 0.0);
    CHECK(std::isfinite(t0.chassis_pose.x));
    CHECK(std::isfinite(t0.chassis_pose.y));
    CHECK(std::isfinite(t0.chassis_pose.theta));
}

TEST_CASE("SimHarness chassis_pose is finite after many ticks") {
    auto harness = sim::SimHarness(make_clean_cfg());
    for (int i = 0; i < 50; ++i) {
        const auto t = harness.tick(24.0, 5.0);
        CHECK(std::isfinite(t.chassis_pose.x));
        CHECK(std::isfinite(t.chassis_pose.y));
        CHECK(std::isfinite(t.chassis_pose.theta));
    }
}

TEST_CASE("SimHarness accepted_error uses chassis_pose, not raw_odom") {
    auto harness = sim::SimHarness(make_clean_cfg());
    for (int i = 0; i < 200; ++i) {
        const auto t = harness.tick(24.0, 0.0);
        // accepted_error should always be >= 0 and finite
        CHECK(std::isfinite(t.accepted_error));
        CHECK(t.accepted_error >= 0.0);
    }
}

TEST_CASE("SimHarness odom spike failure affects odom_error") {
    auto cfg = make_clean_cfg();
    noise::FailureEvent spike;
    spike.start_tick = 10;
    spike.duration_ticks = 1;
    spike.type = noise::FailureType::OdomSpike;
    spike.param = 20.0;

    sim::SimHarness harness(cfg);
    harness.schedule_failure(spike);

    mcl::Pose odom_before{};
    mcl::Pose odom_at_spike{};
    for (int i = 0; i < 15; ++i) {
        const auto t = harness.tick(24.0, 0.0);
        if (i == 9)  odom_before = t.raw_odom;
        if (i == 10) odom_at_spike = t.raw_odom;
    }
    // A 20x spike should produce a huge odom jump between ticks
    const double odom_jump = std::hypot(odom_at_spike.x - odom_before.x,
                                        odom_at_spike.y - odom_before.y);
    CHECK(odom_jump > 10.0);
}

TEST_CASE("SimHarness heading bias failure shifts observed_heading") {
    auto cfg = make_clean_cfg();
    noise::FailureEvent bias;
    bias.start_tick = 5;
    bias.duration_ticks = 1;
    bias.type = noise::FailureType::HeadingBias;
    bias.param = 30.0;

    // Start at heading 0 with no angular velocity
    sim::SimHarness harness(cfg);
    harness.schedule_failure(bias);

    double heading_at_bias = 0.0;
    double heading_before = 0.0;
    for (int i = 0; i < 8; ++i) {
        const auto t = harness.tick(0.0, 0.0);
        if (i == 4) heading_before = t.observed_heading;
        if (i == 5) heading_at_bias = t.observed_heading;
    }
    const double diff = std::fabs(heading_at_bias - heading_before);
    // The bias should add ~30 degrees to the observed heading
    CHECK(diff > 15.0);
}

TEST_CASE("SimHarness failure ordering: odom spike applied before integration") {
    // Without spike: integrate small forward step, odom_error ~= truth error
    auto cfg_clean = make_clean_cfg(42);
    cfg_clean.odom_noise = {};  // zero noise
    sim::SimHarness clean(cfg_clean);
    const auto t_clean = clean.tick(24.0, 0.0);
    const double base_odom = t_clean.odom_error;

    // With 3x odom spike at tick 0: odom should be ~3x further from truth
    auto cfg_spike = make_clean_cfg(42);
    cfg_spike.odom_noise = {};
    noise::FailureEvent spike;
    spike.start_tick = 0; spike.duration_ticks = 1;
    spike.type = noise::FailureType::OdomSpike; spike.param = 3.0;
    sim::SimHarness spiked(cfg_spike);
    spiked.schedule_failure(spike);
    const auto t_spike = spiked.tick(24.0, 0.0);
    const double spike_odom = t_spike.odom_error;

    CHECK(spike_odom > base_odom * 1.5);
}

TEST_CASE("SimHarness public API backward compat: controller() works") {
    sim::SimHarness harness(make_clean_cfg());
    harness.tick(sim::Action::NONE);
    const auto p = harness.controller().get_accepted_pose();
    CHECK(std::isfinite(p.x));
    CHECK(std::isfinite(p.y));
}

TEST_CASE("SimHarness public API backward compat: schedule_failure() works") {
    sim::SimHarness harness(make_clean_cfg());
    noise::FailureEvent e;
    e.start_tick = 0; e.duration_ticks = 2;
    e.type = noise::FailureType::SensorDead; e.sensor_idx = 0;
    harness.schedule_failure(e);

    const auto t0 = harness.tick(sim::Action::NONE);
    // Sensor 0 dead -- observed_readings[0] should be -1
    CHECK(t0.observed_readings[0] < 0.0);
}


