#include "doctest/doctest.h"

#include "sim/sim_harness.hpp"
#include "state/sim_session.hpp"

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
    e.start_tick = 0;
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

TEST_CASE("SimHarness and SimSession remain close for same seed") {
    state::SimSessionConfig scfg;
    scfg.seed = 123;
    scfg.mcl_config.num_particles = 80;
    scfg.sensor_noise_config.dropout_probability = 0.0;
    scfg.sensor_noise_config.long_dropout_probability = 0.0;
    scfg.sensor_noise_config.gaussian_stddev_mm = 0.0;
    scfg.sensor_noise_config.range_noise_slope = 0.0;
    scfg.sensor_noise_config.spurious_reflection_probability = 0.0;
    state::SimSession session(scfg);

    sim::SimHarness::Config hcfg;
    hcfg.controller_config.seed = scfg.seed;
    hcfg.controller_config.mcl_config = scfg.mcl_config;
    hcfg.controller_config.gate_config = scfg.mcl_gate_config;
    hcfg.controller_config.field = scfg.field;
    hcfg.controller_config.min_sensors_for_update = scfg.min_sensors_for_update;
    hcfg.controller_config.tick_dt_sec = scfg.physics_config.dt;
    hcfg.initial_state = scfg.initial_state;
    hcfg.physics_config = scfg.physics_config;
    hcfg.odom_noise = scfg.odom_noise_config;
    hcfg.sensor_noise = scfg.sensor_noise_config;
    sim::SimHarness harness(hcfg);

    for (int i = 0; i < 10; ++i) {
        const auto a = session.tick(sim::Action::FORWARD);
        const auto b = harness.tick(sim::Action::FORWARD);
        CHECK(a.ground_truth.x == doctest::Approx(b.ground_truth.x).epsilon(1e-6));
        CHECK(a.ground_truth.y == doctest::Approx(b.ground_truth.y).epsilon(1e-6));
        CHECK(std::isfinite(a.mcl_error));
        CHECK(std::isfinite(b.mcl_error));
        CHECK(std::fabs(a.mcl_error - b.mcl_error) < 5.0);
    }
}

