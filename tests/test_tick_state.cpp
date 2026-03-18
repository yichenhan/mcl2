#include "doctest/doctest.h"

#include "nlohmann/json.hpp"
#include "state/sim_session.hpp"
#include "state/tick_state.hpp"

#include <cmath>

TEST_CASE("TickState JSON roundtrip preserves core fields") {
    state::TickState t;
    t.tick = 7;
    t.ground_truth = sim::RobotState{1.5, -2.5, 45.0};
    t.observed_readings = {10.0, 11.0, -1.0, 13.0};
    t.observed_heading = 44.8;
    t.active_failures = {"sensor_dead"};
    t.post_resample.estimate = mcl::Estimate{3.0f, 4.0f};
    t.mcl_error = 2.2;
    t.odom_error = 1.1;
    t.valid_sensor_count = 3;

    const nlohmann::json j = t;
    const state::TickState rt = j.get<state::TickState>();
    CHECK(rt.tick == t.tick);
    CHECK(rt.ground_truth.x == doctest::Approx(t.ground_truth.x));
    CHECK(rt.observed_readings[2] == doctest::Approx(-1.0));
    CHECK(rt.active_failures.size() == 1);
    CHECK(rt.valid_sensor_count == 3);
}

TEST_CASE("SimSession tick emits populated snapshots and metrics") {
    state::SimSessionConfig cfg;
    cfg.seed = 123;
    cfg.mcl_config.num_particles = 120;
    cfg.sensor_noise_config.dropout_probability = 0.0;
    cfg.sensor_noise_config.long_dropout_probability = 0.0;
    cfg.sensor_noise_config.gaussian_stddev_mm = 0.0;
    cfg.sensor_noise_config.range_noise_slope = 0.0;
    cfg.sensor_noise_config.spurious_reflection_probability = 0.0;
    state::SimSession session(cfg);

    const state::TickState t0 = session.tick(sim::Action::NONE);
    CHECK(t0.tick == 0);
    CHECK(t0.post_predict.particles.size() == 120);
    CHECK(t0.post_update.particles.size() == 120);
    CHECK(t0.post_resample.particles.size() == 120);
    CHECK(t0.valid_sensor_count >= 1);
    CHECK(std::isfinite(t0.mcl_error));
    CHECK(std::isfinite(t0.odom_error));
}

TEST_CASE("SimSession tracks tick count and history size") {
    state::SimSession session;
    CHECK(session.current_tick() == 0);
    CHECK(session.history().empty());
    session.tick(sim::Action::FORWARD);
    session.tick(sim::Action::ROTATE_CW);
    CHECK(session.current_tick() == 2);
    CHECK(session.history().size() == 2);
}
