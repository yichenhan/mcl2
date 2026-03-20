#include "doctest/doctest.h"
#include "mcl/mcl_engine.hpp"
#include "state/sim_session.hpp"

static state::SimSessionConfig make_config(int min_sensors) {
    state::SimSessionConfig cfg;
    cfg.seed = 100;
    cfg.min_sensors_for_update = min_sensors;
    cfg.mcl_config.num_particles = 100;
    cfg.mcl_config.field_half = 72.0;
    cfg.initial_state = {0.0, 0.0, 0.0};
    cfg.sensor_noise_config.dropout_probability = 0.0;
    cfg.sensor_noise_config.long_dropout_probability = 0.0;
    return cfg;
}

TEST_CASE("update_skipped is false when enough sensors are valid") {
    auto cfg = make_config(2);
    state::SimSession session(cfg);
    auto t = session.tick(sim::Action::NONE);
    CHECK(t.valid_sensor_count >= 2);
    CHECK_FALSE(t.update_skipped);
}

TEST_CASE("update_skipped is true when threshold set impossibly high") {
    auto cfg = make_config(5);
    state::SimSession session(cfg);
    auto t = session.tick(sim::Action::NONE);
    CHECK(t.valid_sensor_count <= 4);
    CHECK(t.update_skipped);
}

TEST_CASE("skipping update preserves predict-phase particles") {
    auto cfg = make_config(5);
    state::SimSession session(cfg);
    auto t = session.tick(sim::Action::NONE);

    CHECK(t.update_skipped);
    CHECK(t.post_predict.particles.size() == t.post_update.particles.size());
    for (size_t i = 0; i < t.post_predict.particles.size(); ++i) {
        CHECK(t.post_predict.particles[i].x == t.post_update.particles[i].x);
        CHECK(t.post_predict.particles[i].y == t.post_update.particles[i].y);
        CHECK(t.post_predict.particles[i].weight == t.post_update.particles[i].weight);
    }
}

TEST_CASE("threshold=0 never skips update") {
    auto cfg = make_config(0);
    state::SimSession session(cfg);
    for (int i = 0; i < 10; ++i) {
        auto t = session.tick(sim::Action::NONE);
        CHECK_FALSE(t.update_skipped);
    }
}

TEST_CASE("default threshold is 2") {
    state::SimSessionConfig cfg;
    CHECK(cfg.min_sensors_for_update == 2);
}
