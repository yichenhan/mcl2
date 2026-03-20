#include "doctest/doctest.h"
#include "mcl/mcl_engine.hpp"
#include "state/sim_session.hpp"

TEST_CASE("cluster_stats: uniform cloud has large spread and radius_90") {
    mcl::MCLConfig cfg;
    cfg.num_particles = 300;
    cfg.field_half = 72.0;
    mcl::MCLEngine engine(cfg);
    engine.initialize_uniform(42);

    auto cs = engine.cluster_stats();
    CHECK(cs.spread > 30.0);
    CHECK(cs.radius_90 > 40.0);
}

TEST_CASE("cluster_stats: tight cloud at origin has small spread") {
    mcl::MCLConfig cfg;
    cfg.num_particles = 100;
    cfg.field_half = 72.0;
    mcl::MCLEngine engine(cfg);
    engine.initialize_uniform(99);

    auto& particles = const_cast<std::vector<mcl::Particle>&>(engine.particles());
    float w = 1.0f / 100.0f;
    for (auto& p : particles) {
        p.x = 10.0f + (p.x / 144.0f);
        p.y = 20.0f + (p.y / 144.0f);
        p.weight = w;
    }

    auto cs = engine.cluster_stats();
    CHECK(cs.spread < 2.0);
    CHECK(cs.radius_90 < 2.0);
    CHECK(cs.centroid.x == doctest::Approx(10.0).epsilon(0.5));
    CHECK(cs.centroid.y == doctest::Approx(20.0).epsilon(0.5));
}

TEST_CASE("cluster_stats: radius_90 ignores outlier particles") {
    mcl::MCLConfig cfg;
    cfg.num_particles = 100;
    cfg.field_half = 72.0;
    mcl::MCLEngine engine(cfg);
    engine.initialize_uniform(77);

    auto& particles = const_cast<std::vector<mcl::Particle>&>(engine.particles());
    float w = 1.0f / 100.0f;
    for (int i = 0; i < 95; ++i) {
        particles[i].x = 0.0f;
        particles[i].y = 0.0f;
        particles[i].weight = w;
    }
    for (int i = 95; i < 100; ++i) {
        particles[i].x = 70.0f;
        particles[i].y = 70.0f;
        particles[i].weight = w;
    }

    auto cs = engine.cluster_stats();
    CHECK(cs.radius_90 < 5.0);
    CHECK(cs.spread > 5.0);
}

TEST_CASE("cluster_stats: empty engine returns zeros") {
    mcl::MCLConfig cfg;
    cfg.num_particles = 0;
    mcl::MCLEngine engine(cfg);

    auto cs = engine.cluster_stats();
    CHECK(cs.spread == 0.0);
    CHECK(cs.radius_90 == 0.0);
}

TEST_CASE("MCLSnapshot includes spread and radius_90") {
    state::SimSessionConfig cfg;
    cfg.seed = 100;
    cfg.mcl_config.num_particles = 100;
    cfg.mcl_config.field_half = 72.0;
    cfg.initial_state = {0.0, 0.0, 0.0};
    state::SimSession session(cfg);
    auto t = session.tick(sim::Action::NONE);

    CHECK(t.post_resample.spread >= 0.0);
    CHECK(t.post_resample.radius_90 >= 0.0);
}

TEST_CASE("pose_gated is true when radius_90 exceeds threshold") {
    state::SimSessionConfig cfg;
    cfg.seed = 100;
    cfg.mcl_config.num_particles = 100;
    cfg.mcl_config.field_half = 72.0;
    cfg.initial_state = {0.0, 0.0, 0.0};
    cfg.pose_gate_radius_90 = 1.0;
    state::SimSession session(cfg);
    auto t = session.tick(sim::Action::NONE);

    CHECK(t.post_resample.radius_90 > 1.0);
    CHECK(t.pose_gated);
}

TEST_CASE("pose_gated is false when threshold is very large") {
    state::SimSessionConfig cfg;
    cfg.seed = 100;
    cfg.mcl_config.num_particles = 100;
    cfg.mcl_config.field_half = 72.0;
    cfg.initial_state = {0.0, 0.0, 0.0};
    cfg.pose_gate_radius_90 = 999.0;
    state::SimSession session(cfg);
    auto t = session.tick(sim::Action::NONE);

    CHECK_FALSE(t.pose_gated);
}

TEST_CASE("default pose_gate_radius_90 is 20") {
    state::SimSessionConfig cfg;
    CHECK(cfg.pose_gate_radius_90 == doctest::Approx(20.0));
}
