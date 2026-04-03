#include "doctest/doctest.h"

#include "state/session_analytics.hpp"
#include "stress/stress_generators.hpp"
#include "sim/sim_harness.hpp"

#include <cmath>
#include <variant>

TEST_CASE("generate_route produces requested tick count") {
    sim::Field field;
    const auto route = stress::generate_route(field, 240, 12345u);
    CHECK(static_cast<int>(route.velocity_commands.size()) == 240);
}

TEST_CASE("generate_route start pose is passable") {
    sim::Field field;
    for (int i = 0; i < 10; ++i) {
        const auto route = stress::generate_route(field, 50, static_cast<uint64_t>(100 + i));
        CHECK(field.is_passable(route.start_pose.x, route.start_pose.y));
    }
}

TEST_CASE("generate_random_obstacles stay inside field") {
    sim::Field field;
    const auto obs = stress::generate_random_obstacles(999u, field, 5);
    REQUIRE(obs.size() == 5);
    const double fh = field.field_half;
    for (const auto& o : obs) {
        if (const auto* aabb = std::get_if<sim::AABB>(&o.shape)) {
            CHECK(aabb->min_x >= -fh);
            CHECK(aabb->max_x <= fh);
            CHECK(aabb->min_y >= -fh);
            CHECK(aabb->max_y <= fh);
        }
    }
}

TEST_CASE("generate_stress_noise profile 0 has mild noise") {
    const auto b = stress::generate_stress_noise(0, 500, 42u, 72.0);
    CHECK(b.odom.trans_noise_frac <= 0.06);
}

TEST_CASE("generate_stress_noise profile 5 enables kidnap generation") {
    const auto b = stress::generate_stress_noise(5, 800, 42u, 72.0);
    CHECK(b.failure_gen.kidnap_prob > 0.0);
}

TEST_CASE("apply_failure_guardrails bumps early events to tick 50") {
    std::vector<noise::FailureEvent> ev;
    noise::FailureEvent e;
    e.start_tick = 10;
    e.duration_ticks = 5;
    e.type = noise::FailureType::SensorDead;
    e.sensor_idx = 0;
    ev.push_back(e);
    stress::apply_failure_guardrails(ev, 200);
    CHECK(ev[0].start_tick >= 50);
}

TEST_CASE("stress pipeline three short scenarios produces sorted analytics") {
    struct Row {
        state::SessionAnalytics a;
    };
    std::vector<Row> rows;
    for (int si = 0; si < 3; ++si) {
        const uint64_t seed = 500u + static_cast<uint64_t>(si);
        sim::Field field;
        field.obstacles = stress::generate_random_obstacles(seed, field, 2);
        stress::RouteResult route = stress::generate_route(field, 30, seed);
        stress::StressNoiseBundle noise = stress::generate_stress_noise(si % 6, 30, seed, field.field_half);
        sim::SimHarness::Config cfg;
        cfg.initial_state = route.start_pose;
        cfg.controller_config.field = field;
        cfg.odom_noise = noise.odom;
        cfg.sensor_noise = noise.sensor;
        cfg.controller_config.seed = static_cast<uint32_t>(seed & 0xFFFFFFFFu);
        sim::SimHarness harness(cfg);
        for (const auto& fe : noise.failure_events) harness.schedule_failure(fe);
        for (const auto& cmd : route.velocity_commands) harness.tick(cmd.first, cmd.second);
        rows.push_back({state::compute_analytics(harness.history())});
    }
    std::sort(rows.begin(), rows.end(), [](const Row& x, const Row& y) {
        return x.a.peak_accepted_error > y.a.peak_accepted_error;
    });
    CHECK(rows[0].a.peak_accepted_error >= rows[1].a.peak_accepted_error);
    CHECK(rows[1].a.peak_accepted_error >= rows[2].a.peak_accepted_error);
}
