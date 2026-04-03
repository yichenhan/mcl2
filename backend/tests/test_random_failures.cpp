#include "doctest/doctest.h"
#include "noise/failure_config.hpp"

TEST_CASE("generate_random_failures deterministic with same seed") {
    pursuit::FailureGenConfig cfg;
    cfg.sensor_dead_prob = 0.02;
    cfg.min_duration_ticks = 4;
    cfg.max_duration_ticks = 8;
    const auto a = pursuit::generate_random_failures(cfg, 120, 1234, 72.0);
    const auto b = pursuit::generate_random_failures(cfg, 120, 1234, 72.0);
    REQUIRE(a.size() == b.size());
    for (size_t i = 0; i < a.size(); ++i) {
        CHECK(a[i].type == b[i].type);
        CHECK(a[i].start_tick == b[i].start_tick);
        CHECK(a[i].duration_ticks == b[i].duration_ticks);
        CHECK(a[i].sensor_idx == b[i].sensor_idx);
        CHECK(a[i].param == doctest::Approx(b[i].param));
    }
}

TEST_CASE("generate_random_failures with zero probs returns empty") {
    pursuit::FailureGenConfig cfg;
    const auto ev = pursuit::generate_random_failures(cfg, 200, 7, 72.0);
    CHECK(ev.empty());
}

TEST_CASE("spurious reflections are always single-tick events") {
    pursuit::FailureGenConfig cfg;
    cfg.spurious_reflection_prob = 0.5;
    cfg.min_duration_ticks = 5;
    cfg.max_duration_ticks = 25;

    const auto ev = pursuit::generate_random_failures(cfg, 500, 123, 72.0);
    bool saw_spurious = false;
    for (const auto& e : ev) {
        if (e.type != noise::FailureType::SpuriousReflection) continue;
        saw_spurious = true;
        CHECK(e.duration_ticks == 1);
    }
    CHECK(saw_spurious);
}

