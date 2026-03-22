#include "doctest/doctest.h"

#include "mcl/mcl_controller.hpp"
#include "ray/ray_cast_obstacles.hpp"

namespace {

const mcl::ReplayConfig kReplayConfig{
    "replays/test_mcl_controller",
    "test_session"
};

void set_all_particles_to(mcl::MCLController& c, float x, float y) {
    auto& particles = const_cast<std::vector<mcl::Particle>&>(c.particles());
    if (particles.empty()) return;
    const float w = 1.0f / static_cast<float>(particles.size());
    for (auto& p : particles) {
        p.x = x;
        p.y = y;
        p.weight = w;
    }
}

} // namespace

TEST_CASE("MCLController accepts large jump with no velocity gate") {
    mcl::MCLConfig mcfg;
    mcfg.num_particles = 100;
    mcl::GateConfig gcfg;
    gcfg.max_radius_90_in = 1e6;
    gcfg.max_spread_in = 1e6;
    gcfg.max_sensor_residual_in = 1e6;
    gcfg.min_valid_sensors_for_residual = 0;

    mcl::MCLController c(kReplayConfig, mcfg, gcfg);
    c.initialize_uniform(1);
    set_all_particles_to(c, 30.0f, 0.0f);

    sim::Field field;
    mcl::Estimate prev{0.0f, 0.0f};
    std::array<double, 4> readings{-1.0, -1.0, -1.0, -1.0};
    const auto d = c.gate_estimate(field, readings, 0.0, prev);
    CHECK(d.accepted);
}

TEST_CASE("MCLController gate rejects spread ambiguity") {
    mcl::MCLConfig mcfg;
    mcfg.num_particles = 100;
    mcl::GateConfig gcfg;
    gcfg.max_radius_90_in = 5.0;
    gcfg.max_spread_in = 5.0;
    gcfg.max_sensor_residual_in = 1e6;
    gcfg.min_valid_sensors_for_residual = 0;

    mcl::MCLController c(kReplayConfig, mcfg, gcfg);
    c.initialize_uniform(2);
    auto& particles = const_cast<std::vector<mcl::Particle>&>(c.particles());
    const float w = 1.0f / static_cast<float>(particles.size());
    for (size_t i = 0; i < particles.size(); ++i) {
        particles[i].x = (i % 2 == 0) ? -50.0f : 50.0f;
        particles[i].y = 0.0f;
        particles[i].weight = w;
    }

    sim::Field field;
    mcl::Estimate prev{0.0f, 0.0f};
    std::array<double, 4> readings{-1.0, -1.0, -1.0, -1.0};
    const auto d = c.gate_estimate(field, readings, 0.0, prev);
    CHECK_FALSE(d.accepted);
    CHECK(d.failed_spread);
}

TEST_CASE("MCLController gate rejects non-passable estimate") {
    mcl::MCLConfig mcfg;
    mcfg.num_particles = 100;
    mcl::GateConfig gcfg;
    gcfg.max_radius_90_in = 1e6;
    gcfg.max_spread_in = 1e6;
    gcfg.max_sensor_residual_in = 1e6;
    gcfg.min_valid_sensors_for_residual = 0;

    mcl::MCLController c(kReplayConfig, mcfg, gcfg);
    c.initialize_uniform(3);
    set_all_particles_to(c, 0.0f, 0.0f);

    sim::Field field;
    field.obstacles.push_back(sim::AABB{-5.0, -5.0, 5.0, 5.0});
    mcl::Estimate prev{0.0f, 0.0f};
    std::array<double, 4> readings{-1.0, -1.0, -1.0, -1.0};
    const auto d = c.gate_estimate(field, readings, 0.0, prev);
    CHECK_FALSE(d.accepted);
    CHECK(d.failed_passability);
}

TEST_CASE("MCLController gate rejects high per-sensor residual") {
    mcl::MCLConfig mcfg;
    mcfg.num_particles = 100;
    mcl::GateConfig gcfg;
    gcfg.max_radius_90_in = 1e6;
    gcfg.max_spread_in = 1e6;
    gcfg.max_sensor_residual_in = 1.0;
    gcfg.min_valid_sensors_for_residual = 1;

    mcl::MCLController c(kReplayConfig, mcfg, gcfg);
    c.initialize_uniform(4);
    set_all_particles_to(c, 0.0f, 0.0f);

    sim::Field field;
    mcl::Estimate prev{0.0f, 0.0f};
    std::array<double, 4> readings{0.0, -1.0, -1.0, -1.0}; // intentionally impossible
    const auto d = c.gate_estimate(field, readings, 0.0, prev);
    CHECK_FALSE(d.accepted);
    CHECK(d.failed_residual);
}

TEST_CASE("MCLController wall-sum only enforced when pair exists") {
    mcl::MCLConfig mcfg;
    mcfg.num_particles = 100;
    mcl::GateConfig gcfg;
    gcfg.max_radius_90_in = 1e6;
    gcfg.max_spread_in = 1e6;
    gcfg.max_sensor_residual_in = 1e6;
    gcfg.min_valid_sensors_for_residual = 0;
    gcfg.wall_sum_tolerance_in = 1.0;

    mcl::MCLController c(kReplayConfig, mcfg, gcfg);
    c.initialize_uniform(5);
    set_all_particles_to(c, 0.0f, 0.0f);
    sim::Field field;
    mcl::Estimate prev{0.0f, 0.0f};

    // Only one axis sensor => wall-sum gate should not fire.
    std::array<double, 4> one_axis{5.0, -1.0, -1.0, -1.0};
    const auto ok = c.gate_estimate(field, one_axis, 0.0, prev);
    CHECK(ok.accepted);

    // Paired X-axis readings with impossible sum => wall-sum should fail.
    std::array<double, 4> paired_x{5.0, 5.0, -1.0, -1.0};
    const auto bad = c.gate_estimate(field, paired_x, 0.0, prev);
    CHECK_FALSE(bad.accepted);
    CHECK(bad.failed_wall_sum);
}
