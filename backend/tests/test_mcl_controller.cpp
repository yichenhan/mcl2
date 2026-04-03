#include "doctest/doctest.h"

#include "mcl/mcl_controller.hpp"
#include "ray/ray_cast_obstacles.hpp"

#include "nlohmann/json.hpp"

#include <cmath>

namespace {

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

TEST_CASE("MCLController rejects centroid jump") {
    mcl::MCLConfig mcfg;
    mcfg.num_particles = 100;
    mcl::GateConfig gcfg;
    gcfg.max_centroid_jump_ft_per_s = 1.0;
    gcfg.max_jump_in = 100.0;
    gcfg.max_radius_90_in = 1e6;
    gcfg.min_valid_sensors_for_residual = 0;

    mcl::MCLController c(mcfg, gcfg);
    c.initialize_uniform(1);
    set_all_particles_to(c, 30.0f, 0.0f);

    sim::Field field;
    mcl::Estimate prev{0.0f, 0.0f};
    std::array<double, 4> readings{-1.0, -1.0, -1.0, -1.0};
    const auto d = c.gate_estimate(field, readings, 0.0, prev, 0.05);
    CHECK_FALSE(d.accepted);
    CHECK(d.failed_centroid_jump);
}

TEST_CASE("MCLController gate rejects high r90") {
    mcl::MCLConfig mcfg;
    mcfg.num_particles = 100;
    mcl::GateConfig gcfg;
    gcfg.max_centroid_jump_ft_per_s = 1e6;
    gcfg.max_jump_in = 1e6;
    gcfg.max_radius_90_in = 5.0;
    gcfg.min_valid_sensors_for_residual = 0;

    mcl::MCLController c(mcfg, gcfg);
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
    const auto d = c.gate_estimate(field, readings, 0.0, prev, 0.05);
    CHECK_FALSE(d.accepted);
    CHECK(d.failed_r90);
}

TEST_CASE("MCLController gate rejects non-passable estimate") {
    mcl::MCLConfig mcfg;
    mcfg.num_particles = 100;
    mcl::GateConfig gcfg;
    gcfg.max_centroid_jump_ft_per_s = 1e6;
    gcfg.max_jump_in = 1e6;
    gcfg.max_radius_90_in = 1e6;
    gcfg.min_valid_sensors_for_residual = 0;

    mcl::MCLController c(mcfg, gcfg);
    c.initialize_uniform(3);
    set_all_particles_to(c, 0.0f, 0.0f);

    sim::Field field;
    field.obstacles.push_back(sim::AABB{-5.0, -5.0, 5.0, 5.0});
    mcl::Estimate prev{0.0f, 0.0f};
    std::array<double, 4> readings{-1.0, -1.0, -1.0, -1.0};
    const auto d = c.gate_estimate(field, readings, 0.0, prev, 0.05);
    CHECK_FALSE(d.accepted);
    CHECK(d.failed_passability);
}

TEST_CASE("MCLController gate rejects high per-sensor residual") {
    mcl::MCLConfig mcfg;
    mcfg.num_particles = 100;
    mcl::GateConfig gcfg;
    gcfg.max_centroid_jump_ft_per_s = 1e6;
    gcfg.max_jump_in = 1e6;
    gcfg.max_radius_90_in = 1e6;
    gcfg.min_valid_sensors_for_residual = 1;

    mcl::MCLController c(mcfg, gcfg);
    c.initialize_uniform(4);
    set_all_particles_to(c, 0.0f, 0.0f);

    sim::Field field;
    mcl::Estimate prev{0.0f, 0.0f};
    std::array<double, 4> readings{0.0, -1.0, -1.0, -1.0}; // intentionally impossible
    const auto d = c.gate_estimate(field, readings, 0.0, prev, 0.05);
    CHECK_FALSE(d.accepted);
    CHECK(d.failed_residual);
}

TEST_CASE("MCLController tick output serializes and round-trips") {
    mcl::MCLConfig mcfg;
    mcfg.num_particles = 50;
    mcl::GateConfig gcfg;
    gcfg.max_centroid_jump_ft_per_s = 1e6;
    gcfg.max_jump_in = 1e6;
    gcfg.max_radius_90_in = 1e6;
    gcfg.min_valid_sensors_for_residual = 0;

    mcl::MCLController c(mcfg, gcfg);
    c.initialize_uniform(42);

    sim::Field field;
    const std::array<double, 4> readings{10.0, 10.0, -1.0, -1.0};
    const mcl::Estimate prev{0.0f, 0.0f};
    const mcl::MCLTickResult out = c.tick(
        0.0,
        0.0,
        0.0,
        0.0,
        readings,
        2,
        &field,
        &prev,
        0.05);

    CHECK(out.valid_sensor_count == 2);
    CHECK(out.update_skipped == false);
    CHECK(out.post_predict.particles.size() == static_cast<size_t>(mcfg.num_particles));
    CHECK(out.post_update.particles.size() == static_cast<size_t>(mcfg.num_particles));
    CHECK(out.post_resample.particles.size() == static_cast<size_t>(mcfg.num_particles));

    const nlohmann::json j = out;
    const mcl::MCLTickResult decoded = j.get<mcl::MCLTickResult>();
    CHECK(decoded.valid_sensor_count == out.valid_sensor_count);
    CHECK(decoded.update_skipped == out.update_skipped);
    CHECK(decoded.gate.accepted == out.gate.accepted);
    CHECK(decoded.post_resample.particles.size() == out.post_resample.particles.size());
}

TEST_CASE("MCLController tick exposes odom pose and sensor diagnostics") {
    mcl::MCLConfig mcfg;
    mcfg.num_particles = 50;
    mcl::GateConfig gcfg;
    gcfg.max_centroid_jump_ft_per_s = 1e6;
    gcfg.max_jump_in = 1e6;
    gcfg.max_radius_90_in = 1e6;
    gcfg.min_valid_sensors_for_residual = 0;

    mcl::MCLController c(mcfg, gcfg);
    c.initialize_uniform(84);
    set_all_particles_to(c, 0.0f, 0.0f);

    sim::Field field;
    const std::array<double, 4> readings{10.0, 20.0, -1.0, 5.0};
    const mcl::Estimate prev{0.0f, 0.0f};
    const mcl::Pose odom{12.0, -3.0, 90.0};
    const mcl::MCLTickResult out = c.tick(
        0.0,
        0.0,
        0.0,
        0.0,
        readings,
        2,
        &field,
        &prev,
        0.05,
        nullptr,
        &odom);

    CHECK(out.odom_pose.x == doctest::Approx(12.0));
    CHECK(out.odom_pose.y == doctest::Approx(-3.0));
    CHECK(out.odom_pose.theta == doctest::Approx(90.0));
    CHECK(out.observed_readings[0] == doctest::Approx(10.0));
    CHECK(out.observed_readings[1] == doctest::Approx(20.0));
    CHECK(out.observed_readings[2] == doctest::Approx(-1.0));
    CHECK(out.observed_readings[3] == doctest::Approx(5.0));

    CHECK(std::isfinite(out.mcl_predicted_readings[0]));
    CHECK(std::isfinite(out.mcl_predicted_readings[1]));
    CHECK(out.mcl_predicted_readings[2] == doctest::Approx(-1.0));
    CHECK(std::isfinite(out.mcl_predicted_readings[3]));
    CHECK(out.mcl_sensor_residuals[2] == doctest::Approx(0.0));
    CHECK(out.mcl_sensor_residuals[0] == doctest::Approx(std::fabs(out.mcl_predicted_readings[0] - 10.0)));
    CHECK(out.mcl_sensor_residuals[1] == doctest::Approx(std::fabs(out.mcl_predicted_readings[1] - 20.0)));
    CHECK(out.mcl_sensor_residuals[3] == doctest::Approx(std::fabs(out.mcl_predicted_readings[3] - 5.0)));

    const nlohmann::json j = out;
    const mcl::MCLTickResult decoded = j.get<mcl::MCLTickResult>();
    CHECK(decoded.odom_pose.x == doctest::Approx(out.odom_pose.x));
    CHECK(decoded.odom_pose.y == doctest::Approx(out.odom_pose.y));
    CHECK(decoded.odom_pose.theta == doctest::Approx(out.odom_pose.theta));
    CHECK(decoded.observed_readings[0] == doctest::Approx(out.observed_readings[0]));
    CHECK(decoded.observed_readings[1] == doctest::Approx(out.observed_readings[1]));
    CHECK(decoded.observed_readings[2] == doctest::Approx(out.observed_readings[2]));
    CHECK(decoded.observed_readings[3] == doctest::Approx(out.observed_readings[3]));
    CHECK(decoded.mcl_predicted_readings[0] == doctest::Approx(out.mcl_predicted_readings[0]));
    CHECK(decoded.mcl_sensor_residuals[0] == doctest::Approx(out.mcl_sensor_residuals[0]));
}

TEST_CASE("MCLController tick keeps default diagnostics without pose or field") {
    mcl::MCLConfig mcfg;
    mcfg.num_particles = 20;
    mcl::GateConfig gcfg;
    gcfg.max_centroid_jump_ft_per_s = 1e6;
    gcfg.max_jump_in = 1e6;
    gcfg.max_radius_90_in = 1e6;
    gcfg.min_valid_sensors_for_residual = 0;

    mcl::MCLController c(mcfg, gcfg);
    c.initialize_uniform(99);

    const std::array<double, 4> readings{-1.0, -1.0, -1.0, -1.0};
    const mcl::MCLTickResult out = c.tick(
        0.0,
        0.0,
        0.0,
        0.0,
        readings,
        2,
        nullptr,
        nullptr,
        0.05);

    CHECK(out.odom_pose.x == doctest::Approx(0.0));
    CHECK(out.odom_pose.y == doctest::Approx(0.0));
    CHECK(out.odom_pose.theta == doctest::Approx(0.0));
    CHECK(out.mcl_predicted_readings[0] == doctest::Approx(-1.0));
    CHECK(out.mcl_predicted_readings[1] == doctest::Approx(-1.0));
    CHECK(out.mcl_predicted_readings[2] == doctest::Approx(-1.0));
    CHECK(out.mcl_predicted_readings[3] == doctest::Approx(-1.0));
    CHECK(out.mcl_sensor_residuals[0] == doctest::Approx(0.0));
    CHECK(out.mcl_sensor_residuals[1] == doctest::Approx(0.0));
    CHECK(out.mcl_sensor_residuals[2] == doctest::Approx(0.0));
    CHECK(out.mcl_sensor_residuals[3] == doctest::Approx(0.0));
}

TEST_CASE("gate_estimate populates would_fail_* for all gates even when r90 rejects") {
    mcl::MCLConfig mcfg;
    mcfg.num_particles = 100;
    mcl::GateConfig gcfg;
    gcfg.max_radius_90_in = 5.0;
    gcfg.max_centroid_jump_ft_per_s = 1e6;
    gcfg.min_valid_sensors_for_residual = 0;

    mcl::MCLController c(mcfg, gcfg);
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
    const auto d = c.gate_estimate(field, readings, 0.0, prev, 0.05);

    CHECK_FALSE(d.accepted);
    CHECK(d.failed_r90);
    CHECK(d.would_fail_r90);
    CHECK_FALSE(d.would_fail_centroid_jump);
    CHECK_FALSE(d.would_fail_passability);
    CHECK(d.n_eff_at_gate > 0.0);
}

TEST_CASE("gate_estimate records max_sensor_residual_in and residual_threshold_in") {
    mcl::MCLConfig mcfg;
    mcfg.num_particles = 100;
    mcl::GateConfig gcfg;
    gcfg.max_radius_90_in = 1e6;
    gcfg.max_centroid_jump_ft_per_s = 1e6;
    gcfg.min_valid_sensors_for_residual = 1;

    mcl::MCLController c(mcfg, gcfg);
    c.initialize_uniform(1);
    set_all_particles_to(c, 0.0f, 0.0f);

    sim::Field field;
    mcl::Estimate prev{0.0f, 0.0f};
    std::array<double, 4> readings{50.0, 50.0, 50.0, 50.0};
    const auto d = c.gate_estimate(field, readings, 0.0, prev, 0.05);

    CHECK(d.max_sensor_residual_in >= 0.0);
    CHECK(d.residual_threshold_in > 0.0);
    CHECK(std::isfinite(d.centroid_jump_ft_per_s));
}

TEST_CASE("GateDecision JSON roundtrip preserves new fields") {
    mcl::GateDecision d;
    d.would_fail_centroid_jump = true;
    d.would_fail_r90 = false;
    d.would_fail_passability = true;
    d.would_fail_residual = false;
    d.max_sensor_residual_in = 1.23;
    d.residual_threshold_in = 0.98;
    d.centroid_jump_ft_per_s = 5.67;
    d.n_eff_at_gate = 150.0;

    const nlohmann::json j = d;
    const mcl::GateDecision rt = j.get<mcl::GateDecision>();
    CHECK(rt.would_fail_centroid_jump == true);
    CHECK(rt.would_fail_r90 == false);
    CHECK(rt.would_fail_passability == true);
    CHECK(rt.would_fail_residual == false);
    CHECK(rt.max_sensor_residual_in == doctest::Approx(1.23));
    CHECK(rt.residual_threshold_in == doctest::Approx(0.98));
    CHECK(rt.centroid_jump_ft_per_s == doctest::Approx(5.67));
    CHECK(rt.n_eff_at_gate == doctest::Approx(150.0));
}
