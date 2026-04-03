#include "doctest/doctest.h"

#include "nlohmann/json.hpp"
#include "sim/sim_harness.hpp"
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
    t.timestamp_iso = "2026-03-25T15:00:00Z";
    t.odom_pose = {1.0, 2.0, 90.0};
    t.raw_estimate = {5.0f, 6.0f};
    t.accepted_estimate = {7.0f, 8.0f};
    t.gate_decision.accepted = false;
    t.gate_decision.failed_r90 = true;
    t.gate_decision.reason = "r90";
    t.cluster_stats.spread = 2.5;
    t.cluster_stats.radius_90 = 1.25;
    t.cluster_stats.centroid = {9.0f, -1.0f};
    t.mcl_predicted_readings = {12.0, 13.0, 14.0, 15.0};
    t.sensor_residuals = {0.1, 0.2, 0.3, 0.4};

    const nlohmann::json j = t;
    const state::TickState rt = j.get<state::TickState>();
    CHECK(rt.tick == t.tick);
    CHECK(rt.ground_truth.x == doctest::Approx(t.ground_truth.x));
    CHECK(rt.observed_readings[2] == doctest::Approx(-1.0));
    CHECK(rt.active_failures.size() == 1);
    CHECK(rt.valid_sensor_count == 3);
    CHECK(rt.timestamp_iso == t.timestamp_iso);
    CHECK(rt.odom_pose.theta == doctest::Approx(90.0));
    CHECK(rt.raw_estimate.x == doctest::Approx(5.0f));
    CHECK(rt.accepted_estimate.y == doctest::Approx(8.0f));
    CHECK_FALSE(rt.gate_decision.accepted);
    CHECK(rt.gate_decision.failed_r90);
    CHECK(rt.gate_decision.reason == "r90");
    CHECK(rt.cluster_stats.spread == doctest::Approx(2.5));
    CHECK(rt.cluster_stats.centroid.x == doctest::Approx(9.0f));
    CHECK(rt.mcl_predicted_readings[3] == doctest::Approx(15.0));
    CHECK(rt.sensor_residuals[2] == doctest::Approx(0.3));
}

TEST_CASE("TickState JSON roundtrip preserves accepted_error") {
    state::TickState t;
    t.tick = 0;
    t.ground_truth = sim::RobotState{0, 0, 0};
    t.observed_readings = {10, 10, 10, 10};
    t.observed_heading = 0;
    t.mcl_error = 1.0;
    t.accepted_error = 4.56;
    t.odom_error = 2.0;
    const nlohmann::json j = t;
    CHECK(j.contains("accepted_error"));
    CHECK(j["accepted_error"].get<double>() == doctest::Approx(4.56));
    const state::TickState rt = j.get<state::TickState>();
    CHECK(rt.accepted_error == doctest::Approx(4.56));
}

TEST_CASE("TickState from_json defaults accepted_error to 0 when missing") {
    state::TickState t;
    t.tick = 0;
    t.ground_truth = sim::RobotState{0, 0, 0};
    t.observed_readings = {10, 10, 10, 10};
    t.observed_heading = 0;
    t.mcl_error = 1.0;
    t.odom_error = 2.0;
    nlohmann::json j = t;
    j.erase("accepted_error");
    const state::TickState rt = j.get<state::TickState>();
    CHECK(rt.accepted_error == doctest::Approx(0.0));
}

TEST_CASE("SimHarness tick emits populated snapshots and metrics") {
    sim::SimHarness::Config cfg;
    cfg.controller_config.seed = 123;
    cfg.controller_config.mcl_config.num_particles = 120;
    cfg.sensor_noise.dropout_probability = 0.0;
    cfg.sensor_noise.long_dropout_probability = 0.0;
    cfg.sensor_noise.gaussian_stddev_mm = 0.0;
    cfg.sensor_noise.range_noise_slope = 0.0;
    cfg.sensor_noise.spurious_reflection_probability = 0.0;
    sim::SimHarness harness(cfg);

    const state::TickState t0 = harness.tick(sim::Action::NONE);
    CHECK(t0.tick == 0);
    CHECK(t0.post_predict.particles.size() == 120);
    CHECK(t0.post_update.particles.size() == 120);
    CHECK(t0.post_resample.particles.size() == 120);
    CHECK(t0.valid_sensor_count >= 1);
    CHECK(std::isfinite(t0.mcl_error));
    CHECK(std::isfinite(t0.odom_error));
    CHECK(!t0.timestamp_iso.empty());
    CHECK(std::isfinite(t0.raw_estimate.x));
    CHECK(std::isfinite(t0.accepted_estimate.x));
    CHECK(std::isfinite(t0.accepted_error));
    CHECK(t0.accepted_error >= 0.0);
}

TEST_CASE("SimHarness tracks tick count and history size") {
    sim::SimHarness harness(sim::SimHarness::Config{});
    CHECK(harness.current_tick() == 0);
    CHECK(harness.history().empty());
    harness.tick(sim::Action::FORWARD);
    harness.tick(sim::Action::ROTATE_CW);
    CHECK(harness.current_tick() == 2);
    CHECK(harness.history().size() == 2);
}

// ============================================================================
// Phase 4: chassis_pose JSON backward compatibility
// ============================================================================

TEST_CASE("TickState JSON roundtrip preserves chassis_pose") {
    state::TickState t;
    t.tick = 1;
    t.ground_truth = sim::RobotState{0, 0, 0};
    t.observed_readings = {10, 10, 10, 10};
    t.chassis_pose = {1.5, -2.5, 45.0};

    const nlohmann::json j = t;
    const state::TickState rt = j.get<state::TickState>();
    CHECK(rt.chassis_pose.x == doctest::Approx(1.5));
    CHECK(rt.chassis_pose.y == doctest::Approx(-2.5));
    CHECK(rt.chassis_pose.theta == doctest::Approx(45.0));
}

TEST_CASE("TickState from_json falls back to accepted_estimate when chassis_pose missing") {
    state::TickState t;
    t.tick = 1;
    t.ground_truth = sim::RobotState{0, 0, 0};
    t.observed_readings = {10, 10, 10, 10};
    t.observed_heading = 30.0;
    t.accepted_estimate = {7.0f, 8.0f};

    nlohmann::json j = t;
    j.erase("chassis_pose");  // simulate old replay without chassis_pose

    const state::TickState rt = j.get<state::TickState>();
    CHECK(rt.chassis_pose.x == doctest::Approx(7.0).epsilon(0.01));
    CHECK(rt.chassis_pose.y == doctest::Approx(8.0).epsilon(0.01));
    CHECK(rt.chassis_pose.theta == doctest::Approx(30.0));
}

TEST_CASE("TickState from_json defaults to zero when both chassis_pose and accepted_estimate missing") {
    state::TickState t;
    t.tick = 1;
    t.ground_truth = sim::RobotState{0, 0, 0};
    t.observed_readings = {10, 10, 10, 10};

    nlohmann::json j = t;
    j.erase("chassis_pose");
    j.erase("accepted_estimate");

    const state::TickState rt = j.get<state::TickState>();
    CHECK(rt.chassis_pose.x == doctest::Approx(0.0));
    CHECK(rt.chassis_pose.y == doctest::Approx(0.0));
    CHECK(rt.chassis_pose.theta == doctest::Approx(0.0));
}
