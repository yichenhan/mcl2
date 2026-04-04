#include "doctest/doctest.h"
#include "mcl/mcl_engine.hpp"
#include "sim/field.hpp"
#include "sim/physics.hpp"
#include "distance_localization.hpp"
#include <cmath>
#include <random>
#include <vector>

using namespace mcl;
using namespace sim;
using namespace distance_loc;

// ============================================================================
// Helpers
// ============================================================================

static void perfect_readings(double x, double y, double heading_deg,
                             const MCLConfig& cfg, double out[4]) {
    Vec2 pos = { x, y };
    for (int i = 0; i < 4; i++) {
        double d = ray_distance_with_offset(pos, heading_deg,
                                            cfg.sensors[i].offset,
                                            cfg.sensors[i].angle_deg);
        out[i] = std::isfinite(d) ? d : -1.0;
    }
}

static double est_error(const Estimate& est, double tx, double ty) {
    double dx = est.x - tx;
    double dy = est.y - ty;
    return std::sqrt(dx * dx + dy * dy);
}

// Run one MCL tracking session: physics drives robot, MCL predicts + updates.
// Returns final error in inches.
static double run_tracking(
    double start_x, double start_y, double start_heading,
    const std::vector<Action>& actions,
    int convergence_warmup,       // ticks of stationary convergence first
    uint64_t seed,
    double* out_max_error = nullptr  // track worst-case error during movement
) {
    MCLConfig cfg;
    cfg.num_particles = 300;
    // Deterministic kinematics for regression thresholds (defaults add motion noise).
    cfg.predict_noise_fwd = 0.0;
    cfg.predict_noise_lat = 0.0;
    cfg.predict_noise_heading_deg = 0.0;
    cfg.roughening_sigma = 0.0;
    cfg.heading_uncertainty_deg = 0.0;

    Field field;
    PhysicsConfig pcfg;
    Physics phys(field, pcfg);
    phys.set_state({ start_x, start_y, start_heading });

    MCLEngine mcl(cfg);
    mcl.initialize_uniform(seed);

    // Warm-up: stationary convergence
    for (int t = 0; t < convergence_warmup; t++) {
        auto state = phys.state();
        double readings[4];
        perfect_readings(state.x, state.y, state.heading_deg, cfg, readings);
        mcl.predict(0.0, 0.0, state.heading_deg);
        mcl.update(readings, state.heading_deg);
        mcl.resample();
    }

    double max_err = 0.0;

    // Execute actions
    for (auto action : actions) {
        auto result = phys.step(action);
        auto state = phys.state();
        double readings[4];
        perfect_readings(state.x, state.y, state.heading_deg, cfg, readings);

        mcl.predict(result.delta.forward_in, result.delta.rotation_deg, state.heading_deg);
        mcl.update(readings, state.heading_deg);
        mcl.resample();

        double err = est_error(mcl.estimate(), state.x, state.y);
        if (err > max_err) max_err = err;
    }

    if (out_max_error) *out_max_error = max_err;

    auto state = phys.state();
    return est_error(mcl.estimate(), state.x, state.y);
}

// ============================================================================
// 1. Predict step: particle displacement
//    Tests that predict() actually moves particles, independently of the
//    update/resample cycle.
// ============================================================================

TEST_CASE("MCL predict: forward motion heading 0 shifts particles in +Y") {
    MCLConfig cfg;
    cfg.num_particles = 50;
    MCLEngine mcl(cfg);
    mcl.initialize_uniform(42);

    // Record original Y centroid
    double orig_y = 0.0;
    for (auto& p : mcl.particles()) {
        orig_y += p.y;
    }
    orig_y /= cfg.num_particles;

    double delta_forward = 5.0;
    mcl.predict(delta_forward, 0.0, 0.0);

    double new_y = 0.0;
    for (auto& p : mcl.particles()) {
        new_y += p.y;
    }
    new_y /= cfg.num_particles;

    CHECK(new_y == doctest::Approx(orig_y + delta_forward).epsilon(0.1));
}

TEST_CASE("MCL predict: forward motion heading 90 shifts particles in +X") {
    MCLConfig cfg;
    cfg.num_particles = 50;
    MCLEngine mcl(cfg);
    mcl.initialize_uniform(42);

    double orig_x = 0.0;
    for (auto& p : mcl.particles()) {
        orig_x += p.x;
    }
    orig_x /= cfg.num_particles;

    double delta_forward = 3.0;
    mcl.predict(delta_forward, 0.0, 90.0);

    double new_x = 0.0;
    for (auto& p : mcl.particles()) {
        new_x += p.x;
    }
    new_x /= cfg.num_particles;

    CHECK(new_x == doctest::Approx(orig_x + delta_forward).epsilon(0.1));
}

TEST_CASE("MCL predict: backward motion heading 0 shifts particles in -Y") {
    MCLConfig cfg;
    cfg.num_particles = 50;
    MCLEngine mcl(cfg);
    mcl.initialize_uniform(42);

    double orig_y = 0.0;
    for (auto& p : mcl.particles()) {
        orig_y += p.y;
    }
    orig_y /= cfg.num_particles;

    double delta_forward = -4.0;
    mcl.predict(delta_forward, 0.0, 0.0);

    double new_y = 0.0;
    for (auto& p : mcl.particles()) {
        new_y += p.y;
    }
    new_y /= cfg.num_particles;

    CHECK(new_y == doctest::Approx(orig_y + delta_forward).epsilon(0.1));
}

TEST_CASE("MCL predict: diagonal heading 45 shifts both X and Y") {
    MCLConfig cfg;
    cfg.num_particles = 50;
    MCLEngine mcl(cfg);
    mcl.initialize_uniform(42);

    double orig_x = 0.0, orig_y = 0.0;
    for (auto& p : mcl.particles()) {
        orig_x += p.x;
        orig_y += p.y;
    }
    orig_x /= cfg.num_particles;
    orig_y /= cfg.num_particles;

    double delta_forward = 4.0;
    mcl.predict(delta_forward, 0.0, 45.0);

    double new_x = 0.0, new_y = 0.0;
    for (auto& p : mcl.particles()) {
        new_x += p.x;
        new_y += p.y;
    }
    new_x /= cfg.num_particles;
    new_y /= cfg.num_particles;

    double diag = delta_forward / std::sqrt(2.0);
    CHECK(new_x == doctest::Approx(orig_x + diag).epsilon(0.2));
    CHECK(new_y == doctest::Approx(orig_y + diag).epsilon(0.2));
}

TEST_CASE("MCL predict: zero forward is a no-op") {
    MCLConfig cfg;
    cfg.num_particles = 20;
    MCLEngine mcl(cfg);
    mcl.initialize_uniform(42);

    std::vector<Particle> before(mcl.particles().begin(), mcl.particles().end());
    mcl.predict(0.0, 0.0, 0.0);
    const auto& after = mcl.particles();

    REQUIRE(before.size() == after.size());
    for (size_t i = 0; i < before.size(); i++) {
        CHECK(before[i].x == after[i].x);
        CHECK(before[i].y == after[i].y);
    }
}

TEST_CASE("MCL predict: each particle shifts by the same amount (no noise in Phase 3)") {
    MCLConfig cfg;
    cfg.num_particles = 50;
    cfg.predict_noise_fwd = 0.0;
    cfg.predict_noise_lat = 0.0;
    cfg.predict_noise_heading_deg = 0.0;
    MCLEngine mcl(cfg);
    mcl.initialize_uniform(42);

    std::vector<float> orig_x, orig_y;
    for (auto& p : mcl.particles()) {
        orig_x.push_back(p.x);
        orig_y.push_back(p.y);
    }

    double delta = 7.0;
    double heading = 30.0;
    mcl.predict(delta, 0.0, heading);

    double hrad = deg2rad(heading);
    double expected_dx = delta * std::sin(hrad);
    double expected_dy = delta * std::cos(hrad);

    for (size_t i = 0; i < mcl.particles().size(); i++) {
        double actual_dx = mcl.particles()[i].x - orig_x[i];
        double actual_dy = mcl.particles()[i].y - orig_y[i];
        // Particles near the field boundary may be clamped after predict
        const float fh = static_cast<float>(cfg.field_half);
        bool clamped_x = (orig_x[i] + expected_dx > fh) || (orig_x[i] + expected_dx < -fh);
        bool clamped_y = (orig_y[i] + expected_dy > fh) || (orig_y[i] + expected_dy < -fh);
        if (!clamped_x) {
            CHECK(actual_dx == doctest::Approx(expected_dx).epsilon(1e-3));
        }
        if (!clamped_y) {
            CHECK(actual_dy == doctest::Approx(expected_dy).epsilon(1e-3));
        }
    }
}

TEST_CASE("MCL predict: rotation parameter alone does not move particles") {
    MCLConfig cfg;
    cfg.num_particles = 20;
    MCLEngine mcl(cfg);
    mcl.initialize_uniform(42);

    std::vector<Particle> before(mcl.particles().begin(), mcl.particles().end());
    mcl.predict(0.0, 45.0, 90.0);  // rotation only, no forward
    const auto& after = mcl.particles();

    for (size_t i = 0; i < before.size(); i++) {
        CHECK(before[i].x == after[i].x);
        CHECK(before[i].y == after[i].y);
    }
}

TEST_CASE("MCL predict: multiple sequential predicts accumulate displacement") {
    MCLConfig cfg;
    cfg.num_particles = 20;
    MCLEngine mcl(cfg);
    mcl.initialize_uniform(42);

    double orig_y = 0.0;
    for (auto& p : mcl.particles()) orig_y += p.y;
    orig_y /= cfg.num_particles;

    int steps = 10;
    double per_step = 2.0;
    for (int i = 0; i < steps; i++) {
        mcl.predict(per_step, 0.0, 0.0);
    }

    double new_y = 0.0;
    for (auto& p : mcl.particles()) new_y += p.y;
    new_y /= cfg.num_particles;

    CHECK(new_y == doctest::Approx(orig_y + steps * per_step).epsilon(0.5));
}

// ============================================================================
// 2. MCL tracking: robot moves forward
// ============================================================================

TEST_CASE("MCL tracking: robot moves forward 10 ticks, MCL tracks within 3 inches") {
    std::vector<Action> actions(10, Action::FORWARD);
    double err = run_tracking(0.0, 0.0, 0.0, actions, 20, 42);
    INFO("final error = " << err);
    CHECK(err < 3.0);
}

TEST_CASE("MCL tracking: robot moves forward 30 ticks, MCL tracks within 5 inches") {
    std::vector<Action> actions(30, Action::FORWARD);
    double max_err = 0.0;
    double final_err = run_tracking(0.0, 0.0, 0.0, actions, 20, 42, &max_err);
    INFO("final error = " << final_err << " max error = " << max_err);
    CHECK(final_err < 5.0);
}

TEST_CASE("MCL tracking: robot moves backward 10 ticks") {
    std::vector<Action> actions(10, Action::BACKWARD);
    double err = run_tracking(0.0, 0.0, 0.0, actions, 20, 42);
    INFO("final error = " << err);
    CHECK(err < 3.0);
}

// ============================================================================
// 3. MCL tracking: robot rotates then moves
// ============================================================================

TEST_CASE("MCL tracking: rotate 90 CW then move forward") {
    std::vector<Action> actions;
    // 5 rotation steps × 18°/step = 90°
    for (int i = 0; i < 5; i++) actions.push_back(Action::ROTATE_CW);
    for (int i = 0; i < 10; i++) actions.push_back(Action::FORWARD);

    double err = run_tracking(0.0, 0.0, 0.0, actions, 20, 42);
    INFO("final error = " << err);
    CHECK(err < 5.0);
}

TEST_CASE("MCL tracking: rotate 180 then move forward") {
    std::vector<Action> actions;
    for (int i = 0; i < 10; i++) actions.push_back(Action::ROTATE_CW);
    for (int i = 0; i < 10; i++) actions.push_back(Action::FORWARD);

    double err = run_tracking(0.0, 0.0, 0.0, actions, 20, 42);
    INFO("final error = " << err);
    CHECK(err < 5.0);
}

// ============================================================================
// 4. MCL tracking: square path
// ============================================================================

TEST_CASE("MCL tracking: robot moves in a square, MCL tracks the full path") {
    std::vector<Action> actions;
    int steps_per_side = 5;
    for (int side = 0; side < 4; side++) {
        for (int i = 0; i < steps_per_side; i++) actions.push_back(Action::FORWARD);
        for (int i = 0; i < 5; i++) actions.push_back(Action::ROTATE_CW);  // 90°
    }

    double max_err = 0.0;
    double final_err = run_tracking(0.0, 0.0, 0.0, actions, 20, 42, &max_err);
    INFO("final error = " << final_err << " max error = " << max_err);
    CHECK(final_err < 5.0);
    CHECK(max_err < 8.0);
}

// ============================================================================
// 5. MCL tracking: wall collision
// ============================================================================

TEST_CASE("MCL tracking: robot moves into wall, MCL handles stopped motion") {
    std::vector<Action> actions;
    // Start near the top wall and drive into it
    for (int i = 0; i < 40; i++) actions.push_back(Action::FORWARD);

    double err = run_tracking(0.0, 55.0, 0.0, actions, 20, 42);
    INFO("final error after collision = " << err);
    CHECK(err < 5.0);
}

TEST_CASE("MCL tracking: wall collision then move away, MCL re-tracks") {
    std::vector<Action> actions;
    // Drive into wall
    for (int i = 0; i < 10; i++) actions.push_back(Action::FORWARD);
    // Rotate 180
    for (int i = 0; i < 10; i++) actions.push_back(Action::ROTATE_CW);
    // Drive back
    for (int i = 0; i < 10; i++) actions.push_back(Action::FORWARD);

    double err = run_tracking(0.0, 55.0, 0.0, actions, 20, 42);
    INFO("final error = " << err);
    CHECK(err < 5.0);
}

// ============================================================================
// 6. MCL tracking: various starting positions
// ============================================================================

TEST_CASE("MCL tracking: starting near corner") {
    std::vector<Action> actions;
    for (int i = 0; i < 10; i++) actions.push_back(Action::FORWARD);

    double err = run_tracking(-50.0, -50.0, 45.0, actions, 20, 42);
    INFO("corner start error = " << err);
    CHECK(err < 5.0);
}

TEST_CASE("MCL tracking: starting at center moving diagonally") {
    std::vector<Action> actions;
    for (int i = 0; i < 15; i++) actions.push_back(Action::FORWARD);

    double err = run_tracking(0.0, 0.0, 45.0, actions, 20, 42);
    INFO("diagonal error = " << err);
    CHECK(err < 5.0);
}

// ============================================================================
// 7. MCL tracking: interleaved forward and rotation (curved path)
// ============================================================================

TEST_CASE("MCL tracking: gentle curve (forward + small rotation alternating)") {
    std::vector<Action> actions;
    for (int i = 0; i < 15; i++) {
        actions.push_back(Action::FORWARD);
        actions.push_back(Action::ROTATE_CW);
    }

    double err = run_tracking(0.0, 0.0, 0.0, actions, 20, 42);
    INFO("curve error = " << err);
    CHECK(err < 5.0);
}

TEST_CASE("MCL tracking: zigzag path") {
    std::vector<Action> actions;
    for (int leg = 0; leg < 4; leg++) {
        for (int i = 0; i < 5; i++) actions.push_back(Action::FORWARD);
        // Alternate CW and CCW
        Action turn = (leg % 2 == 0) ? Action::ROTATE_CW : Action::ROTATE_CCW;
        for (int i = 0; i < 5; i++) actions.push_back(turn);
    }
    for (int i = 0; i < 5; i++) actions.push_back(Action::FORWARD);

    double max_err = 0.0;
    double final_err = run_tracking(0.0, 0.0, 0.0, actions, 20, 42, &max_err);
    INFO("zigzag final error = " << final_err << " max error = " << max_err);
    CHECK(final_err < 5.0);
}

// ============================================================================
// 8. MCL tracking: long straight run should not drift
// ============================================================================

TEST_CASE("MCL tracking: 50-tick straight run stays locked on") {
    std::vector<Action> actions(50, Action::FORWARD);
    double max_err = 0.0;
    double final_err = run_tracking(0.0, -40.0, 0.0, actions, 20, 42, &max_err);
    INFO("long run final error = " << final_err << " max error = " << max_err);
    CHECK(final_err < 5.0);
    CHECK(max_err < 8.0);
}

// ============================================================================
// 9. MCL tracking: pure rotation (no translation)
// ============================================================================

TEST_CASE("MCL tracking: 20 CW rotation steps, MCL stays locked") {
    std::vector<Action> actions(20, Action::ROTATE_CW);
    double err = run_tracking(20.0, 30.0, 0.0, actions, 20, 42);
    INFO("rotation-only error = " << err);
    CHECK(err < 3.0);
}

// ============================================================================
// 10. MCL tracking: heading convention consistency
//     The predict step must use the same heading convention as the sensor model.
//     Moving forward at heading H should bring the estimate closer to truth
//     at the correct position, not a mirror/rotation artifact.
// ============================================================================

TEST_CASE("MCL tracking: heading consistency across cardinal directions") {
    PhysicsConfig pcfg;
    double step_dist = pcfg.max_velocity * pcfg.dt;

    double headings[] = { 0.0, 90.0, 180.0, 270.0 };
    // Expected displacement per heading (dx, dy)
    double expected_dx[] = { 0.0, step_dist, 0.0, -step_dist };
    double expected_dy[] = { step_dist, 0.0, -step_dist, 0.0 };

    for (int h = 0; h < 4; h++) {
        MCLConfig cfg;
        cfg.num_particles = 100;
        MCLEngine mcl(cfg);
        mcl.initialize_uniform(42);

        // Place all particles at origin
        auto& ps = const_cast<std::vector<Particle>&>(mcl.particles());
        float w = 1.0f / static_cast<float>(cfg.num_particles);
        for (auto& p : ps) {
            p.x = 0.0f;
            p.y = 0.0f;
            p.weight = w;
        }

        mcl.predict(step_dist, 0.0, headings[h]);

        double mean_x = 0.0, mean_y = 0.0;
        for (auto& p : mcl.particles()) {
            mean_x += p.x;
            mean_y += p.y;
        }
        mean_x /= cfg.num_particles;
        mean_y /= cfg.num_particles;

        INFO("heading=" << headings[h]);
        CHECK(mean_x == doctest::Approx(expected_dx[h]).epsilon(0.01));
        CHECK(mean_y == doctest::Approx(expected_dy[h]).epsilon(0.01));
    }
}

// ============================================================================
// 11. MCL tracking: predict preserves weight
//     The predict step should not alter particle weights.
// ============================================================================

TEST_CASE("MCL predict: weights are unchanged after predict") {
    MCLConfig cfg;
    cfg.num_particles = 50;
    MCLEngine mcl(cfg);
    mcl.initialize_uniform(42);

    std::vector<float> weights_before;
    for (auto& p : mcl.particles()) {
        weights_before.push_back(p.weight);
    }

    mcl.predict(5.0, 0.0, 45.0);

    for (size_t i = 0; i < mcl.particles().size(); i++) {
        CHECK(mcl.particles()[i].weight == weights_before[i]);
    }
}

// ============================================================================
// 12. MCL tracking: predict + update + resample cycle convergence
//     After movement, the MCL estimate must track the new truth position.
//     This is the core Phase 3 integration test.
// ============================================================================

TEST_CASE("MCL tracking: converged MCL re-converges after displacement") {
    MCLConfig cfg;
    cfg.num_particles = 300;
    MCLEngine mcl(cfg);
    mcl.initialize_uniform(42);

    double truth_x = 20.0, truth_y = 30.0, truth_heading = 0.0;
    double readings[4];

    // First converge stationarily
    for (int t = 0; t < 20; t++) {
        perfect_readings(truth_x, truth_y, truth_heading, cfg, readings);
        mcl.predict(0.0, 0.0, truth_heading);
        mcl.update(readings, truth_heading);
        mcl.resample();
    }

    double err_before = est_error(mcl.estimate(), truth_x, truth_y);
    CHECK(err_before < 3.0);

    // Now move forward 5 inches
    double delta_forward = 5.0;
    double hrad = deg2rad(truth_heading);
    truth_x += delta_forward * std::sin(hrad);
    truth_y += delta_forward * std::cos(hrad);

    perfect_readings(truth_x, truth_y, truth_heading, cfg, readings);
    mcl.predict(delta_forward, 0.0, truth_heading);
    mcl.update(readings, truth_heading);
    mcl.resample();

    // After a few more ticks, should re-converge to new position
    for (int t = 0; t < 5; t++) {
        perfect_readings(truth_x, truth_y, truth_heading, cfg, readings);
        mcl.predict(0.0, 0.0, truth_heading);
        mcl.update(readings, truth_heading);
        mcl.resample();
    }

    double err_after = est_error(mcl.estimate(), truth_x, truth_y);
    INFO("error after movement + re-convergence = " << err_after);
    CHECK(err_after < 3.0);
}

// ============================================================================
// 13. MCL tracking: multi-seed robustness for movement
// ============================================================================

TEST_CASE("MCL tracking movement: robust across seeds") {
    std::vector<Action> actions;
    for (int i = 0; i < 10; i++) actions.push_back(Action::FORWARD);

    uint64_t seeds[] = { 1, 7, 42, 99, 256, 512, 1337, 5000, 9999, 31415 };
    int pass_count = 0;
    for (uint64_t seed : seeds) {
        double err = run_tracking(0.0, 0.0, 0.0, actions, 20, seed);
        INFO("seed=" << seed << " err=" << err);
        if (err < 5.0) pass_count++;
    }
    CHECK(pass_count >= 8);
}

// ============================================================================
// 14. MCL tracking: parametric multi-position movement test
//     Sweep starting positions and directions, require high pass rate.
// ============================================================================

TEST_CASE("MCL tracking: parametric sweep of starting positions and headings") {
    int pass_count = 0;
    int total = 0;

    double positions[][2] = {
        {0.0, 0.0}, {-40.0, -40.0}, {40.0, 40.0}, {-40.0, 40.0}, {40.0, -40.0},
        {0.0, 40.0}, {0.0, -40.0}, {40.0, 0.0}, {-40.0, 0.0}
    };
    double headings[] = { 0.0, 45.0, 90.0, 135.0, 180.0, 225.0, 270.0, 315.0 };

    for (auto& pos : positions) {
        for (double heading : headings) {
            total++;
            std::vector<Action> actions;
            for (int i = 0; i < 8; i++) actions.push_back(Action::FORWARD);

            // Try up to 3 seeds for each scenario
            bool passed = false;
            for (uint64_t seed : { 1ULL, 42ULL, 99ULL }) {
                double err = run_tracking(pos[0], pos[1], heading, actions, 20, seed);
                if (err < 5.0) {
                    passed = true;
                    break;
                }
            }
            if (passed) {
                pass_count++;
            } else {
                INFO("FAILED at (" << pos[0] << "," << pos[1] << ") heading=" << heading);
            }
        }
    }

    INFO("parametric movement pass_count=" << pass_count << " / " << total);
    CHECK(pass_count >= total * 90 / 100);
}

// ============================================================================
// 15. MCL tracking: random movement scenarios
// ============================================================================

TEST_CASE("MCL tracking: 100 random start positions with forward movement") {
    std::mt19937 meta_rng(20260317);
    std::uniform_real_distribution<double> pos_dist(-50.0, 50.0);
    std::uniform_real_distribution<double> heading_dist(0.0, 360.0);

    int pass_count = 0;
    const int N = 100;

    for (int i = 0; i < N; i++) {
        double x = pos_dist(meta_rng);
        double y = pos_dist(meta_rng);
        double heading = heading_dist(meta_rng);

        std::vector<Action> actions;
        for (int j = 0; j < 8; j++) actions.push_back(Action::FORWARD);

        bool ok = false;
        for (uint64_t seed : { 1ULL, 42ULL, 99ULL }) {
            double err = run_tracking(x, y, heading, actions, 20, seed);
            if (err < 5.0) { ok = true; break; }
        }
        if (ok) pass_count++;
        else {
            INFO("FAILED scenario " << i << " at (" << x << "," << y
                 << ") heading=" << heading);
        }
    }

    INFO("random movement pass_count=" << pass_count << " / " << N);
    CHECK(pass_count >= N * 85 / 100);
}

// ============================================================================
// 16. MCL tracking: numerical stability during extended movement
// ============================================================================

TEST_CASE("MCL tracking: 200-tick movement produces finite estimates and valid weights") {
    MCLConfig cfg;
    cfg.num_particles = 300;
    MCLEngine mcl(cfg);
    mcl.initialize_uniform(42);

    Field field;
    PhysicsConfig pcfg;
    Physics phys(field, pcfg);
    phys.set_state({ 0.0, 0.0, 30.0 });

    // Alternate forward and rotation to stay in bounds
    for (int tick = 0; tick < 200; tick++) {
        Action action;
        if (tick % 20 < 15) {
            action = Action::FORWARD;
        } else {
            action = Action::ROTATE_CW;
        }

        auto result = phys.step(action);
        auto state = phys.state();
        double readings[4];
        perfect_readings(state.x, state.y, state.heading_deg, cfg, readings);

        mcl.predict(result.delta.forward_in, result.delta.rotation_deg, state.heading_deg);
        mcl.update(readings, state.heading_deg);
        mcl.resample();

        auto est = mcl.estimate();
        CHECK(std::isfinite(est.x));
        CHECK(std::isfinite(est.y));

        double wsum = 0.0;
        for (auto& p : mcl.particles()) {
            CHECK(std::isfinite(p.weight));
            CHECK(p.weight >= 0.0f);
            wsum += p.weight;
        }
        CHECK(wsum == doctest::Approx(1.0).epsilon(1e-4));
    }
}

// ============================================================================
// 17. MCL tracking: deterministic repeatability with movement
// ============================================================================

TEST_CASE("MCL tracking: same seed + same actions = identical results") {
    auto run = [](uint64_t seed) -> std::pair<float, float> {
        MCLConfig cfg;
        cfg.num_particles = 100;
        MCLEngine mcl(cfg);
        mcl.initialize_uniform(seed);

        Field field;
        Physics phys(field);
        phys.set_state({ 10.0, 10.0, 0.0 });

        for (int t = 0; t < 10; t++) {
            auto state = phys.state();
            double readings[4];
            perfect_readings(state.x, state.y, state.heading_deg, cfg, readings);
            mcl.predict(0.0, 0.0, state.heading_deg);
            mcl.update(readings, state.heading_deg);
            mcl.resample();
        }

        auto result = phys.step(Action::FORWARD);
        auto state = phys.state();
        double readings[4];
        perfect_readings(state.x, state.y, state.heading_deg, cfg, readings);
        mcl.predict(result.delta.forward_in, result.delta.rotation_deg, state.heading_deg);
        mcl.update(readings, state.heading_deg);
        mcl.resample();

        auto est = mcl.estimate();
        return { est.x, est.y };
    };

    auto [x1, y1] = run(42);
    auto [x2, y2] = run(42);
    CHECK(x1 == x2);
    CHECK(y1 == y2);

    auto [x3, y3] = run(99);
    CHECK((x1 != x3 || y1 != y3));
}

// ============================================================================
// 18. MCL tracking: error does not grow monotonically during movement
//     After initial convergence, moving should not cause unbounded error growth.
// ============================================================================

TEST_CASE("MCL tracking: error stays bounded during 40-tick straight movement") {
    MCLConfig cfg;
    cfg.num_particles = 300;
    MCLEngine mcl(cfg);
    mcl.initialize_uniform(42);

    Field field;
    PhysicsConfig pcfg;
    Physics phys(field, pcfg);
    phys.set_state({ 0.0, -30.0, 0.0 });

    // Converge first
    for (int t = 0; t < 20; t++) {
        auto state = phys.state();
        double readings[4];
        perfect_readings(state.x, state.y, state.heading_deg, cfg, readings);
        mcl.predict(0.0, 0.0, state.heading_deg);
        mcl.update(readings, state.heading_deg);
        mcl.resample();
    }

    // Move forward 40 ticks and track errors
    std::vector<double> errors;
    for (int t = 0; t < 40; t++) {
        auto result = phys.step(Action::FORWARD);
        auto state = phys.state();
        double readings[4];
        perfect_readings(state.x, state.y, state.heading_deg, cfg, readings);
        mcl.predict(result.delta.forward_in, result.delta.rotation_deg, state.heading_deg);
        mcl.update(readings, state.heading_deg);
        mcl.resample();

        errors.push_back(est_error(mcl.estimate(), state.x, state.y));
    }

    // Every error should be bounded
    for (size_t i = 0; i < errors.size(); i++) {
        INFO("tick=" << i << " error=" << errors[i]);
        CHECK(errors[i] < 8.0);
    }

    // Average of last 10 errors should be low (filter is tracking well)
    double avg_last10 = 0.0;
    for (size_t i = errors.size() - 10; i < errors.size(); i++) {
        avg_last10 += errors[i];
    }
    avg_last10 /= 10.0;
    INFO("avg error last 10 ticks = " << avg_last10);
    CHECK(avg_last10 < 5.0);
}
