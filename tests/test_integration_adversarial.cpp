// =============================================================================
// test_integration_adversarial.cpp
//
// Full-system adversarial integration tests.
// These tests combine MCL + Physics + SensorModel + Obstacles into
// worst-case production scenarios that individually tested components
// might not reveal.
// =============================================================================

#include "doctest/doctest.h"
#include "mcl/mcl_engine.hpp"
#include "distance_localization.hpp"
#include "sim/field.hpp"
#include "sim/physics.hpp"
#include "sim/sensor_model.hpp"
#include <cmath>
#include <random>
#include <vector>

using namespace mcl;
using namespace sim;
using namespace distance_loc;

// ============================================================================
// Helpers
// ============================================================================

static void observe_readings(SensorModel& sm, const MCLConfig& cfg,
                             const Field& field,
                             double x, double y, double heading,
                             double out[4]) {
    Vec2 pos{x, y};
    for (int i = 0; i < 4; i++) {
        out[i] = sm.observe_distance_sensor(
            pos, heading, cfg.sensors[i].offset,
            cfg.sensors[i].angle_deg, i, field.obstacles);
    }
}

static double est_error(const Estimate& est, double tx, double ty) {
    double dx = est.x - tx;
    double dy = est.y - ty;
    return std::sqrt(dx * dx + dy * dy);
}

// Full pipeline runner with all noise and obstacles
static double run_full_pipeline(
    uint64_t seed,
    const std::vector<Action>& actions,
    const Field& field,
    const OdomNoiseConfig& odom_cfg,
    const SensorNoiseConfig& sensor_cfg,
    double start_x, double start_y, double start_heading,
    double* max_err_out = nullptr
) {
    MCLConfig mcl_cfg;
    mcl_cfg.num_particles = 300;
    mcl_cfg.predict_noise_fwd = 0.03;
    mcl_cfg.predict_noise_lat = 0.015;

    Physics phys(field);
    phys.set_state({start_x, start_y, start_heading});

    SensorModel sm(seed + 500, odom_cfg, sensor_cfg);
    MCLEngine mcl(mcl_cfg);
    mcl.initialize_uniform(seed);

    // Warmup
    for (int t = 0; t < 25; t++) {
        auto truth = phys.state();
        double readings[4];
        observe_readings(sm, mcl_cfg, field, truth.x, truth.y, truth.heading_deg, readings);
        double obs_heading = sm.observe_heading(truth.heading_deg);
        mcl.predict(0.0, 0.0, obs_heading, 0.0);
        mcl.update(readings, obs_heading);
        mcl.resample();
    }

    double max_err = 0.0;
    for (Action action : actions) {
        auto step = phys.step(action);
        auto truth = phys.state();

        MotionDelta od = sm.apply_odom_noise(step.delta, truth.heading_deg);
        od = sm.apply_collision_stall(od, step.colliding);
        double obs_heading = sm.observe_heading(truth.heading_deg);

        double readings[4];
        observe_readings(sm, mcl_cfg, field, truth.x, truth.y, truth.heading_deg, readings);
        mcl.predict(od.forward_in, od.rotation_deg, obs_heading, od.lateral_in);
        mcl.update(readings, obs_heading);
        mcl.resample();

        double err = est_error(mcl.estimate(), truth.x, truth.y);
        if (err > max_err) max_err = err;
    }

    if (max_err_out) *max_err_out = max_err;
    auto truth = phys.state();
    return est_error(mcl.estimate(), truth.x, truth.y);
}

// ============================================================================
// 1. WORST-CASE NOISE: ALL NOISE SOURCES SIMULTANEOUSLY
// ============================================================================

TEST_CASE("Integration adversarial: high odom + sensor noise straight-line") {
    Field field;
    OdomNoiseConfig odom;
    odom.trans_noise_frac = 0.08;
    odom.drift_noise_frac = 0.04;
    odom.drift_per_tick_in = 0.05;

    SensorNoiseConfig sensor;
    sensor.gaussian_stddev_mm = 20.0;
    sensor.dropout_probability = 0.05;
    sensor.long_dropout_probability = 0.01;
    sensor.spurious_reflection_probability = 0.03;
    sensor.imu_noise_stddev_deg = 0.5;

    std::vector<Action> actions(30, Action::FORWARD);
    int pass = 0;
    for (uint64_t seed : {1ULL, 7ULL, 42ULL, 99ULL, 256ULL, 512ULL, 1337ULL, 5000ULL}) {
        double err = run_full_pipeline(seed, actions, field, odom, sensor, 0, 0, 0);
        if (err < 15.0) pass++;
    }
    CHECK(pass >= 5);
}

// ============================================================================
// 2. OBSTACLE MAZE
//    Multiple obstacles creating a complex environment.
// ============================================================================

TEST_CASE("Integration adversarial: obstacle maze tracking") {
    Field field;
    // Create a maze-like environment
    field.obstacles.push_back({-30.0, 15.0, -15.0, 25.0});
    field.obstacles.push_back({15.0, 15.0, 30.0, 25.0});
    field.obstacles.push_back({-10.0, -25.0, 10.0, -15.0});
    field.obstacles.push_back({35.0, -10.0, 50.0, 10.0});
    field.obstacles.push_back({-50.0, -10.0, -35.0, 10.0});

    OdomNoiseConfig odom;
    odom.trans_noise_frac = 0.03;
    SensorNoiseConfig sensor;

    // Navigate through the maze
    std::vector<Action> actions;
    for (int i = 0; i < 8; i++) actions.push_back(Action::FORWARD);
    for (int i = 0; i < 5; i++) actions.push_back(Action::ROTATE_CW);
    for (int i = 0; i < 8; i++) actions.push_back(Action::FORWARD);
    for (int i = 0; i < 5; i++) actions.push_back(Action::ROTATE_CCW);
    for (int i = 0; i < 8; i++) actions.push_back(Action::FORWARD);

    int pass = 0;
    for (uint64_t seed : {42ULL, 99ULL, 256ULL, 512ULL, 1337ULL}) {
        double err = run_full_pipeline(seed, actions, field, odom, sensor, 0, -40, 0);
        if (err < 20.0) pass++;
    }
    CHECK(pass >= 3);
}

// ============================================================================
// 3. WALL HUGGING: ROBOT DRIVES ALONG A WALL
//    Tests that MCL doesn't lose track when one sensor reads near-zero
//    consistently.
// ============================================================================

TEST_CASE("Integration adversarial: wall-hugging path along top wall") {
    Field field;
    OdomNoiseConfig odom;
    SensorNoiseConfig sensor;

    // Start near top wall, heading 90° (east), drive along the wall
    std::vector<Action> actions(25, Action::FORWARD);

    int pass = 0;
    for (uint64_t seed : {42ULL, 99ULL, 256ULL, 512ULL}) {
        double err = run_full_pipeline(seed, actions, field, odom, sensor,
                                       -30, 58, 90);
        if (err < 12.0) pass++;
    }
    CHECK(pass >= 2);
}

// ============================================================================
// 4. CORNER APPROACH AND ESCAPE
//    Drive into a corner, get stuck, rotate out, and drive away.
// ============================================================================

TEST_CASE("Integration adversarial: corner approach, stuck, escape") {
    Field field;
    OdomNoiseConfig odom;
    odom.trans_noise_frac = 0.04;
    SensorNoiseConfig sensor;
    sensor.collision_stall_ticks = 3;

    std::vector<Action> actions;
    // Drive toward top-right corner at 45°
    for (int i = 0; i < 30; i++) actions.push_back(Action::FORWARD);
    // Stuck — rotate 180°
    for (int i = 0; i < 10; i++) actions.push_back(Action::ROTATE_CW);
    // Drive back to safety
    for (int i = 0; i < 20; i++) actions.push_back(Action::FORWARD);

    double err = run_full_pipeline(42, actions, field, odom, sensor, 30, 30, 45);
    INFO("corner escape error = " << err);
    CHECK(std::isfinite(err));
    // Generous tolerance — this is a hard scenario
    CHECK(err < 25.0);
}

// ============================================================================
// 5. RAPID DIRECTION CHANGES (ZIGZAG STRESS TEST)
// ============================================================================

TEST_CASE("Integration adversarial: extreme zigzag — direction change every 2 ticks") {
    Field field;
    OdomNoiseConfig odom;
    SensorNoiseConfig sensor;

    std::vector<Action> actions;
    for (int i = 0; i < 30; i++) {
        actions.push_back(Action::FORWARD);
        actions.push_back(Action::FORWARD);
        if (i % 2 == 0) {
            for (int j = 0; j < 5; j++) actions.push_back(Action::ROTATE_CW);
        } else {
            for (int j = 0; j < 5; j++) actions.push_back(Action::ROTATE_CCW);
        }
    }

    int pass = 0;
    for (uint64_t seed : {1ULL, 42ULL, 99ULL, 256ULL, 512ULL}) {
        double max_err = 0;
        double final_err = run_full_pipeline(seed, actions, field, odom, sensor,
                                              0, -40, 0, &max_err);
        if (final_err < 15.0) pass++;
    }
    CHECK(pass >= 3);
}

// ============================================================================
// 6. SPINNING IN PLACE
//    Robot rotates continuously. MCL must not drift position estimate.
// ============================================================================

TEST_CASE("Integration adversarial: continuous rotation for 100 ticks") {
    Field field;
    OdomNoiseConfig odom;
    SensorNoiseConfig sensor;

    std::vector<Action> actions(100, Action::ROTATE_CW);

    double max_err = 0;
    double final_err = run_full_pipeline(42, actions, field, odom, sensor,
                                          0, 0, 0, &max_err);
    INFO("spin-in-place final_err=" << final_err << " max_err=" << max_err);
    CHECK(final_err < 8.0);
}

// ============================================================================
// 7. BACKWARD-ONLY MOVEMENT
// ============================================================================

TEST_CASE("Integration adversarial: long backward movement") {
    Field field;
    OdomNoiseConfig odom;
    SensorNoiseConfig sensor;

    std::vector<Action> actions(25, Action::BACKWARD);

    int pass = 0;
    for (uint64_t seed : {42ULL, 99ULL, 256ULL}) {
        double err = run_full_pipeline(seed, actions, field, odom, sensor,
                                       0, 30, 0);
        if (err < 12.0) pass++;
    }
    CHECK(pass >= 2);
}

// ============================================================================
// 8. COMBINED: OBSTACLE + COLLISION STALL + HIGH NOISE
//    The worst-case production scenario.
// ============================================================================

TEST_CASE("Integration adversarial: obstacle + stall + noise combined") {
    Field field;
    field.obstacles.push_back({-8.0, 20.0, 8.0, 35.0});

    OdomNoiseConfig odom;
    odom.trans_noise_frac = 0.06;
    odom.drift_noise_frac = 0.03;
    odom.drift_per_tick_in = 0.03;

    SensorNoiseConfig sensor;
    sensor.gaussian_stddev_mm = 15.0;
    sensor.dropout_probability = 0.04;
    sensor.spurious_reflection_probability = 0.02;
    sensor.collision_stall_ticks = 4;
    sensor.imu_noise_stddev_deg = 0.3;

    // Drive toward obstacle, collide, rotate, navigate around
    std::vector<Action> actions;
    for (int i = 0; i < 15; i++) actions.push_back(Action::FORWARD);
    for (int i = 0; i < 5; i++) actions.push_back(Action::ROTATE_CW);
    for (int i = 0; i < 10; i++) actions.push_back(Action::FORWARD);
    for (int i = 0; i < 5; i++) actions.push_back(Action::ROTATE_CCW);
    for (int i = 0; i < 10; i++) actions.push_back(Action::FORWARD);

    int pass = 0;
    for (uint64_t seed : {1ULL, 42ULL, 99ULL, 256ULL, 512ULL, 1337ULL}) {
        double err = run_full_pipeline(seed, actions, field, odom, sensor,
                                       0, -10, 0);
        if (err < 25.0) pass++;
    }
    CHECK(pass >= 3);
}

// ============================================================================
// 9. NUMERICAL STABILITY: 500-TICK FULL-PIPELINE RUN
//    Every tick must produce finite values. No NaN, no infinity, no weight collapse.
// ============================================================================

TEST_CASE("Integration adversarial: 500-tick full pipeline produces finite values throughout") {
    MCLConfig mcl_cfg;
    mcl_cfg.num_particles = 300;
    mcl_cfg.predict_noise_fwd = 0.03;
    mcl_cfg.predict_noise_lat = 0.015;

    Field field;
    field.obstacles.push_back({20.0, 20.0, 35.0, 35.0});

    OdomNoiseConfig odom;
    odom.trans_noise_frac = 0.04;
    odom.drift_noise_frac = 0.02;
    odom.drift_per_tick_in = 0.01;

    SensorNoiseConfig sensor;
    sensor.gaussian_stddev_mm = 10.0;
    sensor.dropout_probability = 0.03;
    sensor.spurious_reflection_probability = 0.01;
    sensor.imu_noise_stddev_deg = 0.2;
    sensor.collision_stall_ticks = 3;

    Physics phys(field);
    phys.set_state({0.0, -30.0, 0.0});
    SensorModel sm(42, odom, sensor);
    MCLEngine mcl(mcl_cfg);
    mcl.initialize_uniform(42);

    // Warmup
    for (int t = 0; t < 20; t++) {
        auto truth = phys.state();
        double readings[4];
        observe_readings(sm, mcl_cfg, field, truth.x, truth.y, truth.heading_deg, readings);
        double h = sm.observe_heading(truth.heading_deg);
        mcl.predict(0.0, 0.0, h, 0.0);
        mcl.update(readings, h);
        mcl.resample();
    }

    // 500 ticks of varied movement
    for (int tick = 0; tick < 500; tick++) {
        Action action;
        if (tick % 25 < 15) action = Action::FORWARD;
        else if (tick % 25 < 20) action = Action::ROTATE_CW;
        else action = Action::BACKWARD;

        auto step = phys.step(action);
        auto truth = phys.state();

        MotionDelta od = sm.apply_odom_noise(step.delta, truth.heading_deg);
        od = sm.apply_collision_stall(od, step.colliding);
        double h = sm.observe_heading(truth.heading_deg);

        double readings[4];
        observe_readings(sm, mcl_cfg, field, truth.x, truth.y, truth.heading_deg, readings);
        mcl.predict(od.forward_in, od.rotation_deg, h, od.lateral_in);
        mcl.update(readings, h);
        mcl.resample();

        auto est = mcl.estimate();
        INFO("tick=" << tick);
        CHECK(std::isfinite(est.x));
        CHECK(std::isfinite(est.y));

        float wsum = 0.0f;
        for (const auto& p : mcl.particles()) {
            CHECK(std::isfinite(p.weight));
            CHECK(p.weight >= 0.0f);
            wsum += p.weight;
        }
        CHECK(wsum == doctest::Approx(1.0f).epsilon(1e-3));
    }
}

// ============================================================================
// 10. PARAMETRIC: RANDOM STARTING POSITIONS WITH ALL NOISE AND OBSTACLES
// ============================================================================

TEST_CASE("Integration adversarial: 50 random scenarios with noise + obstacles") {
    Field field;
    field.obstacles.push_back({20.0, 20.0, 35.0, 35.0});
    field.obstacles.push_back({-35.0, -35.0, -20.0, -20.0});

    OdomNoiseConfig odom;
    odom.trans_noise_frac = 0.04;
    odom.drift_noise_frac = 0.02;

    SensorNoiseConfig sensor;
    sensor.gaussian_stddev_mm = 10.0;
    sensor.dropout_probability = 0.03;

    std::mt19937 rng(20260318);
    std::uniform_real_distribution<double> pos_dist(-50.0, 50.0);
    std::uniform_real_distribution<double> heading_dist(0.0, 360.0);
    std::uniform_int_distribution<int> action_dist(0, 3);

    const int N = 50;
    int pass = 0;

    for (int i = 0; i < N; i++) {
        double sx = pos_dist(rng);
        double sy = pos_dist(rng);
        double sh = heading_dist(rng);

        // Check if starting inside an obstacle — skip if so
        bool in_obstacle = false;
        for (const auto& obs : field.obstacles) {
            if (sx >= obs.min_x && sx <= obs.max_x &&
                sy >= obs.min_y && sy <= obs.max_y) {
                in_obstacle = true;
                break;
            }
        }
        if (in_obstacle) { pass++; continue; }

        // Random action sequence
        std::vector<Action> actions;
        for (int j = 0; j < 15; j++) {
            switch (action_dist(rng)) {
                case 0: actions.push_back(Action::FORWARD); break;
                case 1: actions.push_back(Action::BACKWARD); break;
                case 2: actions.push_back(Action::ROTATE_CW); break;
                default: actions.push_back(Action::ROTATE_CCW); break;
            }
        }

        bool scenario_pass = false;
        for (uint64_t seed : {42ULL, 99ULL, 256ULL}) {
            double err = run_full_pipeline(seed, actions, field, odom, sensor,
                                           sx, sy, sh);
            if (err < 20.0) { scenario_pass = true; break; }
        }
        if (scenario_pass) pass++;
    }

    INFO("random integration pass=" << pass << " / " << N);
    CHECK(pass >= N * 75 / 100);
}

// ============================================================================
// 11. PARTICLE DEPLETION RECOVERY
//    Force a scenario where particles cluster badly, then verify the system
//    doesn't produce NaN for the rest of the run.
// ============================================================================

TEST_CASE("Integration adversarial: forced particle cluster then continued tracking") {
    MCLConfig cfg;
    cfg.num_particles = 300;
    MCLEngine mcl(cfg);
    mcl.initialize_uniform(42);

    // Force all particles to a single point
    auto& ps = const_cast<std::vector<Particle>&>(mcl.particles());
    float w = 1.0f / cfg.num_particles;
    for (auto& p : ps) {
        p.x = 20.0f;
        p.y = 30.0f;
        p.weight = w;
    }

    Field field;
    Physics phys(field);
    phys.set_state({20.0, 30.0, 0.0});

    // Now run 50 ticks of movement — particles all start at same spot
    for (int t = 0; t < 50; t++) {
        auto result = phys.step(Action::FORWARD);
        auto state = phys.state();
        double readings[4];
        Vec2 pos{state.x, state.y};
        for (int i = 0; i < 4; i++) {
            double d = ray_distance_with_offset(pos, state.heading_deg,
                                                cfg.sensors[i].offset,
                                                cfg.sensors[i].angle_deg);
            readings[i] = std::isfinite(d) ? d : -1.0;
        }

        mcl.predict(result.delta.forward_in, result.delta.rotation_deg, state.heading_deg);
        mcl.update(readings, state.heading_deg);
        mcl.resample();

        auto est = mcl.estimate();
        CHECK(std::isfinite(est.x));
        CHECK(std::isfinite(est.y));
    }

    auto state = phys.state();
    double err = est_error(mcl.estimate(), state.x, state.y);
    INFO("forced cluster tracking error = " << err);
    // May be poor accuracy since all particles started at one point, but must be finite
    CHECK(std::isfinite(err));
}
