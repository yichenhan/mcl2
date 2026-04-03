// =============================================================================
// test_mcl_recovery.cpp
//
// Tests for MCL recovery from catastrophic failures:
// - Kidnapped robot (sudden teleportation)
// - Sensor blackout mid-run
// - Sensor returning wrong data systematically
// - Convergence to wrong position and recovery
// - Transient sensor spikes
// =============================================================================

#include "doctest/doctest.h"
#include "mcl/mcl_engine.hpp"
#include "distance_localization.hpp"
#include "sim/field.hpp"
#include "sim/physics.hpp"
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

// ============================================================================
// 1. KIDNAPPED ROBOT
//    Robot converges at position A, then is instantly moved to position B.
//    MCL must either re-converge or at least not crash / produce NaN.
//    Note: Standard MCL without recovery heuristics (e.g., random injection)
//    may NOT recover. These tests document the expected behavior.
// ============================================================================

TEST_CASE("MCL recovery: kidnapped robot — estimate stays finite after teleport") {
    MCLConfig cfg;
    cfg.num_particles = 300;
    MCLEngine mcl(cfg);
    mcl.initialize_uniform(42);

    // Converge at position A
    double ax = 20.0, ay = 30.0, heading = 0.0;
    double readings[4];
    perfect_readings(ax, ay, heading, cfg, readings);

    for (int t = 0; t < 25; t++) {
        mcl.predict(0.0, 0.0, heading);
        mcl.update(readings, heading);
        mcl.resample();
    }

    CHECK(est_error(mcl.estimate(), ax, ay) < 3.0);

    // Teleport to position B — sensor readings suddenly change
    double bx = -40.0, by = -40.0;
    perfect_readings(bx, by, heading, cfg, readings);

    // Run 50 ticks at new position
    for (int t = 0; t < 50; t++) {
        mcl.predict(0.0, 0.0, heading);
        mcl.update(readings, heading);
        mcl.resample();
    }

    auto est = mcl.estimate();
    CHECK(std::isfinite(est.x));
    CHECK(std::isfinite(est.y));

    // Weight sum must still be valid
    float sum = 0.0f;
    for (const auto& p : mcl.particles()) {
        CHECK(std::isfinite(p.weight));
        sum += p.weight;
    }
    CHECK(sum == doctest::Approx(1.0f).epsilon(1e-3));
}

TEST_CASE("MCL recovery: re-init after kidnap allows convergence to new position") {
    MCLConfig cfg;
    cfg.num_particles = 300;
    MCLEngine mcl(cfg);
    mcl.initialize_uniform(42);

    // Converge at A
    double ax = 20.0, ay = 30.0, heading = 0.0;
    double readings[4];
    perfect_readings(ax, ay, heading, cfg, readings);
    for (int t = 0; t < 25; t++) {
        mcl.predict(0.0, 0.0, heading);
        mcl.update(readings, heading);
        mcl.resample();
    }

    // Kidnap: re-initialize and converge at B
    double bx = -40.0, by = -40.0;
    mcl.initialize_uniform(99);
    perfect_readings(bx, by, heading, cfg, readings);

    for (int t = 0; t < 25; t++) {
        mcl.predict(0.0, 0.0, heading);
        mcl.update(readings, heading);
        mcl.resample();
    }

    double err = est_error(mcl.estimate(), bx, by);
    INFO("re-init recovery error = " << err);
    CHECK(err < 5.0);
}

// ============================================================================
// 2. SENSOR BLACKOUT
//    All sensors go dark (return -1) for a period, then come back.
// ============================================================================

TEST_CASE("MCL recovery: sensor blackout for 20 ticks, then recovery") {
    MCLConfig cfg;
    cfg.num_particles = 300;
    MCLEngine mcl(cfg);
    mcl.initialize_uniform(42);

    double tx = 10.0, ty = 20.0, heading = 30.0;
    double readings[4];
    perfect_readings(tx, ty, heading, cfg, readings);

    // Converge
    for (int t = 0; t < 25; t++) {
        mcl.predict(0.0, 0.0, heading);
        mcl.update(readings, heading);
        mcl.resample();
    }

    double err_before = est_error(mcl.estimate(), tx, ty);
    CHECK(err_before < 3.0);

    // Blackout: all sensors return -1
    double blind[4] = { -1.0, -1.0, -1.0, -1.0 };
    for (int t = 0; t < 20; t++) {
        mcl.predict(0.0, 0.0, heading);
        mcl.update(blind, heading);
        mcl.resample();
    }

    // Estimate should still be somewhat close (no information = no divergence)
    double err_blind = est_error(mcl.estimate(), tx, ty);
    INFO("error after blackout = " << err_blind);
    // Without motion, particles shouldn't have moved much
    CHECK(err_blind < 10.0);

    // Restore sensors
    for (int t = 0; t < 15; t++) {
        mcl.predict(0.0, 0.0, heading);
        mcl.update(readings, heading);
        mcl.resample();
    }

    double err_recovered = est_error(mcl.estimate(), tx, ty);
    INFO("error after recovery = " << err_recovered);
    CHECK(err_recovered < 5.0);
}

TEST_CASE("MCL recovery: sensor blackout WHILE moving, then recovery") {
    MCLConfig cfg;
    cfg.num_particles = 300;

    Field field;
    Physics phys(field);
    phys.set_state({0.0, 0.0, 0.0});

    MCLEngine mcl(cfg);
    mcl.initialize_uniform(42);

    // Converge stationary
    for (int t = 0; t < 20; t++) {
        auto state = phys.state();
        double readings[4];
        perfect_readings(state.x, state.y, state.heading_deg, cfg, readings);
        mcl.predict(0.0, 0.0, state.heading_deg);
        mcl.update(readings, state.heading_deg);
        mcl.resample();
    }

    // Move forward with blacked out sensors for 10 ticks
    double blind[4] = { -1.0, -1.0, -1.0, -1.0 };
    for (int t = 0; t < 10; t++) {
        auto result = phys.step(Action::FORWARD);
        auto state = phys.state();
        mcl.predict(result.delta.forward_in, result.delta.rotation_deg, state.heading_deg);
        mcl.update(blind, state.heading_deg);
        mcl.resample();
    }

    // Sensors come back — continue moving
    for (int t = 0; t < 20; t++) {
        auto result = phys.step(Action::FORWARD);
        auto state = phys.state();
        double readings[4];
        perfect_readings(state.x, state.y, state.heading_deg, cfg, readings);
        mcl.predict(result.delta.forward_in, result.delta.rotation_deg, state.heading_deg);
        mcl.update(readings, state.heading_deg);
        mcl.resample();
    }

    auto state = phys.state();
    double err = est_error(mcl.estimate(), state.x, state.y);
    INFO("error after moving blind then recovery = " << err);
    CHECK(err < 10.0);
}

// ============================================================================
// 3. SYSTEMATICALLY BIASED SENSORS
//    A sensor offset calibration error causes constant bias.
// ============================================================================

TEST_CASE("MCL recovery: constant sensor bias of 2 inches still converges") {
    MCLConfig cfg;
    cfg.num_particles = 300;
    MCLEngine mcl(cfg);
    mcl.initialize_uniform(42);

    double tx = 0.0, ty = 0.0, heading = 0.0;
    double readings[4];
    perfect_readings(tx, ty, heading, cfg, readings);

    // Add constant bias of 2" to all sensors
    for (int i = 0; i < 4; i++) readings[i] += 2.0;

    for (int t = 0; t < 30; t++) {
        mcl.predict(0.0, 0.0, heading);
        mcl.update(readings, heading);
        mcl.resample();
    }

    auto est = mcl.estimate();
    // Won't converge to exact truth (bias), but must be finite and reasonable
    CHECK(std::isfinite(est.x));
    CHECK(std::isfinite(est.y));
    // Shouldn't be wildly wrong — biased readings shift the estimate slightly
    CHECK(est_error(est, tx, ty) < 20.0);
}

// ============================================================================
// 4. TRANSIENT SENSOR SPIKES
//    A sensor occasionally returns absurd values (electromagnetic interference).
// ============================================================================

TEST_CASE("MCL recovery: periodic sensor spikes every 5th tick") {
    MCLConfig cfg;
    cfg.num_particles = 300;
    MCLEngine mcl(cfg);
    mcl.initialize_uniform(42);

    double tx = 25.0, ty = -15.0, heading = 60.0;

    for (int t = 0; t < 40; t++) {
        double readings[4];
        perfect_readings(tx, ty, heading, cfg, readings);

        // Every 5th tick, one sensor returns garbage
        if (t % 5 == 0) {
            readings[t % 4] = 0.1;  // absurdly short
        }

        mcl.predict(0.0, 0.0, heading);
        mcl.update(readings, heading);
        mcl.resample();
    }

    double err = est_error(mcl.estimate(), tx, ty);
    INFO("error with periodic spikes = " << err);
    CHECK(err < 5.0);
}

TEST_CASE("MCL recovery: burst of 5 consecutive garbage readings, then normal") {
    MCLConfig cfg;
    cfg.num_particles = 300;
    MCLEngine mcl(cfg);
    mcl.initialize_uniform(42);

    double tx = 10.0, ty = 10.0, heading = 0.0;
    double good[4];
    perfect_readings(tx, ty, heading, cfg, good);

    // Converge
    for (int t = 0; t < 25; t++) {
        mcl.predict(0.0, 0.0, heading);
        mcl.update(good, heading);
        mcl.resample();
    }

    CHECK(est_error(mcl.estimate(), tx, ty) < 3.0);

    // Burst of garbage
    double garbage[4] = { 0.5, 120.0, 0.01, 95.0 };
    for (int t = 0; t < 5; t++) {
        mcl.predict(0.0, 0.0, heading);
        mcl.update(garbage, heading);
        mcl.resample();
    }

    // Resume good readings
    for (int t = 0; t < 15; t++) {
        mcl.predict(0.0, 0.0, heading);
        mcl.update(good, heading);
        mcl.resample();
    }

    double err = est_error(mcl.estimate(), tx, ty);
    INFO("error after garbage burst + recovery = " << err);
    CHECK(err < 6.0);
}

// ============================================================================
// 5. PARTIAL SENSOR FAILURE
//    One sensor permanently dies. MCL must still track with 3 sensors.
// ============================================================================

TEST_CASE("MCL recovery: front sensor permanently dead, MCL tracks with 3 sensors") {
    MCLConfig cfg;
    cfg.num_particles = 300;

    Field field;
    Physics phys(field);
    phys.set_state({0.0, 0.0, 0.0});

    MCLEngine mcl(cfg);
    mcl.initialize_uniform(42);

    // Converge + move with front sensor always returning -1
    for (int t = 0; t < 20; t++) {
        auto state = phys.state();
        double readings[4];
        perfect_readings(state.x, state.y, state.heading_deg, cfg, readings);
        readings[0] = -1.0;  // Front sensor dead (index 0 = FRONT typically)
        mcl.predict(0.0, 0.0, state.heading_deg);
        mcl.update(readings, state.heading_deg);
        mcl.resample();
    }

    // Move forward 15 ticks with dead front sensor
    for (int t = 0; t < 15; t++) {
        auto result = phys.step(Action::FORWARD);
        auto state = phys.state();
        double readings[4];
        perfect_readings(state.x, state.y, state.heading_deg, cfg, readings);
        readings[0] = -1.0;

        mcl.predict(result.delta.forward_in, result.delta.rotation_deg, state.heading_deg);
        mcl.update(readings, state.heading_deg);
        mcl.resample();
    }

    auto state = phys.state();
    double err = est_error(mcl.estimate(), state.x, state.y);
    INFO("error with dead front sensor = " << err);
    CHECK(err < 8.0);
}

TEST_CASE("MCL recovery: two sensors dead, MCL still produces finite estimates") {
    MCLConfig cfg;
    cfg.num_particles = 300;
    MCLEngine mcl(cfg);
    mcl.initialize_uniform(42);

    double tx = 0.0, ty = 0.0, heading = 0.0;

    for (int t = 0; t < 30; t++) {
        double readings[4];
        perfect_readings(tx, ty, heading, cfg, readings);
        readings[0] = -1.0;  // Front dead
        readings[1] = -1.0;  // Back dead
        mcl.predict(0.0, 0.0, heading);
        mcl.update(readings, heading);
        mcl.resample();
    }

    auto est = mcl.estimate();
    CHECK(std::isfinite(est.x));
    CHECK(std::isfinite(est.y));
    // With only 2 sensors, accuracy is degraded but should be in ballpark
    CHECK(est_error(est, tx, ty) < 15.0);
}

// ============================================================================
// 6. STALL/STUCK ROBOT
//    Robot is physically stuck (collision) but odometry reports motion.
//    This is a classic failure mode on VEX robots.
// ============================================================================

TEST_CASE("MCL recovery: odometry reports motion but sensors show no change") {
    MCLConfig cfg;
    cfg.num_particles = 300;
    MCLEngine mcl(cfg);
    mcl.initialize_uniform(42);

    double tx = 0.0, ty = 0.0, heading = 0.0;
    double readings[4];
    perfect_readings(tx, ty, heading, cfg, readings);

    // Converge
    for (int t = 0; t < 20; t++) {
        mcl.predict(0.0, 0.0, heading);
        mcl.update(readings, heading);
        mcl.resample();
    }

    CHECK(est_error(mcl.estimate(), tx, ty) < 3.0);

    // Now odometry says we're moving forward, but sensors say we're still at (0,0)
    for (int t = 0; t < 20; t++) {
        mcl.predict(1.8, 0.0, heading);  // phantom forward motion
        mcl.update(readings, heading);    // but sensors still see (0,0) readings
        mcl.resample();
    }

    // Sensors should win over phantom odometry — estimate should stay near (0,0)
    double err = est_error(mcl.estimate(), tx, ty);
    INFO("error after phantom motion = " << err);
    CHECK(err < 8.0);
}

// ============================================================================
// 7. RAPID CONVERGENCE / DECONVERGENCE CYCLE
//    Repeatedly converge, corrupt, reconverge to stress memory/state management.
// ============================================================================

TEST_CASE("MCL recovery: 10 converge/corrupt/reconverge cycles") {
    MCLConfig cfg;
    cfg.num_particles = 200;
    MCLEngine mcl(cfg);

    for (int cycle = 0; cycle < 10; cycle++) {
        mcl.initialize_uniform(static_cast<uint64_t>(cycle * 100));

        double tx = (cycle % 3 - 1) * 30.0;
        double ty = (cycle % 2 == 0) ? 25.0 : -25.0;
        double heading = cycle * 36.0;

        double readings[4];
        perfect_readings(tx, ty, heading, cfg, readings);

        for (int t = 0; t < 20; t++) {
            mcl.predict(0.0, 0.0, heading);
            mcl.update(readings, heading);
            mcl.resample();
        }

        auto est = mcl.estimate();
        CHECK(std::isfinite(est.x));
        CHECK(std::isfinite(est.y));

        float sum = 0.0f;
        for (const auto& p : mcl.particles()) sum += p.weight;
        CHECK(sum == doctest::Approx(1.0f).epsilon(1e-3));
    }
}

// ============================================================================
// 8. SENSOR READINGS THAT MATCH A DIFFERENT POSITION
//    Feed readings from position B while robot is at position A.
//    MCL should converge toward B (or at least stay finite).
// ============================================================================

TEST_CASE("MCL recovery: fed wrong-position readings converges to fed position") {
    MCLConfig cfg;
    cfg.num_particles = 300;
    MCLEngine mcl(cfg);
    mcl.initialize_uniform(42);

    // Feed readings from (-30, -30) regardless of where robot actually is
    double fake_x = -30.0, fake_y = -30.0, heading = 0.0;
    double readings[4];
    perfect_readings(fake_x, fake_y, heading, cfg, readings);

    for (int t = 0; t < 30; t++) {
        mcl.predict(0.0, 0.0, heading);
        mcl.update(readings, heading);
        mcl.resample();
    }

    // MCL should converge to the position the readings describe
    double err = est_error(mcl.estimate(), fake_x, fake_y);
    INFO("error converging to fake position = " << err);
    CHECK(err < 5.0);
}
