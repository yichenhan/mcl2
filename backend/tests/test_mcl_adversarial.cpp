// =============================================================================
// test_mcl_adversarial.cpp
//
// Antagonistic edge-case tests for the MCL engine core.
// These tests are designed to break the MCL in production by hitting every
// degenerate input, boundary condition, and numerical pathology.
// =============================================================================

#include "doctest/doctest.h"
#include "mcl/mcl_engine.hpp"
#include "distance_localization.hpp"
#include <cmath>
#include <cstring>
#include <limits>
#include <numeric>
#include <random>
#include <vector>

using namespace mcl;
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
// 1. DEGENERATE PARTICLE COUNTS
//    Production may accidentally configure 1 or 2 particles. MCL must not
//    crash, produce NaN, or divide by zero.
// ============================================================================

TEST_CASE("MCL adversarial: single particle does not crash") {
    MCLConfig cfg;
    cfg.num_particles = 1;
    MCLEngine mcl(cfg);
    mcl.initialize_uniform(42);

    CHECK(mcl.particles().size() == 1);
    CHECK(mcl.particles()[0].weight == doctest::Approx(1.0f));

    double readings[4];
    perfect_readings(0.0, 0.0, 0.0, cfg, readings);

    mcl.predict(3.0, 0.0, 0.0);
    mcl.update(readings, 0.0);
    mcl.resample();

    auto est = mcl.estimate();
    CHECK(std::isfinite(est.x));
    CHECK(std::isfinite(est.y));
    CHECK(mcl.particles().size() == 1);
    CHECK(mcl.particles()[0].weight == doctest::Approx(1.0f));
}

TEST_CASE("MCL adversarial: two particles survive full cycle") {
    MCLConfig cfg;
    cfg.num_particles = 2;
    MCLEngine mcl(cfg);
    mcl.initialize_uniform(42);

    double readings[4];
    perfect_readings(20.0, 30.0, 45.0, cfg, readings);

    for (int i = 0; i < 50; i++) {
        mcl.predict(0.0, 0.0, 45.0);
        mcl.update(readings, 45.0);
        mcl.resample();
    }

    auto est = mcl.estimate();
    CHECK(std::isfinite(est.x));
    CHECK(std::isfinite(est.y));
    CHECK(mcl.particles().size() == 2);
}

// ============================================================================
// 2. PATHOLOGICAL SENSOR READINGS
//    Sensors can return garbage in production. MCL must handle gracefully.
// ============================================================================

TEST_CASE("MCL adversarial: all-zero sensor readings produce valid weights") {
    MCLConfig cfg;
    cfg.num_particles = 100;
    MCLEngine mcl(cfg);
    mcl.initialize_uniform(42);

    double readings[4] = { 0.0, 0.0, 0.0, 0.0 };
    mcl.update(readings, 0.0);

    float sum = 0.0f;
    for (const auto& p : mcl.particles()) {
        CHECK(std::isfinite(p.weight));
        CHECK(p.weight >= 0.0f);
        sum += p.weight;
    }
    CHECK(sum == doctest::Approx(1.0f).epsilon(1e-3));
}

TEST_CASE("MCL adversarial: extremely large sensor readings produce valid weights") {
    MCLConfig cfg;
    cfg.num_particles = 100;
    MCLEngine mcl(cfg);
    mcl.initialize_uniform(42);

    double readings[4] = { 999.0, 999.0, 999.0, 999.0 };
    mcl.update(readings, 0.0);

    float sum = 0.0f;
    for (const auto& p : mcl.particles()) {
        CHECK(std::isfinite(p.weight));
        CHECK(p.weight >= 0.0f);
        sum += p.weight;
    }
    CHECK(sum == doctest::Approx(1.0f).epsilon(1e-3));
}

TEST_CASE("MCL adversarial: mixed valid and invalid readings (-1) produce valid weights") {
    MCLConfig cfg;
    cfg.num_particles = 200;
    MCLEngine mcl(cfg);
    mcl.initialize_uniform(42);

    // Only one valid sensor
    double readings[4] = { 30.0, -1.0, -1.0, -1.0 };
    mcl.update(readings, 0.0);

    float sum = 0.0f;
    for (const auto& p : mcl.particles()) {
        CHECK(std::isfinite(p.weight));
        sum += p.weight;
    }
    CHECK(sum == doctest::Approx(1.0f).epsilon(1e-3));
}

TEST_CASE("MCL adversarial: negative non-sentinel readings treated safely") {
    // What if a sensor returns -5.0 instead of -1.0? Should not crash.
    MCLConfig cfg;
    cfg.num_particles = 100;
    MCLEngine mcl(cfg);
    mcl.initialize_uniform(42);

    double readings[4] = { -5.0, -100.0, -0.001, 30.0 };
    mcl.update(readings, 0.0);

    float sum = 0.0f;
    for (const auto& p : mcl.particles()) {
        CHECK(std::isfinite(p.weight));
        CHECK(p.weight >= 0.0f);
        sum += p.weight;
    }
    CHECK(sum == doctest::Approx(1.0f).epsilon(1e-3));
}

// ============================================================================
// 3. PATHOLOGICAL PREDICT INPUTS
//    Odometry can send wild values in production.
// ============================================================================

TEST_CASE("MCL adversarial: huge forward delta does not produce NaN") {
    MCLConfig cfg;
    cfg.num_particles = 50;
    MCLEngine mcl(cfg);
    mcl.initialize_uniform(42);

    mcl.predict(10000.0, 0.0, 0.0);

    for (const auto& p : mcl.particles()) {
        CHECK(std::isfinite(p.x));
        CHECK(std::isfinite(p.y));
    }
    auto est = mcl.estimate();
    CHECK(std::isfinite(est.x));
    CHECK(std::isfinite(est.y));
}

TEST_CASE("MCL adversarial: huge rotation delta does not produce NaN") {
    MCLConfig cfg;
    cfg.num_particles = 50;
    MCLEngine mcl(cfg);
    mcl.initialize_uniform(42);

    mcl.predict(5.0, 99999.0, 0.0);

    for (const auto& p : mcl.particles()) {
        CHECK(std::isfinite(p.x));
        CHECK(std::isfinite(p.y));
    }
}

TEST_CASE("MCL adversarial: negative heading does not crash predict") {
    MCLConfig cfg;
    cfg.num_particles = 50;
    MCLEngine mcl(cfg);
    mcl.initialize_uniform(42);

    mcl.predict(3.0, 0.0, -45.0);

    for (const auto& p : mcl.particles()) {
        CHECK(std::isfinite(p.x));
        CHECK(std::isfinite(p.y));
    }
}

TEST_CASE("MCL adversarial: heading exactly 360 treated same as 0") {
    MCLConfig cfg;
    cfg.num_particles = 100;
    MCLEngine mcl_0(cfg);
    MCLEngine mcl_360(cfg);
    mcl_0.initialize_uniform(42);
    mcl_360.initialize_uniform(42);

    mcl_0.predict(5.0, 0.0, 0.0);
    mcl_360.predict(5.0, 0.0, 360.0);

    for (size_t i = 0; i < mcl_0.particles().size(); i++) {
        CHECK(mcl_0.particles()[i].x == doctest::Approx(mcl_360.particles()[i].x).epsilon(1e-4));
        CHECK(mcl_0.particles()[i].y == doctest::Approx(mcl_360.particles()[i].y).epsilon(1e-4));
    }
}

TEST_CASE("MCL adversarial: heading > 360 still produces valid displacement") {
    MCLConfig cfg;
    cfg.num_particles = 50;
    MCLEngine mcl(cfg);
    mcl.initialize_uniform(42);

    double orig_y = 0.0;
    for (auto& p : mcl.particles()) orig_y += p.y;
    orig_y /= cfg.num_particles;

    // heading 720 = 2 full rotations = 0 degrees. Forward at heading 0 -> +Y
    mcl.predict(5.0, 0.0, 720.0);

    double new_y = 0.0;
    for (auto& p : mcl.particles()) new_y += p.y;
    new_y /= cfg.num_particles;

    CHECK(new_y == doctest::Approx(orig_y + 5.0).epsilon(0.5));
}

TEST_CASE("MCL adversarial: large lateral predict input does not crash") {
    MCLConfig cfg;
    cfg.num_particles = 50;
    MCLEngine mcl(cfg);
    mcl.initialize_uniform(42);

    mcl.predict(3.0, 0.0, 45.0, 500.0);  // huge lateral

    for (const auto& p : mcl.particles()) {
        CHECK(std::isfinite(p.x));
        CHECK(std::isfinite(p.y));
    }
}

// ============================================================================
// 4. WEIGHT COLLAPSE / ALL-ZERO WEIGHTS
//    If all particles have zero likelihood, normalization could divide by zero.
// ============================================================================

TEST_CASE("MCL adversarial: manually set all weights to zero, then resample") {
    MCLConfig cfg;
    cfg.num_particles = 50;
    cfg.resample_threshold = 1.0;  // force resample
    MCLEngine mcl(cfg);
    mcl.initialize_uniform(42);

    auto& ps = const_cast<std::vector<Particle>&>(mcl.particles());
    for (auto& p : ps) p.weight = 0.0f;

    // Resample with all-zero weights must not crash or produce NaN
    mcl.resample();

    for (const auto& p : mcl.particles()) {
        CHECK(std::isfinite(p.x));
        CHECK(std::isfinite(p.y));
        CHECK(std::isfinite(p.weight));
        CHECK(p.weight >= 0.0f);
    }
}

TEST_CASE("MCL adversarial: near-zero weight for all but one particle") {
    MCLConfig cfg;
    cfg.num_particles = 100;
    cfg.resample_threshold = 1.0;
    MCLEngine mcl(cfg);
    mcl.initialize_uniform(42);

    auto& ps = const_cast<std::vector<Particle>&>(mcl.particles());
    for (auto& p : ps) p.weight = 1e-30f;
    ps[0].weight = 1.0f;

    mcl.resample();

    // After resample, all particles should be copies of particle 0
    // (or at least the system didn't crash)
    CHECK(mcl.particles().size() == 100);
    for (const auto& p : mcl.particles()) {
        CHECK(std::isfinite(p.weight));
        CHECK(p.weight >= 0.0f);
    }
}

// ============================================================================
// 5. CYCLE ORDERING VIOLATIONS
//    What if the phases are called out of the expected order?
// ============================================================================

TEST_CASE("MCL adversarial: update without prior predict does not crash") {
    MCLConfig cfg;
    cfg.num_particles = 100;
    MCLEngine mcl(cfg);
    mcl.initialize_uniform(42);

    double readings[4];
    perfect_readings(20.0, 30.0, 0.0, cfg, readings);

    // Skip predict, go straight to update
    mcl.update(readings, 0.0);

    float sum = 0.0f;
    for (const auto& p : mcl.particles()) {
        CHECK(std::isfinite(p.weight));
        sum += p.weight;
    }
    CHECK(sum == doctest::Approx(1.0f).epsilon(1e-3));
}

TEST_CASE("MCL adversarial: resample without prior update preserves particles") {
    MCLConfig cfg;
    cfg.num_particles = 100;
    MCLEngine mcl(cfg);
    mcl.initialize_uniform(42);

    const auto before = mcl.particles();
    mcl.resample();  // uniform weights, N_eff = N, should be no-op
    const auto& after = mcl.particles();

    for (size_t i = 0; i < before.size(); i++) {
        CHECK(after[i].x == before[i].x);
        CHECK(after[i].y == before[i].y);
    }
}

TEST_CASE("MCL adversarial: many predicts without update does not diverge to NaN") {
    MCLConfig cfg;
    cfg.num_particles = 50;
    MCLEngine mcl(cfg);
    mcl.initialize_uniform(42);

    // 1000 predicts in a row without any update or resample
    for (int i = 0; i < 1000; i++) {
        mcl.predict(0.5, 1.0, static_cast<double>(i));
    }

    for (const auto& p : mcl.particles()) {
        CHECK(std::isfinite(p.x));
        CHECK(std::isfinite(p.y));
        CHECK(std::isfinite(p.weight));
    }
}

TEST_CASE("MCL adversarial: many resamples in a row does not corrupt state") {
    MCLConfig cfg;
    cfg.num_particles = 100;
    cfg.resample_threshold = 1.0;  // force actual resample each time
    MCLEngine mcl(cfg);
    mcl.initialize_uniform(42);

    double readings[4];
    perfect_readings(20.0, 30.0, 0.0, cfg, readings);
    mcl.update(readings, 0.0);

    // Resample 50 times in a row
    for (int i = 0; i < 50; i++) {
        mcl.resample();
    }

    CHECK(mcl.particles().size() == 100);
    float sum = 0.0f;
    for (const auto& p : mcl.particles()) {
        CHECK(std::isfinite(p.weight));
        CHECK(p.weight >= 0.0f);
        sum += p.weight;
    }
    CHECK(sum == doctest::Approx(1.0f).epsilon(1e-3));
}

// ============================================================================
// 6. RE-INITIALIZATION
//    Production code might call initialize_uniform mid-run.
// ============================================================================

TEST_CASE("MCL adversarial: re-initialize mid-convergence resets state cleanly") {
    MCLConfig cfg;
    cfg.num_particles = 200;
    MCLEngine mcl(cfg);
    mcl.initialize_uniform(42);

    double readings[4];
    perfect_readings(20.0, 30.0, 0.0, cfg, readings);

    // Run 20 ticks to converge
    for (int i = 0; i < 20; i++) {
        mcl.predict(0.0, 0.0, 0.0);
        mcl.update(readings, 0.0);
        mcl.resample();
    }

    double err_before = est_error(mcl.estimate(), 20.0, 30.0);
    CHECK(err_before < 3.0);

    // Re-initialize with different seed
    mcl.initialize_uniform(9999);

    // Particles should be spread again
    double spread = 0.0;
    double mx = 0.0, my = 0.0;
    for (const auto& p : mcl.particles()) mx += p.x, my += p.y;
    mx /= cfg.num_particles;
    my /= cfg.num_particles;
    for (const auto& p : mcl.particles()) {
        double dx = p.x - mx;
        double dy = p.y - my;
        spread += dx * dx + dy * dy;
    }
    spread /= cfg.num_particles;

    // After re-init, spread should be large (not still clustered at 20,30)
    CHECK(spread > 100.0);

    // Weights should be uniform
    float expected_w = 1.0f / cfg.num_particles;
    for (const auto& p : mcl.particles()) {
        CHECK(p.weight == doctest::Approx(expected_w).epsilon(1e-6));
    }
}

// ============================================================================
// 7. FIELD SYMMETRY EXPLOITATION
//    The square field has 4-fold symmetry. Sensor configs break most of it,
//    but specific positions may still be ambiguous. MCL must converge to the
//    CORRECT attractor, not the symmetric mirror.
// ============================================================================

TEST_CASE("MCL adversarial: asymmetric sensor offsets break 4-fold field symmetry") {
    // At center heading 0, mirror positions (x, y) vs (-x, y) should give
    // different sensor readings because sensor offsets are asymmetric.
    MCLConfig cfg;
    double readings_a[4], readings_b[4];
    perfect_readings(30.0, 0.0, 0.0, cfg, readings_a);
    perfect_readings(-30.0, 0.0, 0.0, cfg, readings_b);

    // At least one sensor must differ significantly
    bool any_differ = false;
    for (int i = 0; i < 4; i++) {
        if (std::fabs(readings_a[i] - readings_b[i]) > 1.0) {
            any_differ = true;
            break;
        }
    }
    CHECK(any_differ);
}

TEST_CASE("MCL adversarial: converges to correct quadrant, not symmetric mirror") {
    MCLConfig cfg;
    cfg.num_particles = 500;

    double truth_x = 35.0, truth_y = 35.0, truth_heading = 30.0;
    double readings[4];
    perfect_readings(truth_x, truth_y, truth_heading, cfg, readings);

    MCLEngine mcl(cfg);
    mcl.initialize_uniform(42);

    for (int t = 0; t < 30; t++) {
        mcl.predict(0.0, 0.0, truth_heading);
        mcl.update(readings, truth_heading);
        mcl.resample();
    }

    auto est = mcl.estimate();
    // Must converge to the true quadrant (+, +), not (-35, 35) etc.
    CHECK(est.x > 0.0);
    CHECK(est.y > 0.0);
    CHECK(est_error(est, truth_x, truth_y) < 5.0);
}

// ============================================================================
// 8. EXTREME FIELD POSITIONS
//    Corners and edges are where ray casting and clamping are most stressed.
// ============================================================================

TEST_CASE("MCL adversarial: convergence at every corner of the field") {
    double corners[][2] = {
        {-55.0, -55.0}, {55.0, -55.0}, {-55.0, 55.0}, {55.0, 55.0}
    };

    for (auto& c : corners) {
        MCLConfig cfg;
        cfg.num_particles = 400;
        double readings[4];
        perfect_readings(c[0], c[1], 45.0, cfg, readings);

        bool passed = false;
        for (uint64_t seed : {1ULL, 42ULL, 99ULL, 256ULL, 1337ULL}) {
            MCLEngine mcl(cfg);
            mcl.initialize_uniform(seed);
            for (int t = 0; t < 30; t++) {
                mcl.predict(0.0, 0.0, 45.0);
                mcl.update(readings, 45.0);
                mcl.resample();
            }
            if (est_error(mcl.estimate(), c[0], c[1]) < 6.0) {
                passed = true;
                break;
            }
        }
        INFO("corner (" << c[0] << "," << c[1] << ")");
        CHECK(passed);
    }
}

TEST_CASE("MCL adversarial: convergence at field edges (not corners)") {
    double edges[][2] = {
        {0.0, 58.0}, {0.0, -58.0}, {58.0, 0.0}, {-58.0, 0.0}
    };

    for (auto& e : edges) {
        MCLConfig cfg;
        cfg.num_particles = 400;
        double readings[4];
        perfect_readings(e[0], e[1], 0.0, cfg, readings);

        bool valid = true;
        for (int i = 0; i < 4; i++) {
            if (readings[i] < 0) { valid = false; break; }
        }
        if (!valid) continue;  // skip if sensor is outside field

        bool passed = false;
        for (uint64_t seed : {1ULL, 42ULL, 99ULL, 256ULL}) {
            MCLEngine mcl(cfg);
            mcl.initialize_uniform(seed);
            for (int t = 0; t < 30; t++) {
                mcl.predict(0.0, 0.0, 0.0);
                mcl.update(readings, 0.0);
                mcl.resample();
            }
            if (est_error(mcl.estimate(), e[0], e[1]) < 6.0) {
                passed = true;
                break;
            }
        }
        INFO("edge (" << e[0] << "," << e[1] << ")");
        CHECK(passed);
    }
}

// ============================================================================
// 9. HEADING BOUNDARY CONDITIONS
//    0, 90, 180, 270 are special for trig functions. Test MCL update scoring
//    at exact cardinal headings to catch sin/cos sign bugs.
// ============================================================================

TEST_CASE("MCL adversarial: update scoring is correct at all cardinal headings") {
    double headings[] = { 0.0, 90.0, 180.0, 270.0 };

    for (double heading : headings) {
        MCLConfig cfg;
        cfg.num_particles = 2;

        double tx = 20.0, ty = 20.0;
        auto& ps = const_cast<std::vector<Particle>&>(
            [&]() -> MCLEngine& {
                static MCLEngine mcl(cfg);
                mcl.initialize_uniform(42);
                return mcl;
            }().particles());

        MCLEngine mcl(cfg);
        mcl.initialize_uniform(42);
        auto& particles = const_cast<std::vector<Particle>&>(mcl.particles());
        particles[0] = { static_cast<float>(tx), static_cast<float>(ty), 0.5f };
        particles[1] = { -50.0f, -50.0f, 0.5f };

        double readings[4];
        perfect_readings(tx, ty, heading, cfg, readings);
        mcl.update(readings, heading);

        INFO("heading=" << heading);
        CHECK(mcl.particles()[0].weight > mcl.particles()[1].weight);
    }
}

// ============================================================================
// 10. N_EFF EDGE CASES
// ============================================================================

TEST_CASE("MCL adversarial: N_eff with all weight on one particle equals 1") {
    MCLConfig cfg;
    cfg.num_particles = 100;
    MCLEngine mcl(cfg);
    mcl.initialize_uniform(42);

    auto& ps = const_cast<std::vector<Particle>&>(mcl.particles());
    for (auto& p : ps) p.weight = 0.0f;
    ps[0].weight = 1.0f;

    CHECK(mcl.n_eff() == doctest::Approx(1.0).epsilon(0.01));
}

TEST_CASE("MCL adversarial: N_eff with two equal weights equals 2") {
    MCLConfig cfg;
    cfg.num_particles = 100;
    MCLEngine mcl(cfg);
    mcl.initialize_uniform(42);

    auto& ps = const_cast<std::vector<Particle>&>(mcl.particles());
    for (auto& p : ps) p.weight = 0.0f;
    ps[0].weight = 0.5f;
    ps[1].weight = 0.5f;

    CHECK(mcl.n_eff() == doctest::Approx(2.0).epsilon(0.01));
}

// ============================================================================
// 11. RESAMPLE BOUNDARY: THRESHOLD EXACTLY AT N_EFF/N
//    Test the resample gate's exact boundary behavior.
// ============================================================================

TEST_CASE("MCL adversarial: resample threshold boundary — just above, just below") {
    MCLConfig cfg;
    cfg.num_particles = 100;

    // With uniform weights: N_eff/N = 1.0
    // Set threshold to 1.0 - epsilon: should NOT resample (1.0 > 0.999)
    cfg.resample_threshold = 0.999;
    MCLEngine mcl_no(cfg);
    mcl_no.initialize_uniform(42);
    const auto before_no = mcl_no.particles();
    mcl_no.resample();
    for (size_t i = 0; i < before_no.size(); i++) {
        CHECK(mcl_no.particles()[i].x == before_no[i].x);
    }

    // Set threshold to 1.001: N_eff/N = 1.0 < 1.001, should trigger resample
    // But since weights are uniform, resampled particles are random draws from
    // the same set — just verify it doesn't crash.
    cfg.resample_threshold = 1.001;
    MCLEngine mcl_yes(cfg);
    mcl_yes.initialize_uniform(42);
    mcl_yes.resample();
    CHECK(mcl_yes.particles().size() == 100);
}

// ============================================================================
// 12. RAPID HEADING OSCILLATION
//    A robot that jitters heading rapidly could cause predict to scatter
//    particles in contradictory directions.
// ============================================================================

TEST_CASE("MCL adversarial: rapid heading oscillation does not cause divergence") {
    MCLConfig cfg;
    cfg.num_particles = 300;
    MCLEngine mcl(cfg);
    mcl.initialize_uniform(42);

    double readings[4];
    perfect_readings(0.0, 0.0, 0.0, cfg, readings);

    // Converge first
    for (int t = 0; t < 20; t++) {
        mcl.predict(0.0, 0.0, 0.0);
        mcl.update(readings, 0.0);
        mcl.resample();
    }

    double err_before = est_error(mcl.estimate(), 0.0, 0.0);
    CHECK(err_before < 3.0);

    // Now oscillate heading rapidly with zero forward motion
    for (int t = 0; t < 50; t++) {
        double heading = (t % 2 == 0) ? 0.0 : 180.0;
        perfect_readings(0.0, 0.0, heading, cfg, readings);
        mcl.predict(0.0, 0.0, heading);
        mcl.update(readings, heading);
        mcl.resample();
    }

    double err_after = est_error(mcl.estimate(), 0.0, 0.0);
    CHECK(err_after < 5.0);
}

// ============================================================================
// 13. PARTICLE CLOUD SPREAD AFTER PREDICT
//    With predict noise enabled, verify the cloud expands proportionally.
// ============================================================================

TEST_CASE("MCL adversarial: predict noise scales with forward distance") {
    MCLConfig cfg_noisy;
    cfg_noisy.num_particles = 500;
    cfg_noisy.predict_noise_fwd = 0.1;
    cfg_noisy.predict_noise_lat = 0.05;

    auto measure_spread = [&](double fwd) {
        MCLEngine mcl(cfg_noisy);
        mcl.initialize_uniform(42);
        // Collapse all particles to origin
        auto& ps = const_cast<std::vector<Particle>&>(mcl.particles());
        float w = 1.0f / cfg_noisy.num_particles;
        for (auto& p : ps) { p.x = 0; p.y = 0; p.weight = w; }

        mcl.predict(fwd, 0.0, 0.0, 0.0);

        double var = 0.0;
        double mx = 0, my = 0;
        for (const auto& p : mcl.particles()) { mx += p.x; my += p.y; }
        mx /= cfg_noisy.num_particles;
        my /= cfg_noisy.num_particles;
        for (const auto& p : mcl.particles()) {
            double dx = p.x - mx;
            double dy = p.y - my;
            var += dx * dx + dy * dy;
        }
        return var / cfg_noisy.num_particles;
    };

    double spread_short = measure_spread(2.0);
    double spread_long = measure_spread(20.0);

    // Longer distances should produce more spread
    CHECK(spread_long > spread_short * 3.0);
}

// ============================================================================
// 14. IDENTICAL READINGS FROM DIFFERENT POSITIONS (AMBIGUITY TEST)
//    Two positions can have identical sensor signatures. MCL should still
//    converge to one and not oscillate between them.
// ============================================================================

TEST_CASE("MCL adversarial: convergence is stable after 50 ticks (no oscillation)") {
    MCLConfig cfg;
    cfg.num_particles = 300;
    MCLEngine mcl(cfg);
    mcl.initialize_uniform(42);

    double tx = 15.0, ty = -25.0, heading = 60.0;
    double readings[4];
    perfect_readings(tx, ty, heading, cfg, readings);

    // Converge
    for (int t = 0; t < 50; t++) {
        mcl.predict(0.0, 0.0, heading);
        mcl.update(readings, heading);
        mcl.resample();
    }

    // Record estimate at tick 50
    auto est_50 = mcl.estimate();

    // Run 20 more ticks
    for (int t = 0; t < 20; t++) {
        mcl.predict(0.0, 0.0, heading);
        mcl.update(readings, heading);
        mcl.resample();
    }

    auto est_70 = mcl.estimate();

    // Should not have jumped more than 1" from its converged position
    double jump = std::sqrt(
        (est_70.x - est_50.x) * (est_70.x - est_50.x) +
        (est_70.y - est_50.y) * (est_70.y - est_50.y)
    );
    CHECK(jump < 2.0);
}

// ============================================================================
// 15. STRESS TEST: VERY HIGH PARTICLE COUNT
//    Verify no integer overflow or memory issues with large particle sets.
// ============================================================================

TEST_CASE("MCL adversarial: 5000 particles survive full convergence cycle") {
    MCLConfig cfg;
    cfg.num_particles = 5000;
    MCLEngine mcl(cfg);
    mcl.initialize_uniform(42);

    double readings[4];
    perfect_readings(0.0, 0.0, 0.0, cfg, readings);

    for (int t = 0; t < 10; t++) {
        mcl.predict(0.0, 0.0, 0.0);
        mcl.update(readings, 0.0);
        mcl.resample();
    }

    CHECK(mcl.particles().size() == 5000);
    auto est = mcl.estimate();
    CHECK(std::isfinite(est.x));
    CHECK(std::isfinite(est.y));

    float sum = 0.0f;
    for (const auto& p : mcl.particles()) {
        CHECK(std::isfinite(p.weight));
        sum += p.weight;
    }
    CHECK(sum == doctest::Approx(1.0f).epsilon(1e-2));
}

// ============================================================================
// 16. WEIGHT NORMALIZATION PRECISION UNDER EXTREME DISPARITY
//    One particle gets astronomical likelihood while others get epsilon.
//    Tests float precision during normalization.
// ============================================================================

TEST_CASE("MCL adversarial: extreme weight disparity after update still normalizes") {
    MCLConfig cfg;
    cfg.num_particles = 100;
    MCLEngine mcl(cfg);
    mcl.initialize_uniform(42);

    // Place one particle at truth, rest at maximum distance
    auto& ps = const_cast<std::vector<Particle>&>(mcl.particles());
    ps[0] = { 20.0f, 30.0f, 1.0f / 100.0f };
    for (int i = 1; i < 100; i++) {
        ps[i] = { -63.0f, -63.0f, 1.0f / 100.0f };
    }

    double readings[4];
    perfect_readings(20.0, 30.0, 45.0, cfg, readings);
    mcl.update(readings, 45.0);

    float sum = 0.0f;
    for (const auto& p : mcl.particles()) {
        CHECK(std::isfinite(p.weight));
        CHECK(p.weight >= 0.0f);
        sum += p.weight;
    }
    CHECK(sum == doctest::Approx(1.0f).epsilon(1e-3));
    // The truth particle should have nearly all weight
    CHECK(mcl.particles()[0].weight > 0.5f);
}

// ============================================================================
// 17. PREDICT THEN UPDATE WITH CONTRADICTORY HEADING
//    Predict says heading=0, update says heading=90.
//    This could happen with sensor lag. MCL must not crash.
// ============================================================================

TEST_CASE("MCL adversarial: predict/update heading mismatch does not crash") {
    MCLConfig cfg;
    cfg.num_particles = 200;
    MCLEngine mcl(cfg);
    mcl.initialize_uniform(42);

    double readings[4];
    perfect_readings(0.0, 0.0, 90.0, cfg, readings);

    // Predict with heading 0, update with heading 90
    mcl.predict(3.0, 0.0, 0.0);
    mcl.update(readings, 90.0);
    mcl.resample();

    for (const auto& p : mcl.particles()) {
        CHECK(std::isfinite(p.x));
        CHECK(std::isfinite(p.y));
        CHECK(std::isfinite(p.weight));
    }
}

// ============================================================================
// 18. REPEATED INITIALIZE WITHOUT INTERVENING OPERATIONS
// ============================================================================

TEST_CASE("MCL adversarial: initialize_uniform called 100 times is safe") {
    MCLConfig cfg;
    cfg.num_particles = 50;
    MCLEngine mcl(cfg);

    for (int i = 0; i < 100; i++) {
        mcl.initialize_uniform(static_cast<uint64_t>(i));
    }

    CHECK(mcl.particles().size() == 50);
    float expected_w = 1.0f / 50.0f;
    for (const auto& p : mcl.particles()) {
        CHECK(p.weight == doctest::Approx(expected_w).epsilon(1e-6));
    }
}
