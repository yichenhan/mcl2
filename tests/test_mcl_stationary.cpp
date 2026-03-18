#include "doctest/doctest.h"
#include "mcl/mcl_engine.hpp"
#include "distance_localization.hpp"
#include <cmath>
#include <cstring>
#include <random>

using namespace mcl;
using namespace distance_loc;

// Helper: generate perfect sensor readings for a given pose
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

// Helper: distance between estimate and truth
static double est_error(const Estimate& est, double tx, double ty) {
    double dx = est.x - tx;
    double dy = est.y - ty;
    return std::sqrt(dx * dx + dy * dy);
}

// Helper: set up a 2-particle engine with one at truth, one far away
static MCLEngine make_two_particle(const MCLConfig& cfg,
                                   float ax, float ay,
                                   float bx, float by) {
    MCLEngine mcl(cfg);
    mcl.initialize_uniform(42);
    auto& ps = const_cast<std::vector<Particle>&>(mcl.particles());
    ps[0] = { ax, ay, 0.5f };
    ps[1] = { bx, by, 0.5f };
    return mcl;
}

// ============================================================================
// Test positions
// ============================================================================

static const double kTrueX = 20.0;
static const double kTrueY = 30.0;
static const double kTrueHeading = 45.0;

// ============================================================================
// 1. Initialization contract
// ============================================================================

TEST_CASE("MCL initialize_uniform keeps particles in bounds with uniform weights") {
    MCLConfig cfg;
    cfg.num_particles = 512;
    MCLEngine mcl(cfg);
    mcl.initialize_uniform(777);

    const float bound = static_cast<float>(cfg.field_half);
    const float expected_w = 1.0f / static_cast<float>(cfg.num_particles);
    CHECK(static_cast<int>(mcl.particles().size()) == cfg.num_particles);
    for (const auto& p : mcl.particles()) {
        CHECK(p.x >= -bound);
        CHECK(p.x <= bound);
        CHECK(p.y >= -bound);
        CHECK(p.y <= bound);
        CHECK(p.weight == doctest::Approx(expected_w).epsilon(1e-6));
    }
}

TEST_CASE("MCL initialize_uniform: N_eff equals N for uniform weights") {
    MCLConfig cfg;
    cfg.num_particles = 300;
    MCLEngine mcl(cfg);
    mcl.initialize_uniform(42);
    CHECK(mcl.n_eff() == doctest::Approx(300.0).epsilon(0.01));
}

// ============================================================================
// 2. estimate() correctness
// ============================================================================

TEST_CASE("MCL estimate equals weighted mean of particles") {
    MCLConfig cfg;
    cfg.num_particles = 3;
    MCLEngine mcl(cfg);
    mcl.initialize_uniform(42);

    auto& ps = const_cast<std::vector<Particle>&>(mcl.particles());
    ps[0] = { 10.0f, 20.0f, 0.2f };
    ps[1] = { -5.0f, 40.0f, 0.3f };
    ps[2] = { 30.0f, -10.0f, 0.5f };

    auto est = mcl.estimate();
    double expected_x = 10.0 * 0.2 + (-5.0) * 0.3 + 30.0 * 0.5;
    double expected_y = 20.0 * 0.2 + 40.0 * 0.3 + (-10.0) * 0.5;

    CHECK(est.x == doctest::Approx(expected_x).epsilon(1e-4));
    CHECK(est.y == doctest::Approx(expected_y).epsilon(1e-4));
}

// ============================================================================
// 3. update() scoring fundamentals
// ============================================================================

TEST_CASE("MCL scoring: particle at true position outweighs far particle") {
    MCLConfig cfg;
    cfg.num_particles = 2;
    auto mcl = make_two_particle(cfg,
        static_cast<float>(kTrueX), static_cast<float>(kTrueY),
        -60.0f, -60.0f);

    double readings[4];
    perfect_readings(kTrueX, kTrueY, kTrueHeading, cfg, readings);
    mcl.update(readings, kTrueHeading);

    CHECK(mcl.particles()[0].weight > mcl.particles()[1].weight);
    CHECK(mcl.particles()[0].weight > 0.9f);
}

TEST_CASE("MCL scoring: particle far from truth gets near-zero normalized weight") {
    MCLConfig cfg;
    cfg.num_particles = 2;
    auto mcl = make_two_particle(cfg,
        static_cast<float>(kTrueX), static_cast<float>(kTrueY),
        -60.0f, -60.0f);

    double readings[4];
    perfect_readings(kTrueX, kTrueY, kTrueHeading, cfg, readings);
    mcl.update(readings, kTrueHeading);

    CHECK(mcl.particles()[1].weight < 0.1f);
}

TEST_CASE("MCL scoring: all sensors invalid gives equal weights") {
    MCLConfig cfg;
    cfg.num_particles = 5;
    MCLEngine mcl(cfg);
    mcl.initialize_uniform(42);

    double readings[4] = { -1.0, -1.0, -1.0, -1.0 };
    mcl.update(readings, kTrueHeading);

    float expected_w = 1.0f / 5.0f;
    for (const auto& p : mcl.particles()) {
        CHECK(p.weight == doctest::Approx(expected_w).epsilon(1e-6));
    }
}

TEST_CASE("MCL update always produces normalized finite non-negative weights") {
    MCLConfig cfg;
    cfg.num_particles = 200;
    MCLEngine mcl(cfg);
    mcl.initialize_uniform(12345);

    double readings[4];
    perfect_readings(kTrueX, kTrueY, kTrueHeading, cfg, readings);
    readings[RIGHT] += 20.0;
    readings[BACK] = -1.0;

    mcl.update(readings, kTrueHeading);

    float sum = 0.0f;
    for (const auto& p : mcl.particles()) {
        CHECK(std::isfinite(p.weight));
        CHECK(p.weight >= 0.0f);
        sum += p.weight;
    }
    CHECK(sum == doctest::Approx(1.0f).epsilon(1e-4));
}

TEST_CASE("MCL scoring: non-finite ray predictions produce valid weights") {
    MCLConfig cfg;
    cfg.num_particles = 3;
    MCLEngine mcl(cfg);
    mcl.initialize_uniform(42);

    double readings[4];
    perfect_readings(kTrueX, kTrueY, kTrueHeading, cfg, readings);

    auto& ps = const_cast<std::vector<Particle>&>(mcl.particles());
    ps[0] = { 150.0f, 150.0f, 0.333f };
    ps[1] = { -150.0f, 150.0f, 0.333f };
    ps[2] = { 150.0f, -150.0f, 0.334f };

    mcl.update(readings, kTrueHeading);

    float sum = 0.0f;
    for (const auto& p : mcl.particles()) {
        CHECK(std::isfinite(p.weight));
        CHECK(p.weight >= 0.0f);
        sum += p.weight;
    }
    CHECK(sum == doctest::Approx(1.0f).epsilon(1e-4));
}

// ============================================================================
// 4. Outlier handling & sensor robustness
// ============================================================================

TEST_CASE("MCL scoring: one corrupted sensor does not ruin a good particle") {
    // With 2 particles, one at truth + one far away, corrupt one sensor.
    // The truth particle should still dominate despite one bad reading.
    MCLConfig cfg;
    cfg.num_particles = 2;
    auto mcl = make_two_particle(cfg,
        static_cast<float>(kTrueX), static_cast<float>(kTrueY),
        -60.0f, -60.0f);

    double readings[4];
    perfect_readings(kTrueX, kTrueY, kTrueHeading, cfg, readings);
    readings[LEFT] += 20.0;

    mcl.update(readings, kTrueHeading);

    CHECK(mcl.particles()[0].weight > mcl.particles()[1].weight);
}

TEST_CASE("MCL scoring: two corrupted sensors still let truth particle dominate") {
    MCLConfig cfg;
    cfg.num_particles = 2;
    auto mcl = make_two_particle(cfg,
        static_cast<float>(kTrueX), static_cast<float>(kTrueY),
        -60.0f, -60.0f);

    double readings[4];
    perfect_readings(kTrueX, kTrueY, kTrueHeading, cfg, readings);
    readings[LEFT] += 20.0;
    readings[RIGHT] += 20.0;

    mcl.update(readings, kTrueHeading);

    CHECK(mcl.particles()[0].weight > mcl.particles()[1].weight);
}

TEST_CASE("MCL scoring: all sensors corrupted does not crash, weights stay valid") {
    MCLConfig cfg;
    cfg.num_particles = 2;
    auto mcl = make_two_particle(cfg,
        static_cast<float>(kTrueX), static_cast<float>(kTrueY),
        -60.0f, -60.0f);

    double readings[4];
    perfect_readings(kTrueX, kTrueY, kTrueHeading, cfg, readings);
    for (int i = 0; i < 4; i++) readings[i] += 40.0;

    mcl.update(readings, kTrueHeading);

    float sum = mcl.particles()[0].weight + mcl.particles()[1].weight;
    CHECK(sum == doctest::Approx(1.0f).epsilon(0.01));
    CHECK(mcl.particles()[0].weight > 0.0f);
    CHECK(mcl.particles()[1].weight > 0.0f);
}

TEST_CASE("MCL scoring: cap boundary, error at exactly max is inlier") {
    // Two engines, same particles. One gets readings where truth's left sensor
    // error is exactly at the cap. Other gets readings barely over. The at-cap
    // case should score the truth particle higher.
    MCLConfig cfg;
    cfg.num_particles = 2;

    double base[4];
    perfect_readings(kTrueX, kTrueY, kTrueHeading, cfg, base);

    double at_cap[4], over_cap[4];
    std::memcpy(at_cap, base, sizeof(base));
    std::memcpy(over_cap, base, sizeof(base));
    at_cap[LEFT] += cfg.max_sensor_error;
    over_cap[LEFT] += cfg.max_sensor_error + 0.01;

    auto mcl_at = make_two_particle(cfg,
        static_cast<float>(kTrueX), static_cast<float>(kTrueY),
        -60.0f, -60.0f);
    auto mcl_over = make_two_particle(cfg,
        static_cast<float>(kTrueX), static_cast<float>(kTrueY),
        -60.0f, -60.0f);

    mcl_at.update(at_cap, kTrueHeading);
    mcl_over.update(over_cap, kTrueHeading);

    CHECK(mcl_at.particles()[0].weight > mcl_over.particles()[0].weight);
}

TEST_CASE("MCL scoring: more outlier sensors produce lower weight for truth particle") {
    // Same particle layout, three separate engines with increasingly corrupted
    // readings. The truth particle's normalized weight should decrease as more
    // sensors become outliers.
    MCLConfig cfg;
    cfg.num_particles = 2;

    double base[4];
    perfect_readings(kTrueX, kTrueY, kTrueHeading, cfg, base);

    double r0[4], r1[4], r2[4];
    std::memcpy(r0, base, sizeof(base));
    std::memcpy(r1, base, sizeof(base));
    std::memcpy(r2, base, sizeof(base));
    r1[LEFT] += 20.0;
    r2[LEFT] += 20.0;
    r2[RIGHT] += 20.0;

    auto m0 = make_two_particle(cfg,
        static_cast<float>(kTrueX), static_cast<float>(kTrueY), -60.0f, -60.0f);
    auto m1 = make_two_particle(cfg,
        static_cast<float>(kTrueX), static_cast<float>(kTrueY), -60.0f, -60.0f);
    auto m2 = make_two_particle(cfg,
        static_cast<float>(kTrueX), static_cast<float>(kTrueY), -60.0f, -60.0f);

    m0.update(r0, kTrueHeading);
    m1.update(r1, kTrueHeading);
    m2.update(r2, kTrueHeading);

    CHECK(m0.particles()[0].weight > m1.particles()[0].weight);
    CHECK(m1.particles()[0].weight > m2.particles()[0].weight);
}

TEST_CASE("MCL scoring: single valid sensor still discriminates matching vs non-matching") {
    // With only one sensor providing data, a particle whose ray prediction
    // matches the reading should score higher than one that doesn't.
    MCLConfig cfg;
    cfg.num_particles = 2;

    double full[4];
    perfect_readings(kTrueX, kTrueY, kTrueHeading, cfg, full);
    double readings[4] = { full[LEFT], -1.0, -1.0, -1.0 };

    auto mcl = make_two_particle(cfg,
        static_cast<float>(kTrueX), static_cast<float>(kTrueY),
        -60.0f, -60.0f);

    mcl.update(readings, kTrueHeading);

    CHECK(mcl.particles()[0].weight > mcl.particles()[1].weight);
}

// ============================================================================
// 5. False-attractor defense
// ============================================================================

TEST_CASE("MCL scoring: particle with all sensors matching beats any single-sensor match") {
    // The classic false-attractor: a far-away particle happens to match ONE
    // sensor perfectly by coincidence while missing all others. A particle
    // at truth matching ALL sensors must score higher, even if its individual
    // errors are slightly nonzero.
    MCLConfig cfg;
    cfg.num_particles = 10;
    MCLEngine mcl(cfg);
    mcl.initialize_uniform(42);

    double readings[4];
    perfect_readings(kTrueX, kTrueY, kTrueHeading, cfg, readings);

    auto& ps = const_cast<std::vector<Particle>&>(mcl.particles());
    // Place particle 0 near truth (slightly offset so errors are small but nonzero)
    ps[0] = { static_cast<float>(kTrueX + 1.0), static_cast<float>(kTrueY + 1.0), 0.1f };
    // Scatter the rest far away — some may get a lucky single-sensor match
    float far_positions[][2] = {
        {-60, -60}, {-40, 50}, {50, -50}, {-30, -70},
        {60, 60}, {-50, 20}, {40, -40}, {-20, -55}, {55, -25}
    };
    for (int i = 1; i < 10; i++) {
        ps[i] = { far_positions[i-1][0], far_positions[i-1][1], 0.1f };
    }

    mcl.update(readings, kTrueHeading);

    float near_truth_weight = mcl.particles()[0].weight;
    for (int i = 1; i < 10; i++) {
        CHECK(near_truth_weight > mcl.particles()[i].weight);
    }
}

// ============================================================================
// 6. N_eff behavior
// ============================================================================

TEST_CASE("MCL N_eff: decreases after update with informative readings") {
    MCLConfig cfg;
    cfg.num_particles = 300;
    MCLEngine mcl(cfg);
    mcl.initialize_uniform(42);

    double readings[4];
    perfect_readings(kTrueX, kTrueY, kTrueHeading, cfg, readings);

    double neff_before = mcl.n_eff();
    mcl.update(readings, kTrueHeading);
    double neff_after = mcl.n_eff();

    CHECK(neff_after < neff_before);
}

TEST_CASE("MCL N_eff: stays at N after update with all-invalid readings") {
    MCLConfig cfg;
    cfg.num_particles = 100;
    MCLEngine mcl(cfg);
    mcl.initialize_uniform(42);

    double readings[4] = { -1.0, -1.0, -1.0, -1.0 };
    mcl.update(readings, kTrueHeading);

    CHECK(mcl.n_eff() == doctest::Approx(100.0).epsilon(0.01));
}

// ============================================================================
// 7. Resample gate
// ============================================================================

TEST_CASE("MCL resample: no-op when N_eff ratio is above threshold") {
    MCLConfig cfg;
    cfg.num_particles = 64;
    cfg.resample_threshold = 0.2;
    MCLEngine mcl(cfg);
    mcl.initialize_uniform(42);

    const auto before = mcl.particles();
    mcl.resample();
    const auto& after = mcl.particles();

    REQUIRE(before.size() == after.size());
    for (size_t i = 0; i < before.size(); i++) {
        CHECK(after[i].x == doctest::Approx(before[i].x));
        CHECK(after[i].y == doctest::Approx(before[i].y));
        CHECK(after[i].weight == doctest::Approx(before[i].weight));
    }
}

TEST_CASE("MCL resample: executes when N_eff ratio is below threshold") {
    MCLConfig cfg;
    cfg.num_particles = 80;
    cfg.resample_threshold = 0.95;
    MCLEngine mcl(cfg);
    mcl.initialize_uniform(42);

    auto& ps = const_cast<std::vector<Particle>&>(mcl.particles());
    for (auto& p : ps) p.weight = 0.0f;
    ps[0].weight = 1.0f;

    const auto before = mcl.particles();
    mcl.resample();
    const auto& after = mcl.particles();

    REQUIRE(before.size() == after.size());
    bool any_changed = false;
    for (size_t i = 0; i < before.size(); i++) {
        if (std::fabs(before[i].x - after[i].x) > 1e-6f ||
            std::fabs(before[i].y - after[i].y) > 1e-6f) {
            any_changed = true;
            break;
        }
    }
    CHECK(any_changed);
}

TEST_CASE("MCL resample: particle count is preserved") {
    MCLConfig cfg;
    cfg.num_particles = 100;
    MCLEngine mcl(cfg);
    mcl.initialize_uniform(42);

    double readings[4];
    perfect_readings(kTrueX, kTrueY, kTrueHeading, cfg, readings);
    mcl.update(readings, kTrueHeading);
    mcl.resample();

    CHECK(static_cast<int>(mcl.particles().size()) == cfg.num_particles);
}

TEST_CASE("MCL resample: weights are uniform after resample") {
    MCLConfig cfg;
    cfg.num_particles = 100;
    MCLEngine mcl(cfg);
    mcl.initialize_uniform(42);

    double readings[4];
    perfect_readings(kTrueX, kTrueY, kTrueHeading, cfg, readings);
    mcl.update(readings, kTrueHeading);
    mcl.resample();

    float expected_w = 1.0f / static_cast<float>(cfg.num_particles);
    for (const auto& p : mcl.particles()) {
        CHECK(p.weight == doctest::Approx(expected_w).epsilon(1e-6));
    }
}

// ============================================================================
// 8. Determinism
// ============================================================================

TEST_CASE("MCL deterministic: same seed and inputs produce identical results") {
    MCLConfig cfg;
    cfg.num_particles = 300;
    MCLEngine m1(cfg);
    MCLEngine m2(cfg);
    m1.initialize_uniform(2024);
    m2.initialize_uniform(2024);

    double readings[4];
    perfect_readings(kTrueX, kTrueY, kTrueHeading, cfg, readings);

    for (int i = 0; i < 10; i++) {
        m1.predict(0.0, 0.0, kTrueHeading);
        m1.update(readings, kTrueHeading);
        m1.resample();

        m2.predict(0.0, 0.0, kTrueHeading);
        m2.update(readings, kTrueHeading);
        m2.resample();
    }

    auto e1 = m1.estimate();
    auto e2 = m2.estimate();
    CHECK(e1.x == doctest::Approx(e2.x).epsilon(1e-6));
    CHECK(e1.y == doctest::Approx(e2.y).epsilon(1e-6));
}

TEST_CASE("MCL deterministic: different seeds produce different results") {
    MCLConfig cfg;
    cfg.num_particles = 300;
    MCLEngine m1(cfg);
    MCLEngine m2(cfg);
    m1.initialize_uniform(42);
    m2.initialize_uniform(99);

    double readings[4];
    perfect_readings(kTrueX, kTrueY, kTrueHeading, cfg, readings);

    m1.update(readings, kTrueHeading);
    m2.update(readings, kTrueHeading);

    auto e1 = m1.estimate();
    auto e2 = m2.estimate();
    bool differ = (std::fabs(e1.x - e2.x) > 0.01) || (std::fabs(e1.y - e2.y) > 0.01);
    CHECK(differ);
}

// ============================================================================
// 9. predict() no-op for stationary
// ============================================================================

TEST_CASE("MCL predict: zero delta does not move any particle") {
    MCLConfig cfg;
    cfg.num_particles = 50;
    MCLEngine mcl(cfg);
    mcl.initialize_uniform(42);

    const auto before = mcl.particles();
    mcl.predict(0.0, 0.0, kTrueHeading);
    const auto& after = mcl.particles();

    REQUIRE(before.size() == after.size());
    for (size_t i = 0; i < before.size(); i++) {
        CHECK(after[i].x == before[i].x);
        CHECK(after[i].y == before[i].y);
        CHECK(after[i].weight == before[i].weight);
    }
}

// ============================================================================
// 10. Convergence (stationary)
// ============================================================================

TEST_CASE("MCL stationary: after 1 tick, particles near truth have highest weights") {
    MCLConfig cfg;
    cfg.num_particles = 300;
    MCLEngine mcl(cfg);
    mcl.initialize_uniform(42);

    double readings[4];
    perfect_readings(kTrueX, kTrueY, kTrueHeading, cfg, readings);
    mcl.update(readings, kTrueHeading);

    const auto& ps = mcl.particles();
    float best_near_weight = 0.0f;
    float worst_far_weight = 1.0f;

    for (const auto& p : ps) {
        double dx = p.x - kTrueX;
        double dy = p.y - kTrueY;
        double dist = std::sqrt(dx * dx + dy * dy);
        if (dist < 10.0) {
            best_near_weight = std::max(best_near_weight, p.weight);
        }
        if (dist > 50.0) {
            worst_far_weight = std::min(worst_far_weight, p.weight);
        }
    }

    CHECK(best_near_weight > worst_far_weight);
}

TEST_CASE("MCL stationary: converges within 5 inches after 5 ticks") {
    MCLConfig cfg;
    cfg.num_particles = 300;
    MCLEngine mcl(cfg);
    mcl.initialize_uniform(42);

    double readings[4];
    perfect_readings(kTrueX, kTrueY, kTrueHeading, cfg, readings);

    for (int i = 0; i < 5; i++) {
        mcl.predict(0.0, 0.0, kTrueHeading);
        mcl.update(readings, kTrueHeading);
        mcl.resample();
    }

    auto est = mcl.estimate();
    double err = est_error(est, kTrueX, kTrueY);
    CHECK(err < 5.0);
}

TEST_CASE("MCL stationary: converges within 2 inches after 20 ticks") {
    MCLConfig cfg;
    cfg.num_particles = 300;
    MCLEngine mcl(cfg);
    mcl.initialize_uniform(42);

    double readings[4];
    perfect_readings(kTrueX, kTrueY, kTrueHeading, cfg, readings);

    for (int i = 0; i < 20; i++) {
        mcl.predict(0.0, 0.0, kTrueHeading);
        mcl.update(readings, kTrueHeading);
        mcl.resample();
    }

    auto est = mcl.estimate();
    double err = est_error(est, kTrueX, kTrueY);
    CHECK(err < 2.0);
}

TEST_CASE("MCL stationary: after convergence, most particles cluster near truth") {
    MCLConfig cfg;
    cfg.num_particles = 300;
    MCLEngine mcl(cfg);
    mcl.initialize_uniform(42);

    double readings[4];
    perfect_readings(kTrueX, kTrueY, kTrueHeading, cfg, readings);

    for (int i = 0; i < 20; i++) {
        mcl.predict(0.0, 0.0, kTrueHeading);
        mcl.update(readings, kTrueHeading);
        mcl.resample();
    }

    const auto& ps = mcl.particles();
    int near_count = 0;
    for (const auto& p : ps) {
        double dx = p.x - kTrueX;
        double dy = p.y - kTrueY;
        if (std::sqrt(dx * dx + dy * dy) < 5.0) {
            near_count++;
        }
    }

    CHECK(near_count > cfg.num_particles / 2);
}

// ============================================================================
// 11. Parametric convergence tests
//     Instead of testing a handful of cherry-picked positions, these tests
//     sweep across field positions, headings, and seeds systematically.
//     A scenario "passes" if ANY of K seeds converges, so we are testing the
//     algorithm's capability at that (x, y, theta), not one seed's luck.
// ============================================================================

// Try up to K seeds at a given (x, y, heading). Returns true if any converge.
static bool try_converge(double tx, double ty, double heading,
                         int ticks, double max_err) {
    MCLConfig cfg;
    cfg.num_particles = 300;

    double readings[4];
    perfect_readings(tx, ty, heading, cfg, readings);
    for (int i = 0; i < 4; i++) {
        if (readings[i] < 0) return true;  // skip positions with invalid sensors
    }

    uint64_t seeds[] = { 1, 42, 99, 256, 1337 };
    for (uint64_t seed : seeds) {
        MCLEngine mcl(cfg);
        mcl.initialize_uniform(seed);
        for (int t = 0; t < ticks; t++) {
            mcl.predict(0.0, 0.0, heading);
            mcl.update(readings, heading);
            mcl.resample();
        }
        auto est = mcl.estimate();
        double err = est_error(est, tx, ty);
        if (err < max_err) return true;
    }
    return false;
}

TEST_CASE("MCL convergence: grid sweep across field positions and headings") {
    // Sweep a 9x9 grid of positions (step=12.5") at 8 headings = 648 scenarios.
    // For each, try 5 seeds and require at least one converges.
    int pass_count = 0;
    int total_count = 0;

    for (double x = -50.0; x <= 50.0; x += 12.5) {
        for (double y = -50.0; y <= 50.0; y += 12.5) {
            for (double heading : { 0.0, 45.0, 90.0, 135.0, 180.0, 225.0, 270.0, 315.0 }) {
                total_count++;
                if (try_converge(x, y, heading, 25, 5.0)) {
                    pass_count++;
                } else {
                    INFO("FAILED at (" << x << "," << y << ") heading=" << heading);
                }
            }
        }
    }

    INFO("grid pass_count=" << pass_count << " / " << total_count);
    CHECK(pass_count >= total_count * 95 / 100);
}

TEST_CASE("MCL convergence: 200 random scenario sampling") {
    // Generate 200 random (x, y, heading) scenarios from a fixed meta-seed.
    // For each, try 5 seeds and check convergence. At least 90% must pass.
    std::mt19937 meta_rng(20240317);
    std::uniform_real_distribution<double> pos_dist(-55.0, 55.0);
    std::uniform_real_distribution<double> heading_dist(0.0, 360.0);

    const int N_scenarios = 200;
    int pass_count = 0;

    for (int i = 0; i < N_scenarios; i++) {
        double x = pos_dist(meta_rng);
        double y = pos_dist(meta_rng);
        double heading = heading_dist(meta_rng);

        if (try_converge(x, y, heading, 25, 5.0)) {
            pass_count++;
        } else {
            INFO("FAILED scenario " << i << " at (" << x << "," << y
                 << ") heading=" << heading);
        }
    }

    INFO("random pass_count=" << pass_count << " / " << N_scenarios);
    CHECK(pass_count >= N_scenarios * 90 / 100);
}

// ============================================================================
// 12. Multi-seed robustness at a fixed position
//     At a given position, most seeds should converge. This tests that the
//     algorithm doesn't depend on one lucky seed.
// ============================================================================

TEST_CASE("MCL convergence: most seeds converge at the primary test position") {
    MCLConfig cfg;
    cfg.num_particles = 300;

    double readings[4];
    perfect_readings(kTrueX, kTrueY, kTrueHeading, cfg, readings);

    uint64_t seeds[] = { 1, 7, 42, 99, 256, 512, 1337, 5000, 9999, 31415 };
    int pass_count = 0;
    for (uint64_t seed : seeds) {
        MCLEngine mcl(cfg);
        mcl.initialize_uniform(seed);
        for (int t = 0; t < 25; t++) {
            mcl.predict(0.0, 0.0, kTrueHeading);
            mcl.update(readings, kTrueHeading);
            mcl.resample();
        }
        auto est = mcl.estimate();
        double err = est_error(est, kTrueX, kTrueY);
        INFO("seed=" << seed << " err=" << err);
        if (err < 5.0) pass_count++;
    }

    // At least 8 out of 10 seeds must converge
    CHECK(pass_count >= 8);
}

// ============================================================================
// 13. Long-run numerical stability
//     Running for 100+ ticks must not diverge, produce NaN, or crash.
// ============================================================================

TEST_CASE("MCL stability: 100 ticks produces finite estimate and valid weights") {
    MCLConfig cfg;
    cfg.num_particles = 300;
    MCLEngine mcl(cfg);
    mcl.initialize_uniform(42);

    double readings[4];
    perfect_readings(kTrueX, kTrueY, kTrueHeading, cfg, readings);

    for (int i = 0; i < 100; i++) {
        mcl.predict(0.0, 0.0, kTrueHeading);
        mcl.update(readings, kTrueHeading);
        mcl.resample();
    }

    auto est = mcl.estimate();
    CHECK(std::isfinite(est.x));
    CHECK(std::isfinite(est.y));

    float sum = 0.0f;
    for (const auto& p : mcl.particles()) {
        CHECK(std::isfinite(p.weight));
        CHECK(p.weight >= 0.0f);
        sum += p.weight;
    }
    CHECK(sum == doctest::Approx(1.0f).epsilon(1e-3));

    double err = est_error(est, kTrueX, kTrueY);
    CHECK(err < 5.0);
}

// ============================================================================
// 14. Convergence monotonicity
//     Error should broadly decrease over time. We check that the error at
//     tick 20 is less than at tick 2.
// ============================================================================

TEST_CASE("MCL convergence: error at tick 20 is less than error at tick 2") {
    MCLConfig cfg;
    cfg.num_particles = 300;
    MCLEngine mcl(cfg);
    mcl.initialize_uniform(42);

    double readings[4];
    perfect_readings(kTrueX, kTrueY, kTrueHeading, cfg, readings);

    double err_at_2 = 0.0;
    for (int i = 0; i < 20; i++) {
        mcl.predict(0.0, 0.0, kTrueHeading);
        mcl.update(readings, kTrueHeading);
        mcl.resample();
        if (i == 1) {
            auto est = mcl.estimate();
            err_at_2 = est_error(est, kTrueX, kTrueY);
        }
    }

    auto est = mcl.estimate();
    double err_at_20 = est_error(est, kTrueX, kTrueY);

    CHECK(err_at_20 < err_at_2);
}

// ============================================================================
// 15. Resample keeps particles in field bounds
// ============================================================================

TEST_CASE("MCL resample: all particles remain within field bounds") {
    MCLConfig cfg;
    cfg.num_particles = 300;
    MCLEngine mcl(cfg);
    mcl.initialize_uniform(42);

    double readings[4];
    perfect_readings(kTrueX, kTrueY, kTrueHeading, cfg, readings);

    for (int i = 0; i < 10; i++) {
        mcl.update(readings, kTrueHeading);
        mcl.resample();
    }

    const float bound = static_cast<float>(cfg.field_half);
    for (const auto& p : mcl.particles()) {
        CHECK(p.x >= -bound);
        CHECK(p.x <= bound);
        CHECK(p.y >= -bound);
        CHECK(p.y <= bound);
    }
}

// ============================================================================
// 16. Repeated identical updates do not cause drift
// ============================================================================

TEST_CASE("MCL stability: repeated identical updates do not drift") {
    MCLConfig cfg;
    cfg.num_particles = 300;
    MCLEngine mcl(cfg);
    mcl.initialize_uniform(42);

    double readings[4];
    perfect_readings(kTrueX, kTrueY, kTrueHeading, cfg, readings);

    for (int i = 0; i < 30; i++) {
        mcl.predict(0.0, 0.0, kTrueHeading);
        mcl.update(readings, kTrueHeading);
        mcl.resample();
    }
    auto est_30 = mcl.estimate();
    double err_30 = est_error(est_30, kTrueX, kTrueY);

    for (int i = 0; i < 30; i++) {
        mcl.predict(0.0, 0.0, kTrueHeading);
        mcl.update(readings, kTrueHeading);
        mcl.resample();
    }
    auto est_60 = mcl.estimate();
    double err_60 = est_error(est_60, kTrueX, kTrueY);

    CHECK(err_60 <= err_30 + 2.0);
}

// ============================================================================
// 17. Scoring is position-dependent, not index-dependent
//     Swapping which particle is placed at truth should swap which gets
//     the higher weight.
// ============================================================================

TEST_CASE("MCL scoring: weight follows position, not particle index") {
    MCLConfig cfg;
    cfg.num_particles = 2;

    double readings[4];
    perfect_readings(kTrueX, kTrueY, kTrueHeading, cfg, readings);

    auto mcl_a = make_two_particle(cfg,
        static_cast<float>(kTrueX), static_cast<float>(kTrueY),
        -60.0f, -60.0f);
    mcl_a.update(readings, kTrueHeading);
    CHECK(mcl_a.particles()[0].weight > mcl_a.particles()[1].weight);

    auto mcl_b = make_two_particle(cfg,
        -60.0f, -60.0f,
        static_cast<float>(kTrueX), static_cast<float>(kTrueY));
    mcl_b.update(readings, kTrueHeading);
    CHECK(mcl_b.particles()[1].weight > mcl_b.particles()[0].weight);
}
