#include "doctest/doctest.h"
#include "distance_localization.hpp"
#include "mcl/mcl_engine.hpp"
#include "sim/field.hpp"
#include "sim/physics.hpp"
#include "sim/sensor_model.hpp"

#include <cmath>
#include <random>
#include <vector>

using namespace distance_loc;
using namespace mcl;
using namespace sim;

namespace {
void perfect_readings(double x, double y, double heading_deg, const MCLConfig& cfg, double out[4]) {
    Vec2 pos{x, y};
    for (int i = 0; i < 4; ++i) {
        double d = ray_distance_with_offset(pos, heading_deg, cfg.sensors[i].offset, cfg.sensors[i].angle_deg);
        out[i] = std::isfinite(d) ? d : -1.0;
    }
}

double est_error(const Estimate& est, double tx, double ty) {
    const double dx = est.x - tx;
    const double dy = est.y - ty;
    return std::sqrt(dx * dx + dy * dy);
}

double run_tracking_with_odom_noise(
    uint64_t seed,
    const std::vector<Action>& actions,
    const OdomNoiseConfig& odom_cfg,
    const MCLConfig& mcl_cfg,
    double* max_err_out = nullptr
) {
    Field field;
    Physics physics(field);
    physics.set_state({0.0, 0.0, 0.0});

    SensorModel sensor_model(seed + 1000, odom_cfg);
    MCLEngine mcl(mcl_cfg);
    mcl.initialize_uniform(seed);

    // Initial stationary convergence.
    for (int t = 0; t < 20; ++t) {
        auto truth = physics.state();
        double readings[4];
        perfect_readings(truth.x, truth.y, truth.heading_deg, mcl_cfg, readings);
        mcl.predict(0.0, 0.0, truth.heading_deg);
        mcl.update(readings, truth.heading_deg);
        mcl.resample();
    }

    double max_err = 0.0;
    for (Action action : actions) {
        auto step = physics.step(action);
        auto truth = physics.state();
        double readings[4];
        perfect_readings(truth.x, truth.y, truth.heading_deg, mcl_cfg, readings);

        MotionDelta noisy = sensor_model.apply_odom_noise(step.delta, truth.heading_deg);
        mcl.predict(noisy.forward_in, noisy.rotation_deg, truth.heading_deg, noisy.lateral_in);
        mcl.update(readings, truth.heading_deg);
        mcl.resample();

        double err = est_error(mcl.estimate(), truth.x, truth.y);
        if (err > max_err) max_err = err;
    }

    if (max_err_out) *max_err_out = max_err;
    auto truth = physics.state();
    return est_error(mcl.estimate(), truth.x, truth.y);
}
} // namespace

TEST_CASE("MCL odom noise: straight-line tracking remains accurate") {
    MCLConfig cfg;
    cfg.num_particles = 300;
    cfg.predict_noise_fwd = 0.02;
    cfg.predict_noise_lat = 0.01;

    OdomNoiseConfig odom;
    odom.trans_noise_frac = 0.04;
    odom.drift_noise_frac = 0.015;
    odom.drift_per_tick_in = 0.01;

    std::vector<Action> actions(25, Action::FORWARD);
    double max_err = 0.0;
    double final_err = run_tracking_with_odom_noise(42, actions, odom, cfg, &max_err);

    INFO("final_err=" << final_err << " max_err=" << max_err);
    CHECK(final_err < 8.0);
    CHECK(max_err < 12.0);
}

TEST_CASE("MCL odom noise: rotate-then-forward path tracks under noise") {
    MCLConfig cfg;
    cfg.num_particles = 300;

    OdomNoiseConfig odom;
    odom.trans_noise_frac = 0.04;
    odom.drift_noise_frac = 0.015;
    odom.drift_per_tick_in = 0.01;

    std::vector<Action> actions;
    for (int i = 0; i < 5; ++i) actions.push_back(Action::ROTATE_CW);
    for (int i = 0; i < 20; ++i) actions.push_back(Action::FORWARD);

    double final_err = run_tracking_with_odom_noise(7, actions, odom, cfg);
    INFO("final_err=" << final_err);
    CHECK(final_err < 9.0);
}

TEST_CASE("MCL odom noise: square path remains bounded") {
    MCLConfig cfg;
    cfg.num_particles = 300;

    OdomNoiseConfig odom;
    odom.trans_noise_frac = 0.04;
    odom.drift_noise_frac = 0.015;
    odom.drift_per_tick_in = 0.01;

    std::vector<Action> actions;
    for (int side = 0; side < 4; ++side) {
        for (int i = 0; i < 6; ++i) actions.push_back(Action::FORWARD);
        for (int i = 0; i < 5; ++i) actions.push_back(Action::ROTATE_CW);
    }

    double max_err = 0.0;
    double final_err = run_tracking_with_odom_noise(99, actions, odom, cfg, &max_err);
    INFO("final_err=" << final_err << " max_err=" << max_err);
    CHECK(final_err < 10.0);
    CHECK(max_err < 14.0);
}

TEST_CASE("MCL odom noise: robust over multiple seeds") {
    MCLConfig cfg;
    cfg.num_particles = 300;

    OdomNoiseConfig odom;
    odom.trans_noise_frac = 0.04;
    odom.drift_noise_frac = 0.015;
    odom.drift_per_tick_in = 0.01;

    std::vector<Action> actions(20, Action::FORWARD);
    int pass_count = 0;
    const uint64_t seeds[] = {1, 7, 42, 99, 256, 512, 1337, 5000, 9999, 31415};
    for (uint64_t seed : seeds) {
        double err = run_tracking_with_odom_noise(seed, actions, odom, cfg);
        INFO("seed=" << seed << " err=" << err);
        if (err < 10.0) pass_count++;
    }
    CHECK(pass_count >= 8);
}

TEST_CASE("MCL odom noise: noisy odom remains close to clean baseline on average") {
    MCLConfig cfg;
    cfg.num_particles = 300;

    OdomNoiseConfig clean;
    clean.trans_noise_frac = 0.0;
    clean.drift_noise_frac = 0.0;
    clean.drift_per_tick_in = 0.0;

    OdomNoiseConfig noisy;
    noisy.trans_noise_frac = 0.05;
    noisy.drift_noise_frac = 0.02;
    noisy.drift_per_tick_in = 0.02;

    std::vector<Action> actions(25, Action::FORWARD);
    const uint64_t seeds[] = {1, 7, 42, 99, 256, 512};
    double clean_sum = 0.0;
    double noisy_sum = 0.0;
    for (uint64_t seed : seeds) {
        clean_sum += run_tracking_with_odom_noise(seed, actions, clean, cfg);
        noisy_sum += run_tracking_with_odom_noise(seed, actions, noisy, cfg);
    }

    double clean_avg = clean_sum / 6.0;
    double noisy_avg = noisy_sum / 6.0;
    INFO("clean_avg=" << clean_avg << " noisy_avg=" << noisy_avg);
    // Noise may occasionally help escape poor hypotheses, so do not assume
    // noisy avg must always be higher. Instead, require both to stay accurate
    // and noisy tracking to remain in the same quality band.
    CHECK(clean_avg < 6.0);
    CHECK(noisy_avg < 8.0);
    CHECK(noisy_avg <= clean_avg + 3.0);
}

TEST_CASE("MCL predict noise: non-zero predict noise increases cloud spread") {
    MCLConfig no_noise_cfg;
    no_noise_cfg.num_particles = 300;
    no_noise_cfg.predict_noise_fwd = 0.0;
    no_noise_cfg.predict_noise_lat = 0.0;

    MCLConfig noisy_cfg = no_noise_cfg;
    noisy_cfg.predict_noise_fwd = 0.08;
    noisy_cfg.predict_noise_lat = 0.04;

    MCLEngine a(no_noise_cfg);
    MCLEngine b(noisy_cfg);
    a.initialize_uniform(42);
    b.initialize_uniform(42);

    // Place both particle sets at a single point to measure spread post-predict.
    auto& pa = const_cast<std::vector<Particle>&>(a.particles());
    auto& pb = const_cast<std::vector<Particle>&>(b.particles());
    const float w = 1.0f / static_cast<float>(no_noise_cfg.num_particles);
    for (int i = 0; i < no_noise_cfg.num_particles; ++i) {
        pa[i] = {0.0f, 0.0f, w};
        pb[i] = {0.0f, 0.0f, w};
    }

    a.predict(8.0, 0.0, 30.0, 0.0);
    b.predict(8.0, 0.0, 30.0, 0.0);

    auto compute_spread = [](const std::vector<Particle>& ps) {
        double mx = 0.0, my = 0.0;
        for (const auto& p : ps) {
            mx += p.x;
            my += p.y;
        }
        mx /= ps.size();
        my /= ps.size();
        double var = 0.0;
        for (const auto& p : ps) {
            const double dx = p.x - mx;
            const double dy = p.y - my;
            var += dx * dx + dy * dy;
        }
        return var / ps.size();
    };

    double spread_clean = compute_spread(a.particles());
    double spread_noisy = compute_spread(b.particles());
    INFO("spread_clean=" << spread_clean << " spread_noisy=" << spread_noisy);
    CHECK(spread_noisy > spread_clean + 1e-6);
}
