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
double est_error(const Estimate& est, double tx, double ty) {
    const double dx = est.x - tx;
    const double dy = est.y - ty;
    return std::sqrt(dx * dx + dy * dy);
}

void observe_noisy_readings(
    SensorModel& sensor_model,
    const MCLConfig& cfg,
    double x,
    double y,
    double heading_deg,
    double out[4]
) {
    Vec2 pos{ x, y };
    for (int i = 0; i < 4; ++i) {
        out[i] = sensor_model.observe_distance_sensor(
            pos, heading_deg,
            cfg.sensors[i].offset,
            cfg.sensors[i].angle_deg,
            i
        );
    }
}

double run_noisy_tracking(
    uint64_t seed,
    const std::vector<Action>& actions,
    const OdomNoiseConfig& odom_cfg,
    const SensorNoiseConfig& sensor_cfg,
    const MCLConfig& mcl_cfg,
    double* max_err_out = nullptr
) {
    Field field;
    Physics physics(field);
    physics.set_state({0.0, 0.0, 0.0});

    SensorModel sensor_model(seed + 5000, odom_cfg, sensor_cfg);
    MCLEngine mcl(mcl_cfg);
    mcl.initialize_uniform(seed);

    // Initial warmup convergence with noisy sensors.
    for (int t = 0; t < 25; ++t) {
        auto truth = physics.state();
        double readings[4];
        observe_noisy_readings(sensor_model, mcl_cfg, truth.x, truth.y, truth.heading_deg, readings);
        const double observed_heading = sensor_model.observe_heading(truth.heading_deg);
        mcl.predict(0.0, 0.0, observed_heading, 0.0);
        mcl.update(readings, observed_heading);
        mcl.resample();
    }

    double max_err = 0.0;
    for (Action action : actions) {
        auto step = physics.step(action);
        auto truth = physics.state();

        MotionDelta noisy_odom = sensor_model.apply_odom_noise(step.delta, truth.heading_deg);
        noisy_odom = sensor_model.apply_collision_stall(noisy_odom, step.colliding);
        const double observed_heading = sensor_model.observe_heading(truth.heading_deg);

        double readings[4];
        observe_noisy_readings(sensor_model, mcl_cfg, truth.x, truth.y, truth.heading_deg, readings);

        mcl.predict(noisy_odom.forward_in, noisy_odom.rotation_deg, observed_heading, noisy_odom.lateral_in);
        mcl.update(readings, observed_heading);
        mcl.resample();

        const double err = est_error(mcl.estimate(), truth.x, truth.y);
        if (err > max_err) max_err = err;
    }

    if (max_err_out) *max_err_out = max_err;
    auto truth = physics.state();
    return est_error(mcl.estimate(), truth.x, truth.y);
}
} // namespace

TEST_CASE("MCL noisy sensors: straight-line tracking remains bounded") {
    MCLConfig mcl_cfg;
    mcl_cfg.num_particles = 300;
    mcl_cfg.predict_noise_fwd = 0.03;
    mcl_cfg.predict_noise_lat = 0.015;

    OdomNoiseConfig odom_cfg;
    odom_cfg.trans_noise_frac = 0.03;
    odom_cfg.drift_noise_frac = 0.01;
    odom_cfg.drift_per_tick_in = 0.01;

    SensorNoiseConfig sensor_cfg;
    sensor_cfg.gaussian_stddev_mm = 15.0;
    sensor_cfg.dropout_probability = 0.02;
    sensor_cfg.long_dropout_probability = 0.005;
    sensor_cfg.spurious_reflection_probability = 0.01;
    sensor_cfg.imu_noise_stddev_deg = 0.1;

    std::vector<Action> actions(30, Action::FORWARD);
    double max_err = 0.0;
    const double final_err = run_noisy_tracking(42, actions, odom_cfg, sensor_cfg, mcl_cfg, &max_err);
    INFO("final_err=" << final_err << " max_err=" << max_err);
    CHECK(final_err < 10.0);
    CHECK(max_err < 15.0);
}

TEST_CASE("MCL noisy sensors: mixed maneuver still converges") {
    MCLConfig mcl_cfg;
    mcl_cfg.num_particles = 300;
    mcl_cfg.predict_noise_fwd = 0.03;
    mcl_cfg.predict_noise_lat = 0.015;

    OdomNoiseConfig odom_cfg;
    SensorNoiseConfig sensor_cfg;

    std::vector<Action> actions;
    for (int i = 0; i < 10; ++i) actions.push_back(Action::FORWARD);
    for (int i = 0; i < 5; ++i) actions.push_back(Action::ROTATE_CW);
    for (int i = 0; i < 10; ++i) actions.push_back(Action::FORWARD);
    for (int i = 0; i < 5; ++i) actions.push_back(Action::ROTATE_CCW);
    for (int i = 0; i < 10; ++i) actions.push_back(Action::FORWARD);

    const double final_err = run_noisy_tracking(99, actions, odom_cfg, sensor_cfg, mcl_cfg);
    INFO("final_err=" << final_err);
    CHECK(final_err < 12.0);
}

TEST_CASE("MCL noisy sensors: robust under elevated dropout rates") {
    MCLConfig mcl_cfg;
    mcl_cfg.num_particles = 300;
    mcl_cfg.predict_noise_fwd = 0.03;
    mcl_cfg.predict_noise_lat = 0.015;

    OdomNoiseConfig odom_cfg;
    SensorNoiseConfig sensor_cfg;
    sensor_cfg.dropout_probability = 0.06;
    sensor_cfg.long_dropout_probability = 0.02;

    std::vector<Action> actions(25, Action::FORWARD);
    int pass = 0;
    for (uint64_t seed : {1ULL, 7ULL, 42ULL, 99ULL, 256ULL, 512ULL, 1337ULL, 5000ULL, 9999ULL, 31415ULL}) {
        const double err = run_noisy_tracking(seed, actions, odom_cfg, sensor_cfg, mcl_cfg);
        if (err < 14.0) pass++;
    }
    CHECK(pass >= 7);
}

TEST_CASE("MCL noisy sensors: collision stall pipeline keeps estimate bounded") {
    MCLConfig mcl_cfg;
    mcl_cfg.num_particles = 300;
    mcl_cfg.predict_noise_fwd = 0.03;
    mcl_cfg.predict_noise_lat = 0.015;

    OdomNoiseConfig odom_cfg;
    SensorNoiseConfig sensor_cfg;
    sensor_cfg.collision_stall_ticks = 5;

    // Drive into top wall then back away.
    std::vector<Action> actions;
    for (int i = 0; i < 25; ++i) actions.push_back(Action::FORWARD);
    for (int i = 0; i < 10; ++i) actions.push_back(Action::ROTATE_CW);
    for (int i = 0; i < 15; ++i) actions.push_back(Action::FORWARD);

    double max_err = 0.0;
    const double final_err = run_noisy_tracking(512, actions, odom_cfg, sensor_cfg, mcl_cfg, &max_err);
    INFO("final_err=" << final_err << " max_err=" << max_err);
    CHECK(final_err < 14.0);
    CHECK(max_err < 20.0);
}

TEST_CASE("MCL noisy sensors: random scenarios maintain high pass rate") {
    MCLConfig mcl_cfg;
    mcl_cfg.num_particles = 300;
    mcl_cfg.predict_noise_fwd = 0.03;
    mcl_cfg.predict_noise_lat = 0.015;

    OdomNoiseConfig odom_cfg;
    SensorNoiseConfig sensor_cfg;

    std::mt19937 rng(20260317);
    std::uniform_int_distribution<int> len_dist(10, 25);
    std::uniform_int_distribution<int> action_dist(0, 3); // FWD, BWD, CW, CCW
    const int N = 60;
    int pass = 0;

    for (int i = 0; i < N; ++i) {
        std::vector<Action> actions;
        const int len = len_dist(rng);
        actions.reserve(len);
        for (int k = 0; k < len; ++k) {
            switch (action_dist(rng)) {
                case 0: actions.push_back(Action::FORWARD); break;
                case 1: actions.push_back(Action::BACKWARD); break;
                case 2: actions.push_back(Action::ROTATE_CW); break;
                default: actions.push_back(Action::ROTATE_CCW); break;
            }
        }
        const double err = run_noisy_tracking(static_cast<uint64_t>(1000 + i), actions, odom_cfg, sensor_cfg, mcl_cfg);
        if (err < 14.0) pass++;
    }

    INFO("random noisy pass=" << pass << " / " << N);
    CHECK(pass >= N * 85 / 100);
}
