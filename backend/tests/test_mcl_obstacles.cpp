#include "doctest/doctest.h"

#include "distance_localization.hpp"
#include "mcl/mcl_engine.hpp"
#include "sim/field.hpp"
#include "sim/physics.hpp"
#include "sim/sensor_model.hpp"

#include <cmath>
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

void observe_obstacle_truth(
    SensorModel& sensor_model,
    const MCLConfig& cfg,
    const Field& field,
    double x,
    double y,
    double heading_deg,
    double out[4]
) {
    const Vec2 pos{x, y};
    for (int i = 0; i < 4; ++i) {
        out[i] = sensor_model.observe_distance_sensor(
            pos, heading_deg, cfg.sensors[i].offset, cfg.sensors[i].angle_deg, i, field.obstacles);
    }
}

double run_obstacle_tracking(uint64_t seed, const std::vector<Action>& actions, const Field& field) {
    MCLConfig cfg;
    cfg.num_particles = 300;
    cfg.predict_noise_fwd = 0.03;
    cfg.predict_noise_lat = 0.015;

    OdomNoiseConfig odom_cfg;
    SensorNoiseConfig sensor_cfg;

    Physics physics(field);
    physics.set_state({0.0, 0.0, 0.0});
    SensorModel sensor_model(seed + 100, odom_cfg, sensor_cfg);
    MCLEngine mcl(cfg);
    mcl.initialize_uniform(seed);

    for (int t = 0; t < 25; ++t) {
        auto truth = physics.state();
        double readings[4];
        observe_obstacle_truth(sensor_model, cfg, field, truth.x, truth.y, truth.heading_deg, readings);
        const double heading_obs = sensor_model.observe_heading(truth.heading_deg);
        mcl.predict(0.0, 0.0, heading_obs, 0.0);
        mcl.update(readings, heading_obs);
        mcl.resample();
    }

    for (Action action : actions) {
        auto step = physics.step(action);
        auto truth = physics.state();

        MotionDelta od = sensor_model.apply_odom_noise(step.delta, truth.heading_deg);
        od = sensor_model.apply_collision_stall(od, step.colliding);
        const double heading_obs = sensor_model.observe_heading(truth.heading_deg);

        double readings[4];
        observe_obstacle_truth(sensor_model, cfg, field, truth.x, truth.y, truth.heading_deg, readings);
        mcl.predict(od.forward_in, od.rotation_deg, heading_obs, od.lateral_in);
        mcl.update(readings, heading_obs);
        mcl.resample();
    }

    auto truth = physics.state();
    return est_error(mcl.estimate(), truth.x, truth.y);
}
} // namespace

TEST_CASE("MCL obstacles: converges near obstacle with sensor/model asymmetry") {
    Field field;
    // Side obstacle mostly affects right-facing rays, not all sensors.
    field.obstacles.push_back({15.0, 5.0, 25.0, 35.0});

    std::vector<Action> actions;
    for (int i = 0; i < 20; ++i) actions.push_back(Action::FORWARD);

    const double err = run_obstacle_tracking(42, actions, field);
    INFO("err=" << err);
    CHECK(err < 20.0);
}

TEST_CASE("MCL obstacles: robust across seeds for fixed obstacle scenario") {
    Field field;
    field.obstacles.push_back({15.0, 5.0, 25.0, 35.0});

    std::vector<Action> actions(20, Action::FORWARD);
    int pass = 0;
    for (uint64_t seed : {1ULL, 7ULL, 42ULL, 99ULL, 256ULL, 512ULL, 1337ULL, 5000ULL, 9999ULL, 31415ULL}) {
        const double err = run_obstacle_tracking(seed, actions, field);
        if (err < 22.0) pass++;
    }
    CHECK(pass >= 6);
}

TEST_CASE("MCL obstacles: wall+obstacle collision path stays bounded") {
    Field field;
    field.obstacles.push_back({-8.0, 40.0, 8.0, 50.0});

    std::vector<Action> actions;
    for (int i = 0; i < 35; ++i) actions.push_back(Action::FORWARD);
    for (int i = 0; i < 10; ++i) actions.push_back(Action::ROTATE_CW);
    for (int i = 0; i < 10; ++i) actions.push_back(Action::FORWARD);

    const double err = run_obstacle_tracking(512, actions, field);
    INFO("err=" << err);
    CHECK(err < 32.0);
}
