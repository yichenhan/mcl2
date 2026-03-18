#include "doctest/doctest.h"
#include "distance_localization.hpp"
#include "sim/sensor_model.hpp"

#include <cmath>
#include <vector>

using namespace sim;

namespace {
double sample_mean(const std::vector<double>& xs) {
    double s = 0.0;
    for (double x : xs) s += x;
    return xs.empty() ? 0.0 : s / xs.size();
}

double sample_stddev(const std::vector<double>& xs) {
    if (xs.empty()) return 0.0;
    double m = sample_mean(xs);
    double s2 = 0.0;
    for (double x : xs) {
        double d = x - m;
        s2 += d * d;
    }
    return std::sqrt(s2 / xs.size());
}
} // namespace

TEST_CASE("SensorModel odom noise: zero noise config returns identity delta") {
    OdomNoiseConfig cfg;
    cfg.trans_noise_frac = 0.0;
    cfg.drift_noise_frac = 0.0;
    cfg.drift_per_tick_in = 0.0;

    SensorModel model(123, cfg);
    MotionDelta in{3.5, 0.0, -18.0};
    MotionDelta out = model.apply_odom_noise(in, 45.0);

    CHECK(out.forward_in == doctest::Approx(in.forward_in));
    CHECK(out.lateral_in == doctest::Approx(0.0));
    CHECK(out.rotation_deg == doctest::Approx(in.rotation_deg));
}

TEST_CASE("SensorModel odom noise: deterministic drift adds constant lateral bias") {
    OdomNoiseConfig cfg;
    cfg.trans_noise_frac = 0.0;
    cfg.drift_noise_frac = 0.0;
    cfg.drift_per_tick_in = 0.25;

    SensorModel model(123, cfg);
    MotionDelta in{2.0, 0.0, 0.0};
    MotionDelta out = model.apply_odom_noise(in, 10.0);

    CHECK(out.forward_in == doctest::Approx(2.0));
    CHECK(out.lateral_in == doctest::Approx(0.25));
}

TEST_CASE("SensorModel odom noise: same seed gives identical noisy sequence") {
    OdomNoiseConfig cfg;
    cfg.trans_noise_frac = 0.08;
    cfg.drift_noise_frac = 0.04;
    cfg.drift_per_tick_in = 0.01;

    SensorModel a(9001, cfg);
    SensorModel b(9001, cfg);

    for (int i = 0; i < 200; ++i) {
        MotionDelta in{4.0, 0.0, 1.0};
        auto da = a.apply_odom_noise(in, 30.0);
        auto db = b.apply_odom_noise(in, 30.0);
        CHECK(da.forward_in == db.forward_in);
        CHECK(da.lateral_in == db.lateral_in);
        CHECK(da.rotation_deg == db.rotation_deg);
    }
}

TEST_CASE("SensorModel odom noise: heading does not alter local-frame noisy delta") {
    OdomNoiseConfig cfg;
    cfg.trans_noise_frac = 0.05;
    cfg.drift_noise_frac = 0.03;
    cfg.drift_per_tick_in = 0.0;

    SensorModel a(777, cfg);
    SensorModel b(777, cfg);

    MotionDelta in{5.0, 0.0, 0.0};
    auto d0 = a.apply_odom_noise(in, 0.0);
    auto d1 = b.apply_odom_noise(in, 135.0);

    CHECK(d0.forward_in == d1.forward_in);
    CHECK(d0.lateral_in == d1.lateral_in);
}

TEST_CASE("SensorModel odom noise: translation noise scales with traveled distance") {
    OdomNoiseConfig cfg;
    cfg.trans_noise_frac = 0.1;
    cfg.drift_noise_frac = 0.0;
    cfg.drift_per_tick_in = 0.0;

    SensorModel short_model(42, cfg);
    SensorModel long_model(42, cfg);

    std::vector<double> short_errors;
    std::vector<double> long_errors;
    for (int i = 0; i < 5000; ++i) {
        auto s = short_model.apply_odom_noise(MotionDelta{1.0, 0.0, 0.0}, 0.0);
        auto l = long_model.apply_odom_noise(MotionDelta{10.0, 0.0, 0.0}, 0.0);
        short_errors.push_back(s.forward_in - 1.0);
        long_errors.push_back(l.forward_in - 10.0);
    }

    double std_short = sample_stddev(short_errors);
    double std_long = sample_stddev(long_errors);
    CHECK(std_long > std_short * 5.0);
}

TEST_CASE("SensorModel odom noise: lateral drift noise scales with traveled distance") {
    OdomNoiseConfig cfg;
    cfg.trans_noise_frac = 0.0;
    cfg.drift_noise_frac = 0.1;
    cfg.drift_per_tick_in = 0.0;

    SensorModel short_model(1337, cfg);
    SensorModel long_model(1337, cfg);

    std::vector<double> short_lat;
    std::vector<double> long_lat;
    for (int i = 0; i < 5000; ++i) {
        auto s = short_model.apply_odom_noise(MotionDelta{1.0, 0.0, 0.0}, 0.0);
        auto l = long_model.apply_odom_noise(MotionDelta{10.0, 0.0, 0.0}, 0.0);
        short_lat.push_back(s.lateral_in);
        long_lat.push_back(l.lateral_in);
    }

    double std_short = sample_stddev(short_lat);
    double std_long = sample_stddev(long_lat);
    CHECK(std_long > std_short * 5.0);
}

TEST_CASE("SensorModel odom noise: zero forward still preserves rotation") {
    OdomNoiseConfig cfg;
    cfg.trans_noise_frac = 0.05;
    cfg.drift_noise_frac = 0.05;
    cfg.drift_per_tick_in = 0.01;

    SensorModel model(321, cfg);
    MotionDelta in{0.0, 0.0, 12.0};
    auto out = model.apply_odom_noise(in, 270.0);

    CHECK(out.rotation_deg == doctest::Approx(12.0));
}

TEST_CASE("SensorModel heading: zero IMU noise returns exact heading") {
    OdomNoiseConfig odom;
    SensorNoiseConfig sensor;
    sensor.imu_noise_stddev_deg = 0.0;
    SensorModel model(123, odom, sensor);

    CHECK(model.observe_heading(0.0) == doctest::Approx(0.0));
    CHECK(model.observe_heading(45.0) == doctest::Approx(45.0));
    CHECK(model.observe_heading(359.9) == doctest::Approx(359.9));
}

TEST_CASE("SensorModel heading: deterministic with same seed") {
    OdomNoiseConfig odom;
    SensorNoiseConfig sensor;
    sensor.imu_noise_stddev_deg = 0.5;
    SensorModel a(99, odom, sensor);
    SensorModel b(99, odom, sensor);

    for (int i = 0; i < 200; ++i) {
        CHECK(a.observe_heading(123.0) == b.observe_heading(123.0));
    }
}

TEST_CASE("SensorModel distance: zero-noise config returns true distance") {
    OdomNoiseConfig odom;
    SensorNoiseConfig sensor;
    sensor.gaussian_stddev_mm = 0.0;
    sensor.dropout_probability = 0.0;
    sensor.long_dropout_probability = 0.0;
    sensor.range_noise_slope = 0.0;
    sensor.spurious_reflection_probability = 0.0;
    SensorModel model(1, odom, sensor);

    CHECK(model.observe_distance_in(12.5, 0) == doctest::Approx(12.5));
    CHECK(model.observe_distance_in(50.0, 1) == doctest::Approx(50.0));
}

TEST_CASE("SensorModel distance: range gate above max returns invalid") {
    OdomNoiseConfig odom;
    SensorNoiseConfig sensor;
    sensor.dropout_probability = 0.0;
    sensor.long_dropout_probability = 0.0;
    SensorModel model(1, odom, sensor);

    // 80 in ~= 2032 mm > 2000 mm gate
    CHECK(model.observe_distance_in(80.0, 0) == doctest::Approx(-1.0));
}

TEST_CASE("SensorModel distance: dropout probability 1 always invalid") {
    OdomNoiseConfig odom;
    SensorNoiseConfig sensor;
    sensor.dropout_probability = 1.0;
    sensor.long_dropout_probability = 0.0;
    SensorModel model(5, odom, sensor);

    for (int i = 0; i < 20; ++i) {
        CHECK(model.observe_distance_in(20.0, 0) == doctest::Approx(-1.0));
    }
}

TEST_CASE("SensorModel distance: spurious reflection scales to 0.3x..0.9x") {
    OdomNoiseConfig odom;
    SensorNoiseConfig sensor;
    sensor.dropout_probability = 0.0;
    sensor.long_dropout_probability = 0.0;
    sensor.gaussian_stddev_mm = 0.0;
    sensor.range_noise_slope = 0.0;
    sensor.spurious_reflection_probability = 1.0;
    SensorModel model(88, odom, sensor);

    for (int i = 0; i < 100; ++i) {
        double out = model.observe_distance_in(40.0, 2);
        CHECK(out >= 12.0); // 0.3 * 40
        CHECK(out <= 36.0); // 0.9 * 40
    }
}

TEST_CASE("SensorModel distance: range ramp increases far-distance noise") {
    OdomNoiseConfig odom;
    SensorNoiseConfig sensor;
    sensor.dropout_probability = 0.0;
    sensor.long_dropout_probability = 0.0;
    sensor.spurious_reflection_probability = 0.0;
    sensor.gaussian_stddev_mm = 5.0;
    sensor.range_noise_start_mm = 1700.0;
    sensor.range_noise_slope = 0.05;

    SensorModel near_model(1234, odom, sensor);
    SensorModel far_model(1234, odom, sensor);
    std::vector<double> near_err;
    std::vector<double> far_err;

    for (int i = 0; i < 5000; ++i) {
        // 30 in (762 mm) below ramp threshold
        near_err.push_back(near_model.observe_distance_in(30.0, 0) - 30.0);
        // 75 in (1905 mm) above threshold but below max gate
        far_err.push_back(far_model.observe_distance_in(75.0, 1) - 75.0);
    }

    CHECK(sample_stddev(far_err) > sample_stddev(near_err) * 1.3);
}

TEST_CASE("SensorModel distance sensor: zero-noise raycast equals true wall distance") {
    OdomNoiseConfig odom;
    SensorNoiseConfig sensor;
    sensor.gaussian_stddev_mm = 0.0;
    sensor.dropout_probability = 0.0;
    sensor.long_dropout_probability = 0.0;
    sensor.range_noise_slope = 0.0;
    sensor.spurious_reflection_probability = 0.0;
    SensorModel model(11, odom, sensor);

    distance_loc::Vec2 pos{0.0, 0.0};
    distance_loc::Vec2 offset{0.0, 0.0};
    const double truth = distance_loc::ray_distance_with_offset(pos, 0.0, offset, 0.0);
    const double observed = model.observe_distance_sensor(pos, 0.0, offset, 0.0, 0);
    CHECK(observed == doctest::Approx(truth));
}

TEST_CASE("SensorModel distance sensor: zero-noise with obstacle returns obstacle hit") {
    OdomNoiseConfig odom;
    SensorNoiseConfig sensor;
    sensor.gaussian_stddev_mm = 0.0;
    sensor.dropout_probability = 0.0;
    sensor.long_dropout_probability = 0.0;
    sensor.range_noise_slope = 0.0;
    sensor.spurious_reflection_probability = 0.0;
    SensorModel model(11, odom, sensor);

    distance_loc::Vec2 pos{0.0, 0.0};
    distance_loc::Vec2 offset{0.0, 0.0};
    std::vector<AABB> obstacles = { {-5.0, 20.0, 5.0, 30.0} };
    const double observed = model.observe_distance_sensor(pos, 0.0, offset, 0.0, 0, obstacles);
    CHECK(observed == doctest::Approx(20.0).epsilon(1e-6));
}

TEST_CASE("SensorModel collision stall: emits zero deltas for configured ticks") {
    OdomNoiseConfig odom;
    SensorNoiseConfig sensor;
    sensor.collision_stall_ticks = 3;
    SensorModel model(17, odom, sensor);

    MotionDelta in{2.0, 0.5, 10.0};

    auto d0 = model.apply_collision_stall(in, true);
    auto d1 = model.apply_collision_stall(in, false);
    auto d2 = model.apply_collision_stall(in, false);
    auto d3 = model.apply_collision_stall(in, false);

    CHECK(d0.forward_in == doctest::Approx(0.0));
    CHECK(d0.lateral_in == doctest::Approx(0.0));
    CHECK(d0.rotation_deg == doctest::Approx(0.0));
    CHECK(d1.forward_in == doctest::Approx(0.0));
    CHECK(d2.forward_in == doctest::Approx(0.0));
    CHECK(d3.forward_in == doctest::Approx(in.forward_in));
}
