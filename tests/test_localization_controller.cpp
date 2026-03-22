#include "doctest/doctest.h"

#include "mcl/localization_controller.hpp"

#include <cmath>
#include <regex>

namespace {

void set_all_particles(mcl::LocalizationController& c, float x, float y) {
    auto* ctrl = const_cast<mcl::MCLController*>(c.mcl_controller());
    REQUIRE(ctrl != nullptr);
    auto& particles = const_cast<std::vector<mcl::Particle>&>(ctrl->particles());
    REQUIRE(!particles.empty());
    const float w = 1.0f / static_cast<float>(particles.size());
    for (auto& p : particles) {
        p.x = x;
        p.y = y;
        p.weight = w;
    }
}

distance_loc::DistanceReadings make_center_readings_mm(const distance_loc::SensorOffsets& offsets) {
    const distance_loc::Vec2 pos{0.0, 0.0};
    const double heading = 0.0;
    distance_loc::DistanceReadings r;
    auto to_mm = [](double inches) -> std::optional<int32_t> {
        return static_cast<int32_t>(std::lround(inches * 25.4));
    };
    r.left = to_mm(distance_loc::ray_distance_with_offset(
        pos, heading, {offsets.left.x, offsets.left.y}, -90.0));
    r.right = to_mm(distance_loc::ray_distance_with_offset(
        pos, heading, {offsets.right.x, offsets.right.y}, 90.0));
    r.front = to_mm(distance_loc::ray_distance_with_offset(
        pos, heading, {offsets.front.x, offsets.front.y}, 0.0));
    r.back = to_mm(distance_loc::ray_distance_with_offset(
        pos, heading, {offsets.back.x, offsets.back.y}, 180.0));
    return r;
}

} // namespace

TEST_CASE("LocalizationController MCL tick produces output") {
    mcl::ControllerConfig cfg;
    cfg.algorithm = mcl::LocAlgorithm::MCL;
    cfg.mcl_config.num_particles = 80;
    cfg.min_sensors_for_update = 5; // force skip; deterministic
    mcl::LocalizationController c(cfg);

    mcl::TickInput in;
    in.odom_pose = {0.0, 0.0, 0.0};
    in.imu_heading_deg = 0.0;
    in.sensors.left = 1000;
    in.sensors.right = 1000;

    const auto out = c.tick(in);
    CHECK(out.algorithm_used == mcl::LocAlgorithm::MCL);
    CHECK(out.post_resample.particles.size() == 80);
    CHECK(out.valid_sensor_count == 2);
    CHECK(out.update_skipped);
    CHECK(!out.timestamp_iso.empty());
}

TEST_CASE("LocalizationController RayWall tick runs legacy solver path") {
    mcl::ControllerConfig cfg;
    cfg.algorithm = mcl::LocAlgorithm::RayWall;
    cfg.sensor_offsets = {
        {-6.043f, -1.81102f},
        {6.004f, -2.382f},
        {3.268f, 5.512f},
        {-3.878f, -4.055118f}
    };
    cfg.raywall_loc_config.invalid_reading = 9999;
    cfg.raywall_loc_config.max_jump = 100.0f;
    cfg.raywall_loc_config.max_velocity = 1000.0f;
    cfg.raywall_loc_config.wall_sum_tolerance = 10.0f;
    mcl::LocalizationController c(cfg);

    mcl::TickInput in;
    in.odom_pose = {0.0, 0.0, 0.0};
    in.imu_heading_deg = 0.0;
    in.sensors = make_center_readings_mm(cfg.sensor_offsets);

    const auto out = c.tick(in);
    CHECK(out.algorithm_used == mcl::LocAlgorithm::RayWall);
    CHECK(out.valid_sensor_count >= 2);
    CHECK(!out.timestamp_iso.empty());
}

TEST_CASE("LocalizationController computes odom deltas from absolute pose") {
    mcl::ControllerConfig cfg;
    cfg.algorithm = mcl::LocAlgorithm::MCL;
    cfg.mcl_config.num_particles = 40;
    cfg.min_sensors_for_update = 5;
    cfg.gate_enables = {false, false, false, false, false};
    mcl::LocalizationController c(cfg);
    set_all_particles(c, 0.0f, 0.0f);

    mcl::TickInput t0;
    t0.odom_pose = {0.0, 0.0, 0.0};
    t0.imu_heading_deg = 0.0;
    (void)c.tick(t0);

    mcl::TickInput t1;
    t1.odom_pose = {0.0, 10.0, 0.0}; // +10in forward at heading 0
    t1.imu_heading_deg = 0.0;
    const auto out = c.tick(t1);
    CHECK(out.raw_estimate.y == doctest::Approx(10.0).epsilon(0.05));
}

TEST_CASE("LocalizationController supports set/get pose and algorithm switch") {
    mcl::ControllerConfig cfg;
    cfg.algorithm = mcl::LocAlgorithm::MCL;
    cfg.sensor_offsets = {
        {-6.043f, -1.81102f},
        {6.004f, -2.382f},
        {3.268f, 5.512f},
        {-3.878f, -4.055118f}
    };
    cfg.raywall_loc_config.invalid_reading = 9999;
    cfg.raywall_loc_config.max_jump = 100.0f;
    cfg.raywall_loc_config.max_velocity = 1000.0f;
    cfg.raywall_loc_config.wall_sum_tolerance = 10.0f;
    mcl::LocalizationController c(cfg);
    c.set_pose({10.0, 20.0, 45.0});
    const auto p = c.get_accepted_pose();
    CHECK(p.x == doctest::Approx(10.0));
    CHECK(p.y == doctest::Approx(20.0));
    CHECK(p.theta == doctest::Approx(45.0));

    c.set_algorithm(mcl::LocAlgorithm::RayWall);
    mcl::TickInput in;
    in.odom_pose = {0.0, 0.0, 0.0};
    in.imu_heading_deg = 0.0;
    in.sensors = make_center_readings_mm(cfg.sensor_offsets);
    const auto out = c.tick(in);
    CHECK(out.algorithm_used == mcl::LocAlgorithm::RayWall);
}

TEST_CASE("LocalizationController applies velocity gate") {
    mcl::ControllerConfig cfg;
    cfg.algorithm = mcl::LocAlgorithm::MCL;
    cfg.mcl_config.num_particles = 60;
    cfg.min_sensors_for_update = 5;
    cfg.gate_config.max_estimate_speed_ft_per_s = 1.0;
    cfg.gate_config.max_jump_in = 100.0;
    cfg.gate_config.max_radius_90_in = 1e6;
    cfg.gate_config.max_spread_in = 1e6;
    cfg.gate_config.max_sensor_residual_in = 1e6;
    cfg.gate_config.min_valid_sensors_for_residual = 0;
    cfg.gate_enables = {true, false, false, false, false};
    mcl::LocalizationController c(cfg);
    set_all_particles(c, 30.0f, 0.0f);

    mcl::TickInput in;
    in.odom_pose = {0.0, 0.0, 0.0};
    in.imu_heading_deg = 0.0;
    const auto out = c.tick(in);
    CHECK_FALSE(out.gate.accepted);
    CHECK(out.gate.failed_velocity);
}

TEST_CASE("LocalizationController rejects large jumps when velocity gate enabled") {
    mcl::ControllerConfig cfg;
    cfg.algorithm = mcl::LocAlgorithm::MCL;
    cfg.mcl_config.num_particles = 80;
    cfg.min_sensors_for_update = 5; // skip update path for determinism
    cfg.gate_config.max_estimate_speed_ft_per_s = 0.5;
    cfg.gate_config.max_jump_in = 100.0;
    cfg.gate_enables = {true, false, false, false, false};
    mcl::LocalizationController c(cfg);

    // Force MCL estimate far from odom; velocity gate should reject this correction.
    set_all_particles(c, 100.0f, 100.0f);

    mcl::TickInput t0;
    t0.odom_pose = {0.0, 0.0, 0.0};
    t0.imu_heading_deg = 0.0;
    (void)c.tick(t0);

    mcl::TickInput t1;
    t1.odom_pose = {0.0, 10.0, 0.0}; // +10in forward in odom
    t1.imu_heading_deg = 0.0;
    const auto out = c.tick(t1);

    CHECK_FALSE(out.gate.accepted);
    CHECK(out.gate.failed_velocity);
    CHECK(out.accepted_pose.y < 20.0);
}

TEST_CASE("LocalizationController counts valid sensors with mm/null/invalid") {
    mcl::ControllerConfig cfg;
    cfg.algorithm = mcl::LocAlgorithm::MCL;
    cfg.min_sensors_for_update = 3;
    mcl::LocalizationController c(cfg);

    mcl::TickInput in;
    in.odom_pose = {0.0, 0.0, 0.0};
    in.imu_heading_deg = 0.0;
    in.sensors.left = 2540;         // valid
    in.sensors.right = std::nullopt; // invalid
    in.sensors.front = 0;           // valid
    in.sensors.back = 9999;         // invalid
    const auto out = c.tick(in);
    CHECK(out.valid_sensor_count == 2);
    CHECK(out.update_skipped);
}

TEST_CASE("LocalizationController emits ISO timestamp format") {
    mcl::LocalizationController c;
    mcl::TickInput in;
    in.odom_pose = {0.0, 0.0, 0.0};
    in.imu_heading_deg = 0.0;

    const auto a = c.tick(in);
    const auto b = c.tick(in);
    const std::regex iso_re("^\\d{4}-\\d{2}-\\d{2}T\\d{2}:\\d{2}:\\d{2}\\.\\d{3}Z$");
    CHECK(std::regex_match(a.timestamp_iso, iso_re));
    CHECK(std::regex_match(b.timestamp_iso, iso_re));
}

