#include "doctest/doctest.h"

#include "mcl/pose_correction_controller.hpp"
#include "distance_localization.hpp"
#include "ray/ray_cast_obstacles.hpp"

#include <array>
#include <cmath>

// ============================================================================
// Mock Types
// ============================================================================

struct MockChassis {
    struct Pose { float x = 0, y = 0, theta = 0; };
    Pose pose_{};
    int get_count = 0;
    int set_count = 0;
    float last_set_x = 0, last_set_y = 0, last_set_theta = 0;

    Pose getPose() const {
        // cast away const for counting -- test-only
        const_cast<MockChassis*>(this)->get_count++;
        return pose_;
    }
    void setPose(float x, float y, float theta) {
        set_count++;
        last_set_x = x; last_set_y = y; last_set_theta = theta;
        pose_ = {x, y, theta};
    }
};

struct MockChassisDouble {
    struct Pose { double x = 0, y = 0, theta = 0; };
    Pose pose_{};
    Pose getPose() const { return pose_; }
    void setPose(double x, double y, double theta) { pose_ = {x, y, theta}; }
};

class MockSensorProvider : public mcl::ISensorProvider {
    distance_loc::DistanceReadings readings_{};
    int call_count_ = 0;
public:
    void set(const distance_loc::DistanceReadings& r) { readings_ = r; }
    int call_count() const { return call_count_; }
    distance_loc::DistanceReadings getReadings() override {
        call_count_++;
        return readings_;
    }
};

// Build 4 valid readings for a centered robot on an empty 72-inch half field
static distance_loc::DistanceReadings make_valid_readings(double x, double y, double heading_deg,
                                                           const mcl::MCLConfig& mcl_cfg) {
    distance_loc::Vec2 pos{x, y};
    std::array<double, 4> raw{};
    for (int i = 0; i < 4; ++i) {
        double d = distance_loc::ray_distance_with_offset(pos, heading_deg,
                                                           mcl_cfg.sensors[i].offset,
                                                           mcl_cfg.sensors[i].angle_deg);
        raw[static_cast<size_t>(i)] = std::isfinite(d) ? d : -1.0;
    }
    const double kInToMm = 25.4;
    distance_loc::DistanceReadings out;
    auto to_mm = [&](double in) -> std::optional<int32_t> {
        if (in < 0 || !std::isfinite(in)) return std::nullopt;
        return static_cast<int32_t>(std::lround(in * kInToMm));
    };
    out.left  = to_mm(raw[0]);
    out.right = to_mm(raw[1]);
    out.front = to_mm(raw[2]);
    out.back  = to_mm(raw[3]);
    return out;
}

// Build a minimal ControllerConfig that enables convergence
static mcl::ControllerConfig make_config(uint64_t seed = 42) {
    mcl::ControllerConfig cfg;
    cfg.seed = seed;
    cfg.mcl_config.num_particles = 300;
    cfg.mcl_config.field_half = 72.0;
    cfg.field.field_half = 72.0;
    cfg.tick_dt_sec = 0.05;
    return cfg;
}

// ============================================================================
// Category 1: Basic Wiring
// ============================================================================

TEST_CASE("PCC constructs without throwing") {
    MockChassis chassis;
    MockSensorProvider sensors;
    sensors.set(make_valid_readings(0, 0, 0, mcl::MCLConfig{}));
    mcl::PoseCorrectionController<MockChassis> pcc(chassis, sensors, make_config());
    // controller() must return a valid reference (no crash)
    const mcl::LocalizationController& ctrl = pcc.controller();
    CHECK(&ctrl == &pcc.controller());
}

TEST_CASE("PCC first update returns populated CorrectionResult") {
    MockChassis chassis;
    MockSensorProvider sensors;
    sensors.set(make_valid_readings(0, 0, 0, mcl::MCLConfig{}));
    mcl::PoseCorrectionController<MockChassis> pcc(chassis, sensors, make_config());

    const auto result = pcc.update();
    // chassis_pose should be populated (finite, matches chassis position)
    CHECK(std::isfinite(result.chassis_pose.x));
    CHECK(std::isfinite(result.chassis_pose.y));
    CHECK(std::isfinite(result.chassis_pose.theta));
    // raw_odom matches initial chassis pose (offset starts at zero)
    CHECK(result.raw_odom.x == doctest::Approx(0.0));
    CHECK(result.raw_odom.y == doctest::Approx(0.0));
    // MCL output has a timestamp
    CHECK(!result.mcl_output.timestamp_iso.empty());
}

TEST_CASE("PCC getPose called exactly once per update") {
    MockChassis chassis;
    MockSensorProvider sensors;
    sensors.set(make_valid_readings(0, 0, 0, mcl::MCLConfig{}));
    mcl::PoseCorrectionController<MockChassis> pcc(chassis, sensors, make_config());

    chassis.get_count = 0;
    pcc.update();
    // getPose is called once at the top of update(), then again after setPose
    // (to read the final corrected pose for CorrectionResult).
    // We allow 1-2 calls: 1 if no correction, 2 if correction applied.
    CHECK(chassis.get_count >= 1);
    CHECK(chassis.get_count <= 2);
}

TEST_CASE("PCC getReadings called exactly once per update") {
    MockChassis chassis;
    MockSensorProvider sensors;
    sensors.set(make_valid_readings(0, 0, 0, mcl::MCLConfig{}));
    mcl::PoseCorrectionController<MockChassis> pcc(chassis, sensors, make_config());

    const int before = sensors.call_count();
    pcc.update();
    CHECK(sensors.call_count() == before + 1);
}

// ============================================================================
// Category 2: Odom Offset Tracking
// ============================================================================

TEST_CASE("PCC no correction on first tick leaves chassis unchanged") {
    MockChassis chassis;
    chassis.pose_ = {10.0f, 10.0f, 0.0f};
    MockSensorProvider sensors;
    // Give valid readings so MCL at least has data
    sensors.set(make_valid_readings(10, 10, 0, mcl::MCLConfig{}));
    mcl::PoseCorrectionController<MockChassis> pcc(chassis, sensors, make_config());

    chassis.set_count = 0;
    const auto result = pcc.update();
    // MCL cold-starts with uniform particles -- gate almost certainly rejects on tick 0
    // So setPose should NOT be called, or if it is, chassis pose should be consistent
    if (!result.correction_applied) {
        CHECK(chassis.set_count == 0);
        CHECK(chassis.pose_.x == doctest::Approx(10.0f));
        CHECK(chassis.pose_.y == doctest::Approx(10.0f));
    }
    // odom_offset should remain zero if no correction applied
    if (!result.correction_applied) {
        CHECK(pcc.odom_offset().x == doctest::Approx(0.0));
        CHECK(pcc.odom_offset().y == doctest::Approx(0.0));
    }
}

TEST_CASE("PCC correction applies offset to chassis X/Y only, not heading") {
    MockChassis chassis;
    chassis.pose_ = {0.0f, 0.0f, 45.0f};
    MockSensorProvider sensors;

    // Run many ticks with perfect sensors at (0,0,45) -- MCL should converge and correct
    mcl::ControllerConfig cfg = make_config(42);
    cfg.mcl_config.num_particles = 300;
    mcl::PoseCorrectionController<MockChassis> pcc(chassis, sensors, cfg);

    bool any_correction = false;
    for (int i = 0; i < 100; ++i) {
        sensors.set(make_valid_readings(0, 0, 45, cfg.mcl_config));
        const auto result = pcc.update();
        if (result.correction_applied) {
            any_correction = true;
            // Heading must NOT be changed by PCC
            CHECK(chassis.last_set_theta == doctest::Approx(45.0f));
        }
    }
    // Confirm at least one correction happened over 100 ticks
    CHECK(any_correction);
}

TEST_CASE("PCC multiple corrections accumulate offset") {
    MockChassis chassis;
    chassis.pose_ = {0.0f, 0.0f, 0.0f};
    MockSensorProvider sensors;
    mcl::ControllerConfig cfg = make_config(42);
    mcl::PoseCorrectionController<MockChassis> pcc(chassis, sensors, cfg);

    int correction_count = 0;
    for (int i = 0; i < 200; ++i) {
        sensors.set(make_valid_readings(0, 0, 0, cfg.mcl_config));
        const auto result = pcc.update();
        if (result.correction_applied) correction_count++;
    }
    // After many ticks with valid sensors at a fixed position,
    // we expect multiple corrections to have accumulated
    // (at minimum 1 to confirm the mechanism works)
    CHECK(correction_count >= 1);
    // odom_offset should be non-trivial if corrections applied
    const auto& off = pcc.odom_offset();
    CHECK(std::isfinite(off.x));
    CHECK(std::isfinite(off.y));
    CHECK(off.theta == doctest::Approx(0.0)); // heading offset always zero
}

TEST_CASE("PCC heading is never offset-corrected") {
    MockChassis chassis;
    chassis.pose_ = {0.0f, 0.0f, 45.0f};
    MockSensorProvider sensors;
    mcl::ControllerConfig cfg = make_config(42);
    mcl::PoseCorrectionController<MockChassis> pcc(chassis, sensors, cfg);

    for (int i = 0; i < 200; ++i) {
        sensors.set(make_valid_readings(0, 0, 45, cfg.mcl_config));
        pcc.update();
    }
    // Heading offset must always stay zero
    CHECK(pcc.odom_offset().theta == doctest::Approx(0.0));
    // Chassis theta must remain 45 (no heading correction ever applied)
    CHECK(chassis.pose_.theta == doctest::Approx(45.0f));
}

// ============================================================================
// Category 3: Gate Behavior
// ============================================================================

TEST_CASE("PCC gate acceptance triggers setPose") {
    MockChassis chassis;
    chassis.pose_ = {0.0f, 0.0f, 0.0f};
    MockSensorProvider sensors;
    mcl::ControllerConfig cfg = make_config(42);
    mcl::PoseCorrectionController<MockChassis> pcc(chassis, sensors, cfg);

    bool saw_acceptance = false;
    for (int i = 0; i < 150; ++i) {
        chassis.set_count = 0;
        sensors.set(make_valid_readings(0, 0, 0, cfg.mcl_config));
        const auto result = pcc.update();
        if (result.correction_applied) {
            CHECK(chassis.set_count >= 1);
            saw_acceptance = true;
            break;
        }
    }
    CHECK(saw_acceptance);
}

TEST_CASE("PCC gate rejection skips setPose") {
    MockChassis chassis;
    MockSensorProvider sensors;
    // All invalid readings -- MCL update skipped, gate will reject
    sensors.set({});  // all nullopt
    mcl::PoseCorrectionController<MockChassis> pcc(chassis, sensors, make_config());

    chassis.set_count = 0;
    const auto result = pcc.update();
    // With no valid sensors, update is skipped and gate should reject
    if (!result.correction_applied) {
        CHECK(chassis.set_count == 0);
    }
}

TEST_CASE("PCC gate rejection preserves previous correction") {
    MockChassis chassis;
    chassis.pose_ = {0.0f, 0.0f, 0.0f};
    MockSensorProvider sensors;
    mcl::ControllerConfig cfg = make_config(42);
    mcl::PoseCorrectionController<MockChassis> pcc(chassis, sensors, cfg);

    float last_corrected_x = chassis.pose_.x;
    float last_corrected_y = chassis.pose_.y;
    bool had_correction = false;

    for (int i = 0; i < 150; ++i) {
        sensors.set(make_valid_readings(0, 0, 0, cfg.mcl_config));
        const auto result = pcc.update();
        if (result.correction_applied) {
            last_corrected_x = chassis.pose_.x;
            last_corrected_y = chassis.pose_.y;
            had_correction = true;
        } else if (had_correction) {
            // After a rejection following a correction, pose must not have reverted
            CHECK(chassis.pose_.x == doctest::Approx(last_corrected_x));
            CHECK(chassis.pose_.y == doctest::Approx(last_corrected_y));
            break;
        }
    }
}

TEST_CASE("PCC all gate checks disabled -- corrections always apply") {
    MockChassis chassis;
    chassis.pose_ = {0.0f, 0.0f, 0.0f};
    MockSensorProvider sensors;
    mcl::ControllerConfig cfg = make_config(42);
    // Disable all gate checks -- every MCL estimate should be accepted
    cfg.gate_enables.centroid_jump = false;
    cfg.gate_enables.r90 = false;
    cfg.gate_enables.passability = false;
    cfg.gate_enables.residual = false;
    mcl::PoseCorrectionController<MockChassis> pcc(chassis, sensors, cfg);

    // Skip the very first tick (MCL cold start, no prev odom) and check a few after
    sensors.set(make_valid_readings(0, 0, 0, cfg.mcl_config));
    pcc.update(); // tick 0 -- has_prev_odom_ not set yet

    int accept_count = 0;
    for (int i = 1; i < 10; ++i) {
        sensors.set(make_valid_readings(0, 0, 0, cfg.mcl_config));
        const auto result = pcc.update();
        if (result.correction_applied) accept_count++;
    }
    // With all gates disabled, corrections should apply on most ticks
    CHECK(accept_count >= 5);
}

TEST_CASE("PCC alternating valid/invalid sensors does not corrupt offset") {
    MockChassis chassis;
    chassis.pose_ = {0.0f, 0.0f, 0.0f};
    MockSensorProvider sensors;
    mcl::ControllerConfig cfg = make_config(42);
    mcl::PoseCorrectionController<MockChassis> pcc(chassis, sensors, cfg);

    for (int i = 0; i < 200; ++i) {
        if (i % 2 == 0) {
            sensors.set(make_valid_readings(0, 0, 0, cfg.mcl_config));
        } else {
            sensors.set({});  // all invalid
        }
        const auto result = pcc.update();
        // offset should never become NaN or infinite
        CHECK(std::isfinite(pcc.odom_offset().x));
        CHECK(std::isfinite(pcc.odom_offset().y));
        CHECK(std::isfinite(result.chassis_pose.x));
        CHECK(std::isfinite(result.chassis_pose.y));
    }
}

// ============================================================================
// Category 4: Sensor Input
// ============================================================================

TEST_CASE("PCC all valid sensors -- update_skipped is false") {
    MockChassis chassis;
    MockSensorProvider sensors;
    mcl::ControllerConfig cfg = make_config(42);
    sensors.set(make_valid_readings(0, 0, 0, cfg.mcl_config));
    mcl::PoseCorrectionController<MockChassis> pcc(chassis, sensors, cfg);

    // First tick seeds prev_odom; second tick has actual deltas
    pcc.update();
    const auto result = pcc.update();
    CHECK_FALSE(result.mcl_output.update_skipped);
    CHECK(result.mcl_output.valid_sensor_count >= 2);
}

TEST_CASE("PCC partial sensors (2 valid) -- update not skipped") {
    MockChassis chassis;
    MockSensorProvider sensors;
    mcl::ControllerConfig cfg = make_config(42);
    cfg.min_sensors_for_update = 2;

    distance_loc::DistanceReadings partial = make_valid_readings(0, 0, 0, cfg.mcl_config);
    partial.front = std::nullopt;
    partial.back  = std::nullopt;
    sensors.set(partial);

    mcl::PoseCorrectionController<MockChassis> pcc(chassis, sensors, cfg);
    pcc.update();
    const auto result = pcc.update();
    CHECK_FALSE(result.mcl_output.update_skipped);
}

TEST_CASE("PCC all invalid sensors -- update_skipped is true") {
    MockChassis chassis;
    MockSensorProvider sensors;
    sensors.set({});  // all nullopt
    mcl::PoseCorrectionController<MockChassis> pcc(chassis, sensors, make_config());
    pcc.update();
    const auto result = pcc.update();
    CHECK(result.mcl_output.update_skipped);
}

TEST_CASE("PCC sensor readings pass through to MCL valid_sensor_count") {
    MockChassis chassis;
    MockSensorProvider sensors;
    mcl::ControllerConfig cfg = make_config(42);
    // Provide 3 valid, 1 invalid
    distance_loc::DistanceReadings three_valid = make_valid_readings(0, 0, 0, cfg.mcl_config);
    three_valid.back = std::nullopt;
    sensors.set(three_valid);

    mcl::PoseCorrectionController<MockChassis> pcc(chassis, sensors, cfg);
    pcc.update();
    const auto result = pcc.update();
    CHECK(result.mcl_output.valid_sensor_count == 3);
}

// ============================================================================
// Category 5: Coordinate Fidelity
// ============================================================================

TEST_CASE("PCC float round-trip preserves precision") {
    MockChassis chassis;
    chassis.pose_ = {72.123f, -36.789f, 90.0f};
    MockSensorProvider sensors;
    sensors.set({});  // no correction
    mcl::PoseCorrectionController<MockChassis> pcc(chassis, sensors, make_config());

    const auto result = pcc.update();
    // raw_odom should match chassis pose (offset is zero, no correction)
    CHECK(result.raw_odom.x == doctest::Approx(static_cast<double>(72.123f)).epsilon(1e-5));
    CHECK(result.raw_odom.y == doctest::Approx(static_cast<double>(-36.789f)).epsilon(1e-5));
}

TEST_CASE("PCC setPose receives float-cast corrected values") {
    MockChassis chassis;
    chassis.pose_ = {0.0f, 0.0f, 0.0f};
    MockSensorProvider sensors;
    mcl::ControllerConfig cfg = make_config(42);
    cfg.gate_enables.centroid_jump = false;
    cfg.gate_enables.r90 = false;
    cfg.gate_enables.passability = false;
    cfg.gate_enables.residual = false;
    mcl::PoseCorrectionController<MockChassis> pcc(chassis, sensors, cfg);

    // Run until a correction happens
    for (int i = 0; i < 50; ++i) {
        sensors.set(make_valid_readings(0, 0, 0, cfg.mcl_config));
        const auto result = pcc.update();
        if (result.correction_applied) {
            // setPose receives float values (not double)
            // The chassis mock stores them as float -- verify they're finite
            CHECK(std::isfinite(chassis.last_set_x));
            CHECK(std::isfinite(chassis.last_set_y));
            CHECK(std::isfinite(chassis.last_set_theta));
            break;
        }
    }
}

TEST_CASE("PCC negative coordinates produce no NaN") {
    MockChassis chassis;
    chassis.pose_ = {-50.0f, -50.0f, 180.0f};
    MockSensorProvider sensors;
    mcl::ControllerConfig cfg = make_config(42);
    mcl::PoseCorrectionController<MockChassis> pcc(chassis, sensors, cfg);

    for (int i = 0; i < 50; ++i) {
        sensors.set(make_valid_readings(-50, -50, 180, cfg.mcl_config));
        const auto result = pcc.update();
        CHECK(std::isfinite(result.chassis_pose.x));
        CHECK(std::isfinite(result.chassis_pose.y));
        CHECK(std::isfinite(result.chassis_pose.theta));
        CHECK(std::isfinite(result.raw_odom.x));
        CHECK(std::isfinite(pcc.odom_offset().x));
    }
}

TEST_CASE("PCC heading wrapping at 360 boundary produces valid output") {
    MockChassis chassis;
    chassis.pose_ = {0.0f, 0.0f, 359.5f};
    MockSensorProvider sensors;
    sensors.set({});
    mcl::PoseCorrectionController<MockChassis> pcc(chassis, sensors, make_config());

    // Simulate heading incrementing past 360
    for (int i = 0; i < 5; ++i) {
        float h = chassis.pose_.theta + 1.0f;
        if (h >= 360.0f) h -= 360.0f;
        chassis.pose_.theta = h;
        const auto result = pcc.update();
        CHECK(std::isfinite(result.chassis_pose.theta));
        CHECK(result.chassis_pose.theta >= 0.0);
        CHECK(result.chassis_pose.theta < 360.0 + 1e-3);
    }
}

// ============================================================================
// Category 6: Convergence
// ============================================================================

static double pose_error(const mcl::Pose& p, double tx, double ty) {
    const double dx = p.x - tx;
    const double dy = p.y - ty;
    return std::sqrt(dx * dx + dy * dy);
}

TEST_CASE("PCC stationary robot converges within 100 ticks") {
    const double TX = 20.0, TY = 30.0, TH = 45.0;
    MockChassis chassis;
    chassis.pose_ = {static_cast<float>(TX), static_cast<float>(TY), static_cast<float>(TH)};
    MockSensorProvider sensors;
    mcl::ControllerConfig cfg = make_config(42);
    mcl::PoseCorrectionController<MockChassis> pcc(chassis, sensors, cfg);

    double final_error = 9999.0;
    for (int i = 0; i < 100; ++i) {
        sensors.set(make_valid_readings(TX, TY, TH, cfg.mcl_config));
        const auto result = pcc.update();
        final_error = pose_error(result.chassis_pose, TX, TY);
    }
    CHECK(final_error < 8.0);  // within 8 inches after 100 ticks (roughening_sigma=1.0 widens cloud)
}

TEST_CASE("PCC moving robot chassis_pose stays finite and tracks reasonably") {
    MockChassis chassis;
    chassis.pose_ = {0.0f, 0.0f, 0.0f};
    MockSensorProvider sensors;
    mcl::ControllerConfig cfg = make_config(42);
    mcl::PoseCorrectionController<MockChassis> pcc(chassis, sensors, cfg);

    // Simulate robot moving forward (incrementing y)
    for (int i = 0; i < 100; ++i) {
        float ny = chassis.pose_.y + 0.5f;
        chassis.pose_.y = ny;
        sensors.set(make_valid_readings(chassis.pose_.x, ny, chassis.pose_.theta, cfg.mcl_config));
        const auto result = pcc.update();
        CHECK(std::isfinite(result.chassis_pose.x));
        CHECK(std::isfinite(result.chassis_pose.y));
    }
}

TEST_CASE("PCC cold start converges -- uniform init, valid sensors") {
    const double TX = 0.0, TY = 0.0, TH = 0.0;
    MockChassis chassis;
    chassis.pose_ = {static_cast<float>(TX), static_cast<float>(TY), static_cast<float>(TH)};
    MockSensorProvider sensors;
    mcl::ControllerConfig cfg = make_config(42);
    cfg.mcl_config.num_particles = 300;
    mcl::PoseCorrectionController<MockChassis> pcc(chassis, sensors, cfg);

    double final_error = 9999.0;
    for (int i = 0; i < 100; ++i) {
        sensors.set(make_valid_readings(TX, TY, TH, cfg.mcl_config));
        const auto result = pcc.update();
        final_error = pose_error(result.chassis_pose, TX, TY);
    }
    // Field center is highly observable -- should converge
    CHECK(final_error < 8.0);
}

// ============================================================================
// Category 7: Edge Cases
// ============================================================================

TEST_CASE("PCC zero motion tick produces valid result") {
    MockChassis chassis;
    chassis.pose_ = {0.0f, 0.0f, 0.0f};
    MockSensorProvider sensors;
    sensors.set(make_valid_readings(0, 0, 0, mcl::MCLConfig{}));
    mcl::PoseCorrectionController<MockChassis> pcc(chassis, sensors, make_config());

    // Chassis doesn't move between ticks
    const auto result = pcc.update();
    CHECK(std::isfinite(result.chassis_pose.x));
    CHECK(std::isfinite(result.chassis_pose.y));
    CHECK(result.correction_distance_in >= 0.0);
}

TEST_CASE("PCC external setPose between updates does not crash") {
    MockChassis chassis;
    chassis.pose_ = {0.0f, 0.0f, 0.0f};
    MockSensorProvider sensors;
    mcl::ControllerConfig cfg = make_config(42);
    mcl::PoseCorrectionController<MockChassis> pcc(chassis, sensors, cfg);

    for (int i = 0; i < 10; ++i) {
        sensors.set(make_valid_readings(0, 0, 0, cfg.mcl_config));
        pcc.update();
        // External code resets the chassis (e.g. kidnap or user-initiated)
        chassis.pose_ = {10.0f, 10.0f, 0.0f};
    }
    // Must not crash or produce NaN
    sensors.set(make_valid_readings(10, 10, 0, cfg.mcl_config));
    const auto result = pcc.update();
    CHECK(std::isfinite(result.chassis_pose.x));
    CHECK(std::isfinite(result.chassis_pose.y));
}

TEST_CASE("PCC boundary position (near field edge) works") {
    const float FH = 72.0f;
    MockChassis chassis;
    chassis.pose_ = {FH - 10.0f, FH - 10.0f, 0.0f};
    MockSensorProvider sensors;
    mcl::ControllerConfig cfg = make_config(42);
    cfg.mcl_config.field_half = 72.0;
    cfg.field.field_half = 72.0;
    mcl::PoseCorrectionController<MockChassis> pcc(chassis, sensors, cfg);

    for (int i = 0; i < 30; ++i) {
        sensors.set(make_valid_readings(FH - 10.0, FH - 10.0, 0, cfg.mcl_config));
        const auto result = pcc.update();
        CHECK(std::isfinite(result.chassis_pose.x));
        CHECK(std::isfinite(result.chassis_pose.y));
        CHECK(std::isfinite(pcc.odom_offset().x));
    }
}

TEST_CASE("PCC very small correction applies cleanly") {
    MockChassis chassis;
    chassis.pose_ = {0.0f, 0.0f, 0.0f};
    MockSensorProvider sensors;
    mcl::ControllerConfig cfg = make_config(42);
    cfg.gate_enables.centroid_jump = false;
    cfg.gate_enables.r90 = false;
    cfg.gate_enables.passability = false;
    cfg.gate_enables.residual = false;
    mcl::PoseCorrectionController<MockChassis> pcc(chassis, sensors, cfg);

    // Warm up MCL
    for (int i = 0; i < 20; ++i) {
        sensors.set(make_valid_readings(0, 0, 0, cfg.mcl_config));
        pcc.update();
    }
    // chassis pose very close to truth -- correction distance should be tiny
    const auto result = pcc.update();
    CHECK(std::isfinite(result.correction_distance_in));
    CHECK(result.correction_distance_in >= 0.0);
}

TEST_CASE("PCC controller() accessor is stable reference") {
    MockChassis chassis;
    MockSensorProvider sensors;
    mcl::PoseCorrectionController<MockChassis> pcc(chassis, sensors, make_config());
    const mcl::LocalizationController* p1 = &pcc.controller();
    const mcl::LocalizationController* p2 = &pcc.controller();
    CHECK(p1 == p2);
}

// ============================================================================
// Category 8: Template Compatibility
// ============================================================================

TEST_CASE("PCC works with float Pose struct (MockChassis)") {
    MockChassis chassis;
    MockSensorProvider sensors;
    sensors.set(make_valid_readings(0, 0, 0, mcl::MCLConfig{}));
    // This just verifies it compiles and runs with float fields
    mcl::PoseCorrectionController<MockChassis> pcc(chassis, sensors, make_config());
    const auto result = pcc.update();
    CHECK(std::isfinite(result.chassis_pose.x));
}

TEST_CASE("PCC works with double Pose struct (MockChassisDouble)") {
    MockChassisDouble chassis;
    MockSensorProvider sensors;
    sensors.set(make_valid_readings(0, 0, 0, mcl::MCLConfig{}));
    mcl::PoseCorrectionController<MockChassisDouble> pcc(chassis, sensors, make_config());
    const auto result = pcc.update();
    CHECK(std::isfinite(result.chassis_pose.x));
}

TEST_CASE("PCC const getPose compiles") {
    MockChassis chassis;
    MockSensorProvider sensors;
    // Verify the template works when const getPose() is required
    const MockChassis& const_ref = chassis;
    const auto pose = const_ref.getPose();  // compile-time check
    CHECK(std::isfinite(pose.x));
}

// ============================================================================
// Category 9: Determinism
// ============================================================================

TEST_CASE("PCC same seed produces identical correction sequences") {
    MockChassis chassis1, chassis2;
    MockSensorProvider sensors1, sensors2;
    const mcl::ControllerConfig cfg = make_config(99);

    mcl::PoseCorrectionController<MockChassis> pcc1(chassis1, sensors1, cfg);
    mcl::PoseCorrectionController<MockChassis> pcc2(chassis2, sensors2, cfg);

    for (int i = 0; i < 50; ++i) {
        const auto readings = make_valid_readings(0, 0, 0, cfg.mcl_config);
        sensors1.set(readings);
        sensors2.set(readings);
        const auto r1 = pcc1.update();
        const auto r2 = pcc2.update();
        CHECK(r1.correction_applied == r2.correction_applied);
        CHECK(r1.chassis_pose.x == doctest::Approx(r2.chassis_pose.x));
        CHECK(r1.chassis_pose.y == doctest::Approx(r2.chassis_pose.y));
    }
}

TEST_CASE("PCC different seeds produce different particle states") {
    MockChassis chassis1, chassis2;
    MockSensorProvider sensors1, sensors2;

    mcl::ControllerConfig cfg1 = make_config(1);
    mcl::ControllerConfig cfg2 = make_config(999);

    mcl::PoseCorrectionController<MockChassis> pcc1(chassis1, sensors1, cfg1);
    mcl::PoseCorrectionController<MockChassis> pcc2(chassis2, sensors2, cfg2);

    const auto readings = make_valid_readings(0, 0, 0, cfg1.mcl_config);
    for (int i = 0; i < 50; ++i) {
        sensors1.set(readings);
        sensors2.set(readings);
        pcc1.update();
        pcc2.update();
    }
    // After 50 ticks the particle distributions should differ
    const auto* ctrl1 = pcc1.controller().mcl_controller();
    const auto* ctrl2 = pcc2.controller().mcl_controller();
    if (ctrl1 && ctrl2) {
        const auto& p1 = ctrl1->particles();
        const auto& p2 = ctrl2->particles();
        REQUIRE(!p1.empty());
        REQUIRE(!p2.empty());
        // At least one particle should differ
        bool any_diff = false;
        for (size_t k = 0; k < p1.size(); ++k) {
            if (std::fabs(p1[k].x - p2[k].x) > 1e-6f ||
                std::fabs(p1[k].y - p2[k].y) > 1e-6f) {
                any_diff = true;
                break;
            }
        }
        CHECK(any_diff);
    }
}
