#pragma once

#include <atomic>

#include "distance_localization.hpp"
#include "mcl/localization_controller.hpp"

namespace mcl {

// ---------------------------------------------------------------------------
// ISensorProvider
// ---------------------------------------------------------------------------
// Abstract interface for reading distance sensors.
// In simulation: implemented by SimSensorProvider.
// On a real robot: wrap VEX Distance sensors in a subclass.
// ---------------------------------------------------------------------------
class ISensorProvider {
public:
    virtual ~ISensorProvider() = default;
    virtual distance_loc::DistanceReadings getReadings() = 0;
};

// ---------------------------------------------------------------------------
// CorrectionResult
// ---------------------------------------------------------------------------
// Returned by PoseCorrectionController::update().
// Contains everything needed to build a TickState without reaching into
// MCL internals, plus diagnostic data for analytics and logging.
// ---------------------------------------------------------------------------
struct CorrectionResult {
    TickOutput mcl_output{};           // full MCL pipeline output (particles, gate, estimates)
    Pose chassis_pose{};               // corrected pose on the chassis after this tick
    Pose raw_odom{};                   // raw odom fed into MCL (offset subtracted from chassis)
    bool correction_applied = false;   // true if the gate accepted and setPose was called
    double correction_distance_in = 0.0; // euclidean distance of the accepted correction jump
};

// ---------------------------------------------------------------------------
// PoseCorrectionController<T>
// ---------------------------------------------------------------------------
// Template class that bridges any chassis T to the MCL localization system.
//
// Requirements on T:
//   - nested type T::Pose with fields (x, y, theta) convertible to float
//   - T::getPose() const -> T::Pose
//   - T::setPose(float x, float y, float theta) -> void
//
// Usage pattern (PROS / LemLib):
//   PoseCorrectionController<lemlib::Chassis> pcc(chassis, sensors, config);
//   // In a background task at 50 Hz:
//   pcc.update();
//
// The controller:
//   1. Reads chassis odom pose (drifts over time)
//   2. Subtracts accumulated correction offset to recover raw odom for MCL
//   3. Reads sensors via ISensorProvider
//   4. Feeds raw odom + sensors + heading to LocalizationController
//   5. If gate accepts: applies correction to chassis X/Y (never heading)
//   6. Returns CorrectionResult with all diagnostic data
// ---------------------------------------------------------------------------
template <typename T>
class PoseCorrectionController {
public:
    PoseCorrectionController(T& chassis,
                             ISensorProvider& sensor_provider,
                             const ControllerConfig& config)
        : chassis_(chassis),
          sensor_provider_(sensor_provider),
          controller_(config),
          odom_offset_{0.0, 0.0, 0.0}
    {}

    // Run one MCL tick and apply correction if gate accepts.
    // Call this at a fixed rate (e.g. every 20-50ms).
    CorrectionResult update() {
        const auto cp = chassis_.getPose();
        const double raw_x = static_cast<double>(cp.x) - odom_offset_.x;
        const double raw_y = static_cast<double>(cp.y) - odom_offset_.y;
        const double heading = static_cast<double>(cp.theta);

        TickInput input;
        input.sensors = sensor_provider_.getReadings();
        input.odom_pose.x = raw_x;
        input.odom_pose.y = raw_y;
        input.odom_pose.theta = heading;
        input.imu_heading_deg = heading;

        const TickOutput out = controller_.tick(input);

        CorrectionResult result;
        result.mcl_output = out;
        result.raw_odom = {raw_x, raw_y, heading};
        result.correction_applied = out.gate.accepted;
        result.correction_distance_in = out.correction_distance_in;

        if (out.gate.accepted && corrections_enabled_.load(std::memory_order_relaxed)) {
            // Only correct X/Y. Heading is trusted from odom/IMU directly.
            odom_offset_.x = out.accepted_pose.x - raw_x;
            odom_offset_.y = out.accepted_pose.y - raw_y;
            chassis_.setPose(
                static_cast<float>(out.accepted_pose.x),
                static_cast<float>(out.accepted_pose.y),
                cp.theta);  // heading unchanged
        }

        const auto final_cp = chassis_.getPose();
        result.chassis_pose = {
            static_cast<double>(final_cp.x),
            static_cast<double>(final_cp.y),
            static_cast<double>(final_cp.theta)
        };
        result.mcl_output.chassis_pose = result.chassis_pose;

        return result;
    }

    // Access the underlying LocalizationController for diagnostics and
    // existing code that needs MCL internals.
    LocalizationController& controller() { return controller_; }
    const LocalizationController& controller() const { return controller_; }

    // Convenience accessors that delegate to the controller.
    Pose get_accepted_pose() const { return controller_.get_accepted_pose(); }
    Pose get_raw_estimate() const  { return controller_.get_raw_estimate(); }

    void set_corrections_enabled(bool enabled) {
        corrections_enabled_.store(enabled, std::memory_order_relaxed);
    }
    bool corrections_enabled() const {
        return corrections_enabled_.load(std::memory_order_relaxed);
    }

    // Expose accumulated odom offset for testing.
    const Pose& odom_offset() const { return odom_offset_; }

private:
    T& chassis_;
    ISensorProvider& sensor_provider_;
    LocalizationController controller_;
    Pose odom_offset_;
    std::atomic<bool> corrections_enabled_{true};
};

} // namespace mcl
