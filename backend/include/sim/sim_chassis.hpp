#pragma once

#include "sim/field.hpp"
#include "sim/physics.hpp"
#include "sim/sensor_model.hpp"

namespace sim {

// ---------------------------------------------------------------------------
// SimChassis
// ---------------------------------------------------------------------------
// Simulated chassis implementing the getPose()/setPose() contract required
// by PoseCorrectionController<T>.
//
// Mirrors what a real LemLib Chassis does:
//   - physics_     = the real world (ground truth, not observable on robot)
//   - odom_state_  = what wheel encoders + IMU report (drifts from truth)
//
// Usage in SimHarness:
//   1. chassis_.step(vel)                    -- advance physics
//   2. chassis_.applyOdomNoise(delta, hdg)   -- integrate noisy odom
//   3. getPose() returns odom_state_ as floats
//   4. setPose(x, y, theta) resets odom origin (as LemLib does)
//   5. ground_truth() is sim-only debug access
//   6. teleport(target) for kidnap events
// ---------------------------------------------------------------------------
class SimChassis {
public:
    struct Pose { float x = 0, y = 0, theta = 0; };

    SimChassis(const Field& field,
               const PhysicsConfig& phys_config,
               const RobotState& initial_state);

    // PoseCorrectionController<T> contract
    Pose getPose() const;
    void setPose(float x, float y, float theta);

    // Advance physics (moves ground truth; does NOT update odom).
    // Caller must call applyOdomNoise() afterwards to integrate odom.
    StepResult step(Action action);
    StepResult step(double linear_vel, double angular_vel_deg);

    // Integrate a (post-failure, post-noise) motion delta into odom_state_.
    // heading_deg is the final heading to write directly (not accumulated),
    // matching how an IMU would report heading to LemLib.
    void applyOdomNoise(const MotionDelta& delta, double heading_deg);

    // Teleport ground truth for kidnap events (sim-only).
    void teleport(const RobotState& target);

    // Sim-only: true physics state -- not available on a real robot.
    RobotState ground_truth() const;

private:
    static double wrap_heading(double deg);

    Physics physics_;
    Field field_;
    RobotState odom_state_{};
};

} // namespace sim
