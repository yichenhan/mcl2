#pragma once

#include "sim/field.hpp"

namespace sim {

enum class Action { FORWARD, BACKWARD, ROTATE_CW, ROTATE_CCW, NONE };

struct RobotState {
    double x = 0.0;
    double y = 0.0;
    double heading_deg = 0.0;
};

struct MotionDelta {
    double forward_in = 0.0;
    double lateral_in = 0.0;
    double rotation_deg = 0.0;
};

struct PhysicsConfig {
    double max_velocity = 36.0;     // in/s
    double max_angular_vel = 360.0; // deg/s
    double dt = 0.05;               // tick duration (seconds)
};

struct StepResult {
    MotionDelta delta;
    bool colliding = false;
};

class Physics {
public:
    Physics(const Field& field, const PhysicsConfig& config = {});
    StepResult step(Action action);
    const RobotState& state() const;
    void set_state(const RobotState& s);
private:
    Field field_;
    PhysicsConfig config_;
    RobotState state_;
};

} // namespace sim
