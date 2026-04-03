#pragma once

#include "sim/physics.hpp"

#include <array>
#include <optional>
#include <string>
#include <vector>

namespace noise {

enum class FailureType {
    SensorDead,
    SensorStuck,
    OdomSpike,
    HeadingBias,
    Kidnap,
    SpuriousReflection,
};

struct FailureEvent {
    int start_tick = 0;
    int duration_ticks = 1;
    FailureType type = FailureType::SensorDead;
    int sensor_idx = -1;
    double param = 0.0;
    double target_x = 0.0;
    double target_y = 0.0;
};

class FailureInjector {
public:
    void schedule(const FailureEvent& event);
    void clear();

    void apply(
        int tick,
        std::array<double, 4>& readings,
        sim::MotionDelta& odom_delta,
        double& heading_deg
    );

    // Split API: apply odom-related failures (OdomSpike, HeadingBias) BEFORE
    // chassis odom integration. Kidnap is still handled via pending_kidnap().
    void applyOdom(int tick, sim::MotionDelta& odom_delta, double& heading_deg);

    // Split API: apply sensor-related failures (SensorDead, SensorStuck,
    // SpuriousReflection) inside SimSensorProvider::getReadings().
    void applySensors(int tick, std::array<double, 4>& readings);

    std::optional<sim::RobotState> pending_kidnap(int tick) const;

    std::vector<std::string> active_failures(int tick) const;

private:
    static bool is_active(const FailureEvent& e, int tick);

    std::vector<FailureEvent> events_;
    std::array<double, 4> stuck_values_{ -1.0, -1.0, -1.0, -1.0 };
    std::array<bool, 4> stuck_initialized_{ false, false, false, false };
};

} // namespace noise
