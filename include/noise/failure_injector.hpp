#pragma once

#include "sim/physics.hpp"

#include <array>
#include <string>
#include <vector>

namespace noise {

enum class FailureType {
    SensorDead,
    SensorStuck,
    OdomSpike,
    HeadingBias,
};

struct FailureEvent {
    int start_tick = 0;
    int duration_ticks = 1;
    FailureType type = FailureType::SensorDead;
    int sensor_idx = -1;
    double param = 0.0;
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

    std::vector<std::string> active_failures(int tick) const;

private:
    static bool is_active(const FailureEvent& e, int tick);

    std::vector<FailureEvent> events_;
    std::array<double, 4> stuck_values_{ -1.0, -1.0, -1.0, -1.0 };
    std::array<bool, 4> stuck_initialized_{ false, false, false, false };
};

} // namespace noise
