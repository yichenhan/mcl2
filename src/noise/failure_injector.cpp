#include "noise/failure_injector.hpp"

#include <algorithm>
#include <sstream>

namespace noise {

namespace {
constexpr double kInvalidReading = -1.0;
}

bool FailureInjector::is_active(const FailureEvent& e, int tick) {
    if (e.duration_ticks <= 0) return false;
    return tick >= e.start_tick && tick < (e.start_tick + e.duration_ticks);
}

void FailureInjector::schedule(const FailureEvent& event) {
    events_.push_back(event);
}

void FailureInjector::clear() {
    events_.clear();
    stuck_values_ = { kInvalidReading, kInvalidReading, kInvalidReading, kInvalidReading };
    stuck_initialized_ = { false, false, false, false };
}

void FailureInjector::apply(
    int tick,
    std::array<double, 4>& readings,
    sim::MotionDelta& odom_delta,
    double& heading_deg) {
    for (const auto& e : events_) {
        if (!is_active(e, tick)) continue;

        switch (e.type) {
            case FailureType::SensorDead:
                if (e.sensor_idx >= 0 && e.sensor_idx < static_cast<int>(readings.size())) {
                    readings[static_cast<size_t>(e.sensor_idx)] = kInvalidReading;
                }
                break;
            case FailureType::SensorStuck:
                if (e.sensor_idx >= 0 && e.sensor_idx < static_cast<int>(readings.size())) {
                    const size_t idx = static_cast<size_t>(e.sensor_idx);
                    if (!stuck_initialized_[idx]) {
                        stuck_values_[idx] = readings[idx];
                        stuck_initialized_[idx] = true;
                    }
                    readings[idx] = stuck_values_[idx];
                }
                break;
            case FailureType::OdomSpike:
                odom_delta.forward_in *= e.param;
                odom_delta.lateral_in *= e.param;
                odom_delta.rotation_deg *= e.param;
                break;
            case FailureType::HeadingBias:
                heading_deg += e.param;
                break;
            case FailureType::SpuriousReflection:
                if (e.sensor_idx >= 0 && e.sensor_idx < static_cast<int>(readings.size())) {
                    readings[static_cast<size_t>(e.sensor_idx)] = e.param;
                }
                break;
            case FailureType::Kidnap:
                // Handled by pending_kidnap() in SimSession tick orchestration.
                break;
        }
    }

    for (int i = 0; i < static_cast<int>(readings.size()); ++i) {
        bool still_stuck = false;
        for (const auto& e : events_) {
            if (e.type == FailureType::SensorStuck && e.sensor_idx == i && is_active(e, tick)) {
                still_stuck = true;
                break;
            }
        }
        if (!still_stuck) {
            stuck_initialized_[static_cast<size_t>(i)] = false;
            stuck_values_[static_cast<size_t>(i)] = kInvalidReading;
        }
    }
}

std::optional<sim::RobotState> FailureInjector::pending_kidnap(int tick) const {
    for (const auto& e : events_) {
        if (e.type != FailureType::Kidnap) continue;
        if (e.start_tick != tick) continue;
        sim::RobotState target{};
        target.x = e.target_x;
        target.y = e.target_y;
        target.heading_deg = 0.0;
        return target;
    }
    return std::nullopt;
}

std::vector<std::string> FailureInjector::active_failures(int tick) const {
    std::vector<std::string> out;
    for (const auto& e : events_) {
        if (!is_active(e, tick)) continue;
        std::ostringstream ss;
        ss << "tick=" << e.start_tick << " ";
        switch (e.type) {
            case FailureType::SensorDead:
                ss << "sensor_dead";
                break;
            case FailureType::SensorStuck:
                ss << "sensor_stuck";
                break;
            case FailureType::OdomSpike:
                ss << "odom_spike";
                break;
            case FailureType::HeadingBias:
                ss << "heading_bias";
                break;
            case FailureType::Kidnap:
                ss << "kidnap";
                break;
            case FailureType::SpuriousReflection:
                ss << "spurious_reflection";
                break;
        }
        if (e.sensor_idx >= 0) ss << " sensor=" << e.sensor_idx;
        if (e.type == FailureType::Kidnap) {
            ss << " target_x=" << e.target_x << " target_y=" << e.target_y;
        }
        ss << " param=" << e.param;
        out.push_back(ss.str());
    }
    return out;
}

} // namespace noise
