#include "sim/sim_sensor_provider.hpp"

#include "ray/ray_cast_obstacles.hpp"

#include <cmath>

namespace sim {

namespace {
constexpr double kInToMm = 25.4;
} // namespace

SimSensorProvider::SimSensorProvider(const SimChassis& chassis,
                                     uint64_t seed,
                                     const SensorNoiseConfig& noise_config,
                                     const mcl::MCLConfig& sensor_geometry,
                                     const std::vector<Obstacle>& obstacles)
    : chassis_(chassis),
      sensor_model_(seed, OdomNoiseConfig{}, noise_config),
      obstacles_(obstacles),
      sensor_geometry_(sensor_geometry) {}

distance_loc::DistanceReadings SimSensorProvider::getReadings() {
    const RobotState truth = chassis_.ground_truth();
    const distance_loc::Vec2 pos{truth.x, truth.y};

    std::array<double, 4> raw{};
    for (int i = 0; i < 4; ++i) {
        raw[static_cast<size_t>(i)] = sensor_model_.observe_distance_sensor(
            pos,
            truth.heading_deg,
            sensor_geometry_.sensors[i].offset,
            sensor_geometry_.sensors[i].angle_deg,
            i,
            obstacles_);
    }

    // Apply sensor-only failures (dead, stuck, spurious) for this tick.
    failure_injector_.applySensors(tick_, raw);

    return readings_to_mm(raw);
}

void SimSensorProvider::advanceTick() {
    tick_++;
}

int SimSensorProvider::current_tick() const {
    return tick_;
}

void SimSensorProvider::schedule_failure(const noise::FailureEvent& event) {
    failure_injector_.schedule(event);
}

noise::FailureInjector& SimSensorProvider::failure_injector() {
    return failure_injector_;
}

const noise::FailureInjector& SimSensorProvider::failure_injector() const {
    return failure_injector_;
}

distance_loc::DistanceReadings SimSensorProvider::readings_to_mm(const std::array<double, 4>& raw) {
    const auto to_mm = [](double v) -> std::optional<int32_t> {
        if (v < 0.0 || !std::isfinite(v)) return std::nullopt;
        return static_cast<int32_t>(std::lround(v * kInToMm));
    };
    distance_loc::DistanceReadings out;
    out.left  = to_mm(raw[0]);
    out.right = to_mm(raw[1]);
    out.front = to_mm(raw[2]);
    out.back  = to_mm(raw[3]);
    return out;
}

} // namespace sim
