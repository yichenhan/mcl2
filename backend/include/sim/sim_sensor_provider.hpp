#pragma once

#include "mcl/pose_correction_controller.hpp"
#include "mcl/mcl_engine.hpp"
#include "noise/failure_injector.hpp"
#include "sim/sim_chassis.hpp"
#include "sim/sensor_model.hpp"

#include <vector>

namespace sim {

// ---------------------------------------------------------------------------
// SimSensorProvider
// ---------------------------------------------------------------------------
// Pull-based sensor provider for simulation.
// Implements mcl::ISensorProvider -- usable directly as input to
// PoseCorrectionController.
//
// On each getReadings() call:
//   1. Reads chassis_.ground_truth() for the true robot position
//   2. Ray-casts 4 distance sensors from truth (+ gaussian noise + dropout)
//   3. Applies sensor-only failures (dead, stuck, spurious) for current tick
//   4. Converts from inches to mm and returns DistanceReadings
//
// There is NO prepareReadings() step -- this eliminates temporal coupling.
// The caller manages timing by calling advanceTick() at the start of each
// simulation tick so failure events fire at the correct tick number.
// ---------------------------------------------------------------------------
class SimSensorProvider : public mcl::ISensorProvider {
public:
    SimSensorProvider(const SimChassis& chassis,
                      uint64_t seed,
                      const SensorNoiseConfig& noise_config,
                      const mcl::MCLConfig& sensor_geometry,
                      const std::vector<Obstacle>& obstacles);

    // mcl::ISensorProvider contract -- fully self-contained read.
    distance_loc::DistanceReadings getReadings() override;

    // Advance the internal tick counter.  Call once per simulation tick,
    // BEFORE correction_controller_.update(), so failure timing is correct.
    void advanceTick();
    int current_tick() const;

    void schedule_failure(const noise::FailureEvent& event);
    noise::FailureInjector& failure_injector();
    const noise::FailureInjector& failure_injector() const;

private:
    static distance_loc::DistanceReadings readings_to_mm(const std::array<double, 4>& raw);

    const SimChassis& chassis_;
    SensorModel sensor_model_;
    noise::FailureInjector failure_injector_;
    const std::vector<Obstacle>& obstacles_;
    const mcl::MCLConfig& sensor_geometry_;
    int tick_ = 0;
};

} // namespace sim
