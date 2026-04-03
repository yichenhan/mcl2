#pragma once

#include "mcl/localization_controller.hpp"
#include "mcl/pose_correction_controller.hpp"
#include "noise/failure_injector.hpp"
#include "sim/physics.hpp"
#include "sim/sensor_model.hpp"
#include "sim/sim_chassis.hpp"
#include "sim/sim_sensor_provider.hpp"
#include "state/tick_state.hpp"

#include <vector>

namespace sim {

class SimHarness {
public:
    struct Config {
        mcl::ControllerConfig controller_config{};
        PhysicsConfig physics_config{};
        OdomNoiseConfig odom_noise{};
        SensorNoiseConfig sensor_noise{};
        RobotState initial_state{};
        int max_ticks = 0;
    };

    explicit SimHarness(const Config& config);

    state::TickState tick(Action action);
    state::TickState tick(double linear_vel, double angular_vel_deg);

    const Config& config() const;
    const std::vector<state::TickState>& history() const;
    int current_tick() const;

    void schedule_failure(const noise::FailureEvent& event);
    const noise::FailureInjector& failure_injector() const;
    mcl::GateDecision gate_estimate_for_control(const mcl::Estimate& prev_accepted, const state::TickState& tick) const;

    // Delegate to correction controller's underlying LocalizationController.
    mcl::LocalizationController& controller();
    const mcl::LocalizationController& controller() const;

    // Exposed for readings_to_mm re-use in SimSensorProvider.
    static distance_loc::DistanceReadings readings_to_mm(const std::array<double, 4>& readings);

private:
    static double wrap_heading(double deg);
    static double euclidean(double x0, double y0, double x1, double y1);
    static state::MCLSnapshot to_state_snapshot(const mcl::PhaseSnapshot& snap);

    state::TickState process_step(const StepResult& step_result);

    Config config_{};

    // Physics-level odom noise and collision stall -- kept separate from
    // sensor_provider_ so their RNGs don't share state.
    SensorModel sensor_model_;

    // New chassis-centric components
    SimChassis chassis_;
    SimSensorProvider sensor_provider_;
    mcl::PoseCorrectionController<SimChassis> correction_controller_;

    int tick_ = 0;
    std::vector<state::TickState> history_;
};

} // namespace sim
