#pragma once

#include "mcl/localization_controller.hpp"
#include "noise/failure_injector.hpp"
#include "sim/physics.hpp"
#include "sim/sensor_model.hpp"
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

    mcl::LocalizationController& controller();
    const mcl::LocalizationController& controller() const;

private:
    static double wrap_heading(double deg);
    static double euclidean(double x0, double y0, double x1, double y1);
    static distance_loc::DistanceReadings readings_to_mm(const std::array<double, 4>& readings);
    static state::MCLSnapshot to_state_snapshot(const mcl::PhaseSnapshot& snap);

    state::TickState process_step(const StepResult& step_result);
    std::array<double, 4> read_sensors(const RobotState& state);

    Config config_{};
    Physics physics_;
    SensorModel sensor_model_;
    mcl::LocalizationController controller_;
    noise::FailureInjector failure_injector_;

    int tick_ = 0;
    RobotState odom_state_{};
    std::vector<state::TickState> history_;
};

} // namespace sim
