#pragma once

#include "mcl/mcl_controller.hpp"
#include "noise/failure_injector.hpp"
#include "sim/field.hpp"
#include "sim/physics.hpp"
#include "sim/sensor_model.hpp"
#include "state/tick_state.hpp"

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace state {

struct SimSessionConfig {
    uint64_t seed = 42;
    mcl::MCLConfig mcl_config{};
    sim::PhysicsConfig physics_config{};
    sim::OdomNoiseConfig odom_noise_config{};
    sim::SensorNoiseConfig sensor_noise_config{};
    sim::RobotState initial_state{};
    sim::Field field{};
    int min_sensors_for_update = 2;
    double pose_gate_radius_90 = 20.0;
    mcl::GateConfig mcl_gate_config{};
};

class SimSession {
public:
    explicit SimSession(const SimSessionConfig& config = {});

    TickState tick(sim::Action action);
    TickState tick(double linear_vel, double angular_vel_deg);

    const SimSessionConfig& config() const;
    const std::vector<TickState>& history() const;
    int current_tick() const;

    void schedule_failure(const noise::FailureEvent& event);
    const noise::FailureInjector& failure_injector() const;
    mcl::GateDecision gate_estimate_for_control(const mcl::Estimate& prev_accepted, const TickState& tick) const;

private:
    static double wrap_heading(double deg);
    static double euclidean(double x0, double y0, double x1, double y1);

    TickState process_step(const sim::StepResult& step_result);
    std::array<double, 4> read_sensors(const sim::RobotState& s);
    MCLSnapshot snapshot_from_mcl() const;

    SimSessionConfig config_;
    sim::Physics physics_;
    sim::SensorModel sensor_model_;
    mcl::MCLController mcl_;
    noise::FailureInjector failure_injector_;

    int tick_ = 0;
    sim::RobotState odom_state_{};
    std::vector<TickState> history_;
};

std::string action_to_string(sim::Action action);
bool action_from_string(const std::string& value, sim::Action& action);

} // namespace state
