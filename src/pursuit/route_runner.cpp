#include "pursuit/route_runner.hpp"

#include "nlohmann/json.hpp"
#include "state/session_recorder.hpp"

#include <cmath>
#include <sstream>

namespace pursuit {

RouteResult RouteRunner::run(const RouteDefinition& route, const std::string& replay_dir) {
    RouteResult result;
    result.route_name = route.name;
    result.seed = route.failure_seed;

    state::SimSessionConfig cfg;
    cfg.seed = route.failure_seed;
    cfg.initial_state = {route.initial_position.x, route.initial_position.y, route.initial_heading_deg};
    cfg.field.obstacles = route.obstacles;
    cfg.mcl_gate_config.max_estimate_speed_ft_per_s = route.max_estimate_speed_ft_per_s;

    state::SimSession session(cfg);
    for (const auto& f : build_failure_schedule(route, cfg.field.field_half)) {
        session.schedule_failure(f);
    }

    PurePursuit pursuit(route.pure_pursuit);
    sim::RobotState control_state = cfg.initial_state;
    mcl::Estimate accepted_estimate{
        static_cast<float>(control_state.x),
        static_cast<float>(control_state.y)
    };
    for (int i = 0; i < route.max_ticks; ++i) {
        // Use estimated pose for control input to mimic real robots:
        // no direct access to ground truth during route following.
        const Command cmd = pursuit.compute(control_state, route.waypoints);
        const state::TickState tick = session.tick(cmd.linear_vel, cmd.angular_vel_deg);

        const auto decision = session.gate_estimate_for_control(accepted_estimate, tick);
        if (decision.accepted) {
            accepted_estimate = tick.post_resample.estimate;
            control_state.x = accepted_estimate.x;
            control_state.y = accepted_estimate.y;
        }
        if (std::isfinite(tick.observed_heading)) {
            control_state.heading_deg = tick.observed_heading;
        }

        result.waypoints_reached = pursuit.current_waypoint_index();
        if (pursuit.is_complete()) break;
    }

    const auto& history = session.history();
    result.total_ticks = static_cast<int>(history.size());
    result.completed = pursuit.is_complete();
    if (!history.empty()) {
        result.final_mcl_error = history.back().mcl_error;
        double sum = 0.0;
        for (const auto& t : history) sum += t.mcl_error;
        result.mean_mcl_error = sum / static_cast<double>(history.size());
    }

    std::ostringstream sid;
    sid << "route_" << route.name << "_seed_" << route.failure_seed;
    state::SessionRecorder recorder(replay_dir, sid.str());

    nlohmann::json cfg_json = {
        {"seed", route.failure_seed},
        {"route_name", route.name},
        {"description", route.description},
        {"max_ticks", route.max_ticks},
        {"max_estimate_speed_ft_per_s", route.max_estimate_speed_ft_per_s},
        {"pure_pursuit", {
            {"lookahead_distance", route.pure_pursuit.lookahead_distance},
            {"linear_velocity", route.pure_pursuit.linear_velocity},
            {"waypoint_tolerance", route.pure_pursuit.waypoint_tolerance},
        }},
    };
    nlohmann::json waypoints = nlohmann::json::array();
    for (const auto& w : route.waypoints) {
        waypoints.push_back({{"x", w.x}, {"y", w.y}});
    }
    cfg_json["waypoints"] = waypoints;
    recorder.set_config(cfg_json);

    nlohmann::json obstacles = nlohmann::json::array();
    for (const auto& o : route.obstacles) {
        obstacles.push_back({{"min_x", o.min_x}, {"min_y", o.min_y}, {"max_x", o.max_x}, {"max_y", o.max_y}});
    }
    recorder.set_obstacles(obstacles);
    for (const auto& t : history) recorder.record(t);
    if (recorder.write_atomic()) {
        result.replay_file = sid.str() + ".json";
    }
    return result;
}

} // namespace pursuit

