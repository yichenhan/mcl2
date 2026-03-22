#include "pursuit/route_runner.hpp"

#include "nlohmann/json.hpp"

#include <algorithm>
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
    const double max_teleport_in = std::min(route.max_inches_odom_delta_per_tick, 12.0);
    cfg.mcl_gate_config.max_inches_odom_delta_per_tick = max_teleport_in;
    std::ostringstream sid;
    sid << "route_" << route.name << "_seed_" << route.failure_seed;
    cfg.replay_config.directory = replay_dir;
    cfg.replay_config.session_id = sid.str();

    state::SimSession session(cfg);
    for (const auto& f : build_failure_schedule(route, cfg.field.field_half)) {
        session.schedule_failure(f);
    }

    {
        nlohmann::json wp_json = nlohmann::json::array();
        for (const auto& w : route.waypoints) {
            wp_json.push_back({{"x", w.x}, {"y", w.y}});
        }
        nlohmann::json obs_json = nlohmann::json::array();
        for (const auto& o : route.obstacles) {
            obs_json.push_back({{"min_x", o.min_x}, {"min_y", o.min_y},
                                {"max_x", o.max_x}, {"max_y", o.max_y}});
        }
        nlohmann::json route_cfg = {
            {"route_name", route.name},
            {"description", route.description},
            {"max_ticks", route.max_ticks},
            {"max_inches_odom_delta_per_tick", max_teleport_in},
            {"pure_pursuit", {
                {"lookahead_distance", route.pure_pursuit.lookahead_distance},
                {"linear_velocity", route.pure_pursuit.linear_velocity},
                {"waypoint_tolerance", route.pure_pursuit.waypoint_tolerance},
            }},
        };
        session.mcl_controller().set_route_overlay(
            std::move(wp_json), std::move(obs_json), std::move(route_cfg));
        session.mcl_controller().set_flush_interval(50);
    }

    PurePursuit pursuit(route.pure_pursuit);
    sim::RobotState control_state = cfg.initial_state;
    mcl::Estimate accepted_estimate{
        static_cast<float>(control_state.x),
        static_cast<float>(control_state.y)
    };
    for (int i = 0; i < route.max_ticks; ++i) {
        session.mcl_controller().set_accepted_pose(accepted_estimate.x, accepted_estimate.y);
        const Command cmd = pursuit.compute(control_state, route.waypoints);
        const state::TickState tick = session.tick(cmd.linear_vel, cmd.angular_vel_deg);

        const auto decision = session.gate_estimate_for_control(accepted_estimate, tick);
        if (decision.accepted) {
            accepted_estimate = tick.post_resample.estimate;
            control_state.x = accepted_estimate.x;
            control_state.y = accepted_estimate.y;
        }
        session.mcl_controller().set_accepted_pose(accepted_estimate.x, accepted_estimate.y);
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

    result.replay_file = sid.str() + ".json";
    return result;
}

} // namespace pursuit

