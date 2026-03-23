#include "pursuit/route_runner.hpp"

#include "nlohmann/json.hpp"
#include "state/session_recorder.hpp"

#include <cstdio>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace pursuit {

namespace {

bool write_mcl_replay_atomic(
    const std::string& directory,
    const std::string& file_stem,
    const sim::Field& field,
    const std::vector<mcl::MCLTickResult>& ticks) {
    if (directory.empty()) return false;
    std::error_code ec;
    std::filesystem::create_directories(directory, ec);
    if (ec) return false;

    nlohmann::json j;
    j["session_id"] = file_stem;
    j["total_ticks"] = ticks.size();
    j["field_half"] = field.field_half;
    j["obstacles"] = nlohmann::json::array();
    for (const auto& o : field.obstacles) {
        j["obstacles"].push_back({
            {"min_x", o.min_x},
            {"min_y", o.min_y},
            {"max_x", o.max_x},
            {"max_y", o.max_y},
        });
    }
    j["ticks"] = ticks;

    const std::string final_path = directory + "/" + file_stem + ".json";
    const std::string tmp_path = final_path + ".tmp";
    std::ofstream out(tmp_path, std::ios::trunc);
    if (!out.is_open()) return false;
    out << j.dump(2);
    out.close();
    if (!out) return false;
    if (std::rename(tmp_path.c_str(), final_path.c_str()) != 0) {
        std::remove(tmp_path.c_str());
        return false;
    }
    return true;
}

} // namespace

RouteResult RouteRunner::run(
    const RouteDefinition& route,
    const std::string& replay_dir,
    const std::string& mcl_replay_dir) {
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

    WaypointFollower follower(route.follower);
    sim::RobotState control_state = cfg.initial_state;
    for (int i = 0; i < route.max_ticks; ++i) {
        // Use estimated pose for control input to mimic real robots:
        // no direct access to ground truth during route following.
        const Command cmd = follower.compute(control_state, route.waypoints);
        const state::TickState tick = session.tick(cmd.linear_vel, cmd.angular_vel_deg);
        // Follow the latest MCL estimate directly for control state. The
        // session already applies MCL gating/noise handling at source.
        control_state.x = tick.post_resample.estimate.x;
        control_state.y = tick.post_resample.estimate.y;
        if (std::isfinite(tick.observed_heading)) {
            control_state.heading_deg = tick.observed_heading;
        }

        result.waypoints_reached = follower.current_waypoint_index();
        if (follower.is_complete()) break;
    }

    const auto& history = session.history();
    result.total_ticks = static_cast<int>(history.size());
    result.completed = follower.is_complete();
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
        {"follower", {
            {"linear_velocity", route.follower.linear_velocity},
            {"waypoint_tolerance", route.follower.waypoint_tolerance},
            {"max_angular_velocity_deg", route.follower.max_angular_velocity_deg},
            {"turn_in_place_threshold_deg", route.follower.turn_in_place_threshold_deg},
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
    write_mcl_replay_atomic(mcl_replay_dir, sid.str(), cfg.field, session.mcl_history());
    return result;
}

} // namespace pursuit

