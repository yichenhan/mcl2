#include "server/api.hpp"

#include "noise/failure_config.hpp"
#include "noise/failure_injector.hpp"
#include "state/replay_loader.hpp"

#include "nlohmann/json.hpp"

#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <unordered_set>

namespace server {

namespace {
bool is_safe_filename(const std::string& name) {
    return !name.empty() && name.find("..") == std::string::npos && name.find('/') == std::string::npos;
}

/** Stress CLI writes here; server also lists this alongside `replay_dir_` for live sessions. */
constexpr const char* kStressReplayDir = "replays/stress";

int hex_nibble(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

/** Decode %XX in a single path segment (e.g. from encodeURIComponent). */
std::string url_decode_percent(const std::string& in) {
    std::string out;
    out.reserve(in.size());
    for (size_t i = 0; i < in.size(); ++i) {
        if (in[i] == '%' && i + 2 < in.size()) {
            const int hi = hex_nibble(in[i + 1]);
            const int lo = hex_nibble(in[i + 2]);
            if (hi >= 0 && lo >= 0) {
                out.push_back(static_cast<char>((hi << 4) | lo));
                i += 2;
                continue;
            }
        }
        out.push_back(in[i]);
    }
    return out;
}

void append_json_files_in_dir(
    const std::string& dir,
    std::vector<std::string>& out,
    const std::unordered_set<std::string>* exclude_basenames) {
    std::error_code ec;
    if (!std::filesystem::exists(dir, ec)) return;
    for (const auto& entry : std::filesystem::directory_iterator(dir, ec)) {
        if (ec) break;
        if (!entry.is_regular_file()) continue;
        if (entry.path().extension() != ".json") continue;
        const std::string base = entry.path().filename().string();
        if (exclude_basenames && exclude_basenames->count(base) > 0) continue;
        out.push_back((std::filesystem::path(dir) / base).generic_string());
    }
}

std::vector<std::string> list_replay_paths_merged(const std::string& primary_replay_dir) {
    std::vector<std::string> out;
    append_json_files_in_dir(primary_replay_dir, out, nullptr);
    static const std::unordered_set<std::string> kStressExclude = { "summary_report.json" };
    append_json_files_in_dir(kStressReplayDir, out, &kStressExclude);
    std::sort(out.begin(), out.end());
    return out;
}

/** Allow `name.json` or `primary/foo.json` or `replays/stress/foo.json` (no .. or absolute). */
bool is_safe_replay_path_id(const std::string& id, const std::string& primary_replay_dir) {
    if (id.empty() || id.find("..") != std::string::npos) return false;
    if (!id.empty() && (id[0] == '/' || id[0] == '\\')) return false;
    if (id.find('\\') != std::string::npos) return false;
    if (id.find('/') == std::string::npos) return is_safe_filename(id);
    const std::string stress_prefix = std::string(kStressReplayDir) + "/";
    if (id.rfind(stress_prefix, 0) == 0) {
        return is_safe_filename(id.substr(stress_prefix.size()));
    }
    const std::string prim_prefix = primary_replay_dir + "/";
    if (id.rfind(prim_prefix, 0) == 0) {
        return is_safe_filename(id.substr(prim_prefix.size()));
    }
    return false;
}

std::string replay_file_path_on_disk(const std::string& id, const std::string& primary_replay_dir) {
    if (id.find('/') == std::string::npos) return primary_replay_dir + "/" + id;
    return id;
}

noise::FailureType failure_type_from_string(const std::string& v) {
    if (v == "sensor_stuck") return noise::FailureType::SensorStuck;
    if (v == "odom_spike") return noise::FailureType::OdomSpike;
    if (v == "heading_bias") return noise::FailureType::HeadingBias;
    if (v == "kidnap") return noise::FailureType::Kidnap;
    if (v == "spurious_reflection") return noise::FailureType::SpuriousReflection;
    return noise::FailureType::SensorDead;
}

bool action_from_string(const std::string& value, sim::Action& action) {
    if (value == "forward") {
        action = sim::Action::FORWARD;
        return true;
    }
    if (value == "backward") {
        action = sim::Action::BACKWARD;
        return true;
    }
    if (value == "rotate_cw") {
        action = sim::Action::ROTATE_CW;
        return true;
    }
    if (value == "rotate_ccw") {
        action = sim::Action::ROTATE_CCW;
        return true;
    }
    if (value == "none") {
        action = sim::Action::NONE;
        return true;
    }
    return false;
}

void maybe_read_range(const nlohmann::json& body, const char* key, pursuit::Range& range) {
    if (!body.contains(key)) return;
    const auto& v = body[key];
    if (v.is_array() && v.size() >= 2 && v[0].is_number() && v[1].is_number()) {
        range.min = v[0].get<double>();
        range.max = v[1].get<double>();
        return;
    }
    if (v.is_object()) {
        range.min = v.value("min", range.min);
        range.max = v.value("max", range.max);
    }
}

nlohmann::json default_maps() {
    nlohmann::json maps = nlohmann::json::array();
    maps.push_back({
        {"name", "Empty Field"},
        {"obstacles", nlohmann::json::array()}
    });
    maps.push_back({
        {"name", "Center Block"},
        {"obstacles", nlohmann::json::array({
            nlohmann::json{
                {"type", "rect"},
                {"min_x", -12.0},
                {"min_y", -12.0},
                {"max_x", 12.0},
                {"max_y", 12.0},
            }
        })}
    });
    maps.push_back({
        {"name", "Obstacle Slalom"},
        {"obstacles", nlohmann::json::array({
            nlohmann::json{{"type", "rect"}, {"min_x", -42.0}, {"min_y", 20.0}, {"max_x", -24.0}, {"max_y", 34.0}},
            nlohmann::json{{"type", "rect"}, {"min_x", -10.0}, {"min_y", -8.0}, {"max_x", 8.0}, {"max_y", 8.0}},
            nlohmann::json{{"type", "rect"}, {"min_x", 24.0}, {"min_y", -34.0}, {"max_x", 42.0}, {"max_y", -20.0}}
        })}
    });
    maps.push_back({
        {"name", "VEX Object Map"},
        {"obstacles", nlohmann::json::array({
            // Approximate VEX-object-style obstacle arrangement for field testing.
            nlohmann::json{{"type", "rect"}, {"min_x", -40.0}, {"min_y", 26.0}, {"max_x", -28.0}, {"max_y", 38.0}},
            nlohmann::json{{"type", "rect"}, {"min_x", 28.0}, {"min_y", 26.0}, {"max_x", 40.0}, {"max_y", 38.0}},
            nlohmann::json{{"type", "rect"}, {"min_x", -40.0}, {"min_y", -38.0}, {"max_x", -28.0}, {"max_y", -26.0}},
            nlohmann::json{{"type", "rect"}, {"min_x", 28.0}, {"min_y", -38.0}, {"max_x", 40.0}, {"max_y", -26.0}},
            nlohmann::json{{"type", "rect"}, {"min_x", -9.0}, {"min_y", 16.0}, {"max_x", 9.0}, {"max_y", 24.0}},
            nlohmann::json{{"type", "rect"}, {"min_x", -9.0}, {"min_y", -24.0}, {"max_x", 9.0}, {"max_y", -16.0}},
            nlohmann::json{{"type", "circle"}, {"cx", -20.0}, {"cy", 0.0}, {"radius", 5.0}},
            nlohmann::json{{"type", "circle"}, {"cx", 20.0}, {"cy", 0.0}, {"radius", 5.0}},
            nlohmann::json{{"type", "circle"}, {"cx", 0.0}, {"cy", 0.0}, {"radius", 4.5}},
            nlohmann::json{{"type", "rect"}, {"min_x", -3.5}, {"min_y", 33.0}, {"max_x", 3.5}, {"max_y", 42.0}},
            nlohmann::json{{"type", "rect"}, {"min_x", -3.5}, {"min_y", -42.0}, {"max_x", 3.5}, {"max_y", -33.0}}
        })}
    });
    return maps;
}
} // namespace

SimServer::SimServer(int port, std::string replay_dir)
    : port_(port), replay_dir_(std::move(replay_dir)) {
    setup_routes();
}

void SimServer::set_common_headers(httplib::Response& res) {
    res.set_header("Access-Control-Allow-Origin", "*");
    res.set_header("Access-Control-Allow-Methods", "GET,POST,DELETE,OPTIONS");
    res.set_header("Access-Control-Allow-Headers", "Content-Type");
}

nlohmann::json SimServer::json_error(const std::string& message) {
    return nlohmann::json{ { "error", message } };
}

void SimServer::setup_routes() {
    server_.set_post_routing_handler([](const httplib::Request&, httplib::Response& res) {
        set_common_headers(res);
    });

    server_.Options(R"(.*)", [](const httplib::Request&, httplib::Response& res) {
        set_common_headers(res);
        res.status = 204;
    });

    server_.Get("/api/health", [](const httplib::Request&, httplib::Response& res) {
        set_common_headers(res);
        res.set_content(nlohmann::json({ { "status", "ok" } }).dump(), "application/json");
    });

    server_.Get("/api/maps", [](const httplib::Request&, httplib::Response& res) {
        set_common_headers(res);
        res.set_content(default_maps().dump(), "application/json");
    });

    server_.Post("/api/session/start", [this](const httplib::Request& req, httplib::Response& res) {
        nlohmann::json body = nlohmann::json::object();
        if (!req.body.empty()) {
            body = nlohmann::json::parse(req.body, nullptr, false);
            if (body.is_discarded()) {
                res.status = 400;
                res.set_content(json_error("invalid json").dump(), "application/json");
                return;
            }
        }

        sim::SimHarness::Config cfg;
        cfg.controller_config.seed = body.value("seed", cfg.controller_config.seed);
        if (body.contains("num_particles")) {
            cfg.controller_config.mcl_config.num_particles = body["num_particles"].get<int>();
        }
        if (body.contains("field_half")) {
            cfg.controller_config.field.field_half = body["field_half"].get<double>();
            cfg.controller_config.mcl_config.field_half = cfg.controller_config.field.field_half;
        }
        if (body.contains("initial_state") && body["initial_state"].is_object()) {
            cfg.initial_state.x = body["initial_state"].value("x", cfg.initial_state.x);
            cfg.initial_state.y = body["initial_state"].value("y", cfg.initial_state.y);
            cfg.initial_state.heading_deg = body["initial_state"].value("heading_deg", cfg.initial_state.heading_deg);
        }
        if (body.contains("max_ticks") && body["max_ticks"].is_number_integer()) {
            cfg.max_ticks = body["max_ticks"].get<int>();
        }

        if (body.contains("obstacles") && body["obstacles"].is_array()) {
            cfg.controller_config.field.obstacles.clear();
            for (const auto& o : body["obstacles"]) {
                cfg.controller_config.field.obstacles.push_back(o.get<sim::Obstacle>());
            }
        }

        std::string session_id;
        std::string replay_name;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            std::ostringstream ss;
            ss << "s" << next_session_id_++;
            session_id = ss.str();
            const std::string mode = body.value("mode", std::string("live"));
            const std::string session_name = body.value("session_name", std::string());
            replay_name = state::SessionRecorder::generate_session_name(mode, session_name);
            sessions_[session_id] = std::make_unique<sim::SimHarness>(cfg);
            recorders_[session_id] = std::make_unique<state::SessionRecorder>(replay_dir_, replay_name);
            recorders_[session_id]->set_config(nlohmann::json{
                { "mode", mode },
                { "seed", cfg.controller_config.seed },
                { "num_particles", cfg.controller_config.mcl_config.num_particles },
                { "field_half", cfg.controller_config.field.field_half },
                { "map_name", body.value("map_name", std::string("custom")) },
                { "algorithm", body.value("algorithm", std::string("manual")) },
                { "max_ticks", cfg.max_ticks },
            });
            nlohmann::json obstacles = nlohmann::json::array();
            for (const auto& o : cfg.controller_config.field.obstacles) {
                obstacles.push_back(nlohmann::json(o));
            }
            recorders_[session_id]->set_obstacles(obstacles);

            auto it = sessions_.find(session_id);
            if (it != sessions_.end() && body.contains("failures") && body["failures"].is_array()) {
                for (const auto& f : body["failures"]) {
                    noise::FailureEvent event;
                    event.start_tick = f.value("start_tick", 0);
                    event.duration_ticks = f.value("duration_ticks", 1);
                    event.sensor_idx = f.value("sensor_idx", -1);
                    event.param = f.value("param", 0.0);
                    event.type = failure_type_from_string(f.value("type", std::string("sensor_dead")));
                    it->second->schedule_failure(event);
                }
            }

            if (it != sessions_.end() && body.contains("failure_config") && body["failure_config"].is_object()) {
                pursuit::FailureGenConfig fcfg;
                const auto& fc = body["failure_config"];
                fcfg.sensor_dead_prob = fc.value("sensor_dead_prob", fcfg.sensor_dead_prob);
                fcfg.sensor_stuck_prob = fc.value("sensor_stuck_prob", fcfg.sensor_stuck_prob);
                fcfg.odom_spike_prob = fc.value("odom_spike_prob", fcfg.odom_spike_prob);
                fcfg.heading_bias_prob = fc.value("heading_bias_prob", fcfg.heading_bias_prob);
                fcfg.spurious_reflection_prob = fc.value("spurious_reflection_prob", fcfg.spurious_reflection_prob);
                fcfg.kidnap_prob = fc.value("kidnap_prob", fcfg.kidnap_prob);
                fcfg.min_duration_ticks = fc.value("min_duration_ticks", fcfg.min_duration_ticks);
                fcfg.max_duration_ticks = fc.value("max_duration_ticks", fcfg.max_duration_ticks);
                fcfg.sensor_dead_max_duration_ticks =
                    fc.value("sensor_dead_max_duration_ticks", fcfg.sensor_dead_max_duration_ticks);
                fcfg.sensor_stuck_max_duration_ticks =
                    fc.value("sensor_stuck_max_duration_ticks", fcfg.sensor_stuck_max_duration_ticks);
                fcfg.odom_spike_max_duration_ticks =
                    fc.value("odom_spike_max_duration_ticks", fcfg.odom_spike_max_duration_ticks);
                fcfg.heading_bias_max_duration_ticks =
                    fc.value("heading_bias_max_duration_ticks", fcfg.heading_bias_max_duration_ticks);
                fcfg.spurious_reflection_max_duration_ticks =
                    fc.value("spurious_reflection_max_duration_ticks", fcfg.spurious_reflection_max_duration_ticks);
                maybe_read_range(fc, "odom_spike_range", fcfg.odom_spike_range);
                maybe_read_range(fc, "heading_bias_range", fcfg.heading_bias_range);
                maybe_read_range(fc, "spurious_reflection_range", fcfg.spurious_reflection_range);
                const int max_ticks = fc.value("max_ticks", cfg.max_ticks > 0 ? cfg.max_ticks : 2000);
                const auto generated = pursuit::generate_random_failures(
                    fcfg,
                    max_ticks,
                    static_cast<uint64_t>(cfg.controller_config.seed),
                    cfg.controller_config.field.field_half);
                for (const auto& event : generated) {
                    it->second->schedule_failure(event);
                }
            }
        }

        res.set_content(nlohmann::json({
            { "session_id", session_id },
            { "session_name", replay_name }
        }).dump(), "application/json");
    });

    server_.Post(R"(/api/session/([^/]+)/tick)", [this](const httplib::Request& req, httplib::Response& res) {
        const std::string session_id = req.matches[1];
        nlohmann::json body = nlohmann::json::object();
        if (!req.body.empty()) {
            body = nlohmann::json::parse(req.body, nullptr, false);
            if (body.is_discarded()) {
                res.status = 400;
                res.set_content(json_error("invalid json").dump(), "application/json");
                return;
            }
        }
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = sessions_.find(session_id);
        auto rec_it = recorders_.find(session_id);
        if (it == sessions_.end() || rec_it == recorders_.end()) {
            res.status = 404;
            res.set_content(json_error("session not found").dump(), "application/json");
            return;
        }
        state::TickState tick;
        const bool has_explicit_vel = body.contains("linear_vel") && body.contains("angular_vel_deg");
        NavigationState& nav = nav_states_[session_id];

        if (has_explicit_vel) {
            if (!body["linear_vel"].is_number() || !body["angular_vel_deg"].is_number()) {
                res.status = 400;
                res.set_content(json_error("linear_vel and angular_vel_deg must be numbers").dump(), "application/json");
                return;
            }
            const double linear_vel = body["linear_vel"].get<double>();
            const double angular_vel_deg = body["angular_vel_deg"].get<double>();
            if (nav.status == NavigationState::Status::Navigating) {
                nav.status = NavigationState::Status::ManualOverride;
            }
            tick = it->second->tick(linear_vel, angular_vel_deg);
        } else if (nav.status == NavigationState::Status::Navigating &&
                   !nav.waypoints.empty()) {
            // Auto-navigate: compute velocity from WaypointFollower
            pursuit::WaypointFollower follower(nav.follower_config);
            const auto& last = it->second->history().empty()
                ? state::TickState{}
                : it->second->history().back();
            const sim::RobotState robot_state{
                last.chassis_pose.x,
                last.chassis_pose.y,
                last.chassis_pose.theta
            };
            const auto cmd = follower.compute(robot_state, nav.waypoints, nav.current_waypoint_idx);

            if (cmd.waypoint_reached) {
                nav.current_waypoint_idx++;
            }
            if (cmd.all_done || nav.current_waypoint_idx >= static_cast<int>(nav.waypoints.size())) {
                nav.status = NavigationState::Status::Idle;
                tick = it->second->tick(0.0, 0.0);
            } else {
                tick = it->second->tick(cmd.linear_velocity, cmd.angular_vel_deg);
            }
        } else if (nav.status == NavigationState::Status::ManualOverride) {
            // Empty body from ManualOverride -- resume navigation
            nav.status = NavigationState::Status::Navigating;
            sim::Action action = sim::Action::NONE;
            if (!action_from_string(body.value("action", std::string("none")), action)) {
                res.status = 400;
                res.set_content(json_error("invalid action").dump(), "application/json");
                return;
            }
            tick = it->second->tick(action);
        } else {
            sim::Action action = sim::Action::NONE;
            if (!action_from_string(body.value("action", std::string("none")), action)) {
                res.status = 400;
                res.set_content(json_error("invalid action").dump(), "application/json");
                return;
            }
            tick = it->second->tick(action);
        }
        rec_it->second->record(tick);

        // Include nav_status in response
        const std::string nav_status_str =
            nav.status == NavigationState::Status::Navigating ? "navigating" :
            nav.status == NavigationState::Status::ManualOverride ? "manual_override" : "idle";
        nlohmann::json tick_json = tick;
        tick_json["nav_status"] = nlohmann::json{
            { "status", nav_status_str },
            { "current_waypoint_index", nav.current_waypoint_idx },
            { "waypoints_remaining", std::max(0, static_cast<int>(nav.waypoints.size()) - nav.current_waypoint_idx) },
            { "completed", nav.status == NavigationState::Status::Idle && nav.waypoints.empty() == false }
        };
        res.set_content(tick_json.dump(), "application/json");
    });

    server_.Get(R"(/api/session/([^/]+)/state)", [this](const httplib::Request& req, httplib::Response& res) {
        const std::string session_id = req.matches[1];
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = sessions_.find(session_id);
        if (it == sessions_.end()) {
            res.status = 404;
            res.set_content(json_error("session not found").dump(), "application/json");
            return;
        }
        res.set_content(nlohmann::json(it->second->history()).dump(), "application/json");
    });

    server_.Get(R"(/api/session/([^/]+)/config)", [this](const httplib::Request& req, httplib::Response& res) {
        const std::string session_id = req.matches[1];
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = sessions_.find(session_id);
        if (it == sessions_.end()) {
            res.status = 404;
            res.set_content(json_error("session not found").dump(), "application/json");
            return;
        }

        const auto& cfg = it->second->config();
        nlohmann::json out = {
            { "seed", cfg.controller_config.seed },
            { "num_particles", cfg.controller_config.mcl_config.num_particles },
            { "field_half", cfg.controller_config.field.field_half },
            { "robot_radius", cfg.controller_config.field.robot_radius },
            { "max_ticks", cfg.max_ticks },
            { "obstacles", nlohmann::json::array() },
        };
        for (const auto& o : cfg.controller_config.field.obstacles) {
            out["obstacles"].push_back(nlohmann::json(o));
        }
        res.set_content(out.dump(), "application/json");
    });

    server_.Delete(R"(/api/session/([^/]+))", [this](const httplib::Request& req, httplib::Response& res) {
        const std::string session_id = req.matches[1];
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = sessions_.find(session_id);
        auto rec_it = recorders_.find(session_id);
        if (it == sessions_.end() || rec_it == recorders_.end()) {
            res.status = 404;
            res.set_content(json_error("session not found").dump(), "application/json");
            return;
        }
        const bool wrote = rec_it->second->write_atomic();
        const bool wrote_log = rec_it->second->write_log();
        const int tick_count = rec_it->second->tick_count();
        const std::string path = rec_it->second->output_path();
        const std::string log_path = rec_it->second->log_path();
        recorders_.erase(rec_it);
        sessions_.erase(it);
        nav_states_.erase(session_id);  // clean up navigation state

        res.set_content(nlohmann::json{
            { "saved", wrote },
            { "saved_log", wrote_log },
            { "ticks", tick_count },
            { "path", path },
            { "log_path", log_path },
        }.dump(), "application/json");
    });

    // POST /api/session/:id/navigate -- start waypoint navigation
    server_.Post(R"(/api/session/([^/]+)/navigate)", [this](const httplib::Request& req, httplib::Response& res) {
        const std::string session_id = req.matches[1];
        nlohmann::json body = nlohmann::json::object();
        if (!req.body.empty()) {
            body = nlohmann::json::parse(req.body, nullptr, false);
            if (body.is_discarded()) {
                res.status = 400;
                res.set_content(json_error("invalid json").dump(), "application/json");
                return;
            }
        }
        std::lock_guard<std::mutex> lock(mutex_);
        if (sessions_.find(session_id) == sessions_.end()) {
            res.status = 404;
            res.set_content(json_error("session not found").dump(), "application/json");
            return;
        }
        if (!body.contains("waypoints") || !body["waypoints"].is_array() ||
            body["waypoints"].empty()) {
            res.status = 400;
            res.set_content(json_error("waypoints array is required and must be non-empty").dump(), "application/json");
            return;
        }
        NavigationState nav;
        nav.status = NavigationState::Status::Navigating;
        nav.current_waypoint_idx = 0;

        for (const auto& wp : body["waypoints"]) {
            pursuit::Waypoint w;
            w.x = wp.value("x", 0.0);
            w.y = wp.value("y", 0.0);
            nav.waypoints.push_back(w);
        }
        if (body.contains("follower_config") && body["follower_config"].is_object()) {
            const auto& fc = body["follower_config"];
            nav.follower_config.linear_velocity    = fc.value("linear_velocity",    nav.follower_config.linear_velocity);
            nav.follower_config.waypoint_tolerance = fc.value("waypoint_tolerance", nav.follower_config.waypoint_tolerance);
            nav.follower_config.max_angular_vel_deg = fc.value("max_angular_vel_deg", nav.follower_config.max_angular_vel_deg);
            nav.follower_config.heading_gain       = fc.value("heading_gain",       nav.follower_config.heading_gain);
            nav.follower_config.slowdown_radius    = fc.value("slowdown_radius",    nav.follower_config.slowdown_radius);
        }
        nav_states_[session_id] = nav;

        res.set_content(nlohmann::json{
            { "status", "navigating" },
            { "total_waypoints", static_cast<int>(nav.waypoints.size()) }
        }.dump(), "application/json");
    });

    // GET /api/session/:id/navigate/status
    server_.Get(R"(/api/session/([^/]+)/navigate/status)", [this](const httplib::Request& req, httplib::Response& res) {
        const std::string session_id = req.matches[1];
        std::lock_guard<std::mutex> lock(mutex_);
        if (sessions_.find(session_id) == sessions_.end()) {
            res.status = 404;
            res.set_content(json_error("session not found").dump(), "application/json");
            return;
        }
        const auto& nav = nav_states_[session_id];
        const std::string status_str =
            nav.status == NavigationState::Status::Navigating ? "navigating" :
            nav.status == NavigationState::Status::ManualOverride ? "manual_override" : "idle";
        const int remaining = std::max(0, static_cast<int>(nav.waypoints.size()) - nav.current_waypoint_idx);
        res.set_content(nlohmann::json{
            { "status", status_str },
            { "current_waypoint_index", nav.current_waypoint_idx },
            { "waypoints_remaining", remaining },
            { "completed", nav.status == NavigationState::Status::Idle && !nav.waypoints.empty() }
        }.dump(), "application/json");
    });

    // DELETE /api/session/:id/navigate -- cancel navigation
    server_.Delete(R"(/api/session/([^/]+)/navigate)", [this](const httplib::Request& req, httplib::Response& res) {
        const std::string session_id = req.matches[1];
        std::lock_guard<std::mutex> lock(mutex_);
        if (sessions_.find(session_id) == sessions_.end()) {
            res.status = 404;
            res.set_content(json_error("session not found").dump(), "application/json");
            return;
        }
        const int remaining = std::max(0,
            static_cast<int>(nav_states_[session_id].waypoints.size()) -
            nav_states_[session_id].current_waypoint_idx);
        nav_states_[session_id] = NavigationState{};  // reset to Idle
        res.set_content(nlohmann::json{
            { "cancelled", true },
            { "waypoints_remaining", remaining }
        }.dump(), "application/json");
    });

    server_.Get("/api/replays", [this](const httplib::Request&, httplib::Response& res) {
        const auto files = list_replay_paths_merged(replay_dir_);
        nlohmann::json out = nlohmann::json::array();
        for (const auto& f : files) {
            out.push_back(f);
        }
        res.set_content(out.dump(), "application/json");
    });

    server_.Get(R"(/api/replays/(.*)/meta)", [this](const httplib::Request& req, httplib::Response& res) {
        const std::string file = url_decode_percent(req.matches[1]);
        if (!is_safe_replay_path_id(file, replay_dir_)) {
            res.status = 400;
            res.set_content(json_error("invalid file").dump(), "application/json");
            return;
        }
        const std::string path = replay_file_path_on_disk(file, replay_dir_);
        res.set_content(state::ReplayLoader::load_metadata(path).dump(), "application/json");
    });

    server_.Get(R"(/api/replays/(.*)/ticks)", [this](const httplib::Request& req, httplib::Response& res) {
        const std::string file = url_decode_percent(req.matches[1]);
        if (!is_safe_replay_path_id(file, replay_dir_)) {
            res.status = 400;
            res.set_content(json_error("invalid file").dump(), "application/json");
            return;
        }
        int from = 0;
        int to = 0;
        if (req.has_param("from")) from = std::stoi(req.get_param_value("from"));
        if (req.has_param("to")) to = std::stoi(req.get_param_value("to"));
        const std::string path = replay_file_path_on_disk(file, replay_dir_);
        const auto ticks = state::ReplayLoader::load_ticks(path, from, to);
        res.set_content(nlohmann::json(ticks).dump(), "application/json");
    });

}

void SimServer::run() {
    server_.listen("0.0.0.0", port_);
}

} // namespace server
