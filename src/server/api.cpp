#include "server/api.hpp"

#include "noise/failure_injector.hpp"
#include "pursuit/route.hpp"
#include "pursuit/route_runner.hpp"
#include "state/replay_loader.hpp"

#include "nlohmann/json.hpp"

#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace server {

namespace {
bool is_safe_filename(const std::string& name) {
    return !name.empty() && name.find("..") == std::string::npos && name.find('/') == std::string::npos;
}

bool write_mcl_replay_atomic(
    const std::string& directory,
    const std::string& file_stem,
    const state::SimSession& session) {
    std::error_code ec;
    std::filesystem::create_directories(directory, ec);
    if (ec) return false;

    nlohmann::json j;
    j["session_id"] = file_stem;
    j["total_ticks"] = session.mcl_history().size();
    j["field_half"] = session.config().field.field_half;
    j["obstacles"] = nlohmann::json::array();
    for (const auto& o : session.config().field.obstacles) {
        j["obstacles"].push_back(nlohmann::json(o));
    }
    j["ticks"] = session.mcl_history();

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

noise::FailureType failure_type_from_string(const std::string& v) {
    if (v == "sensor_stuck") return noise::FailureType::SensorStuck;
    if (v == "odom_spike") return noise::FailureType::OdomSpike;
    if (v == "heading_bias") return noise::FailureType::HeadingBias;
    if (v == "kidnap") return noise::FailureType::Kidnap;
    if (v == "spurious_reflection") return noise::FailureType::SpuriousReflection;
    return noise::FailureType::SensorDead;
}
} // namespace

SimServer::SimServer(int port, std::string replay_dir, std::string mcl_replay_dir)
    : port_(port), replay_dir_(std::move(replay_dir)), mcl_replay_dir_(std::move(mcl_replay_dir)) {
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

        state::SimSessionConfig cfg;
        cfg.seed = body.value("seed", cfg.seed);
        if (body.contains("num_particles")) {
            cfg.mcl_config.num_particles = body["num_particles"].get<int>();
        }

        if (body.contains("obstacles") && body["obstacles"].is_array()) {
            cfg.field.obstacles.clear();
            for (const auto& o : body["obstacles"]) {
                cfg.field.obstacles.push_back(o.get<sim::Obstacle>());
            }
        }

        std::string session_id;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            std::ostringstream ss;
            ss << "s" << next_session_id_++;
            session_id = ss.str();
            sessions_[session_id] = std::make_unique<state::SimSession>(cfg);
            recorders_[session_id] = std::make_unique<state::SessionRecorder>(replay_dir_, session_id);
            recorders_[session_id]->set_config(nlohmann::json{
                { "seed", cfg.seed },
                { "num_particles", cfg.mcl_config.num_particles },
                { "field_half", cfg.field.field_half },
            });
            nlohmann::json obstacles = nlohmann::json::array();
            for (const auto& o : cfg.field.obstacles) {
                obstacles.push_back(nlohmann::json(o));
            }
            recorders_[session_id]->set_obstacles(obstacles);
        }

        if (body.contains("failures") && body["failures"].is_array()) {
            std::lock_guard<std::mutex> lock(mutex_);
            auto it = sessions_.find(session_id);
            if (it != sessions_.end()) {
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
        }

        res.set_content(nlohmann::json({ { "session_id", session_id } }).dump(), "application/json");
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
        if (body.contains("linear_vel") && body.contains("angular_vel_deg")) {
            if (!body["linear_vel"].is_number() || !body["angular_vel_deg"].is_number()) {
                res.status = 400;
                res.set_content(json_error("linear_vel and angular_vel_deg must be numbers").dump(), "application/json");
                return;
            }
            const double linear_vel = body["linear_vel"].get<double>();
            const double angular_vel_deg = body["angular_vel_deg"].get<double>();
            tick = it->second->tick(linear_vel, angular_vel_deg);
        } else {
            sim::Action action = sim::Action::NONE;
            if (!state::action_from_string(body.value("action", std::string("none")), action)) {
                res.status = 400;
                res.set_content(json_error("invalid action").dump(), "application/json");
                return;
            }
            tick = it->second->tick(action);
        }
        rec_it->second->record(tick);
        res.set_content(nlohmann::json(tick).dump(), "application/json");
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
            { "seed", cfg.seed },
            { "num_particles", cfg.mcl_config.num_particles },
            { "field_half", cfg.field.field_half },
            { "robot_radius", cfg.field.robot_radius },
            { "obstacles", nlohmann::json::array() },
        };
        for (const auto& o : cfg.field.obstacles) {
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
        const bool wrote_mcl = write_mcl_replay_atomic(mcl_replay_dir_, session_id, *it->second);
        const int tick_count = rec_it->second->tick_count();
        const std::string path = rec_it->second->output_path();
        recorders_.erase(rec_it);
        sessions_.erase(it);

        res.set_content(nlohmann::json{
            { "saved", wrote },
            { "saved_mcl", wrote_mcl },
            { "ticks", tick_count },
            { "path", path },
        }.dump(), "application/json");
    });

    server_.Get("/api/replays", [this](const httplib::Request&, httplib::Response& res) {
        const auto files = state::ReplayLoader::list_replays(replay_dir_);
        nlohmann::json out = nlohmann::json::array();
        for (const auto& f : files) {
            out.push_back(f);
        }
        res.set_content(out.dump(), "application/json");
    });

    server_.Get("/api/routes", [this](const httplib::Request&, httplib::Response& res) {
        nlohmann::json out = nlohmann::json::array();
        std::error_code ec;
        std::filesystem::create_directories(route_dir_, ec);
        if (ec) {
            res.status = 500;
            res.set_content(json_error("failed to create route dir").dump(), "application/json");
            return;
        }
        for (const auto& entry : std::filesystem::directory_iterator(route_dir_, ec)) {
            if (ec) break;
            if (!entry.is_regular_file()) continue;
            if (entry.path().extension() != ".json") continue;
            out.push_back(entry.path().filename().string());
        }
        res.set_content(out.dump(), "application/json");
    });

    server_.Get(R"(/api/routes/([^/]+))", [this](const httplib::Request& req, httplib::Response& res) {
        const std::string route_file = req.matches[1];
        if (!is_safe_filename(route_file)) {
            res.status = 400;
            res.set_content(json_error("invalid route file").dump(), "application/json");
            return;
        }
        const std::string path = route_dir_ + "/" + route_file;
        std::ifstream in(path);
        if (!in.is_open()) {
            res.status = 404;
            res.set_content(json_error("route not found").dump(), "application/json");
            return;
        }
        nlohmann::json route_json;
        in >> route_json;
        res.set_content(route_json.dump(), "application/json");
    });

    server_.Post("/api/routes", [this](const httplib::Request& req, httplib::Response& res) {
        nlohmann::json body = nlohmann::json::parse(req.body, nullptr, false);
        if (body.is_discarded() || !body.is_object()) {
            res.status = 400;
            res.set_content(json_error("invalid json").dump(), "application/json");
            return;
        }
        std::string name = body.value("name", std::string(""));
        if (name.empty()) {
            res.status = 400;
            res.set_content(json_error("missing route name").dump(), "application/json");
            return;
        }
        if (name.find('/') != std::string::npos || name.find("..") != std::string::npos) {
            res.status = 400;
            res.set_content(json_error("invalid route name").dump(), "application/json");
            return;
        }
        if (name.size() < 5 || name.substr(name.size() - 5) != ".json") {
            name += ".json";
        }

        std::error_code ec;
        std::filesystem::create_directories(route_dir_, ec);
        if (ec) {
            res.status = 500;
            res.set_content(json_error("failed to create route dir").dump(), "application/json");
            return;
        }

        const std::string out_path = route_dir_ + "/" + name;
        std::ofstream out(out_path, std::ios::trunc);
        if (!out.is_open()) {
            res.status = 500;
            res.set_content(json_error("failed to write route").dump(), "application/json");
            return;
        }
        out << body.dump(2);
        out.close();
        res.set_content(nlohmann::json{
            {"saved", true},
            {"file", name},
        }.dump(), "application/json");
    });

    server_.Post(R"(/api/routes/([^/]+)/run)", [this](const httplib::Request& req, httplib::Response& res) {
        const std::string route_file = req.matches[1];
        if (!is_safe_filename(route_file)) {
            res.status = 400;
            res.set_content(json_error("invalid route file").dump(), "application/json");
            return;
        }

        try {
            pursuit::RouteDefinition route = pursuit::load_route(route_dir_ + "/" + route_file);
            if (req.has_param("seed")) {
                route.failure_seed = static_cast<uint64_t>(std::strtoull(req.get_param_value("seed").c_str(), nullptr, 10));
            }
            pursuit::RouteRunner runner;
            const pursuit::RouteResult rr = runner.run(route, replay_dir_, mcl_replay_dir_);

            nlohmann::json out = {
                {"replay_file", rr.replay_file},
                {"result", {
                    {"route_name", rr.route_name},
                    {"seed", rr.seed},
                    {"total_ticks", rr.total_ticks},
                    {"waypoints_reached", rr.waypoints_reached},
                    {"completed", rr.completed},
                    {"final_mcl_error", rr.final_mcl_error},
                    {"mean_mcl_error", rr.mean_mcl_error},
                }},
            };
            res.set_content(out.dump(), "application/json");
        } catch (const std::exception& ex) {
            res.status = 400;
            res.set_content(json_error(ex.what()).dump(), "application/json");
        }
    });

    server_.Get(R"(/api/replays/([^/]+)/meta)", [this](const httplib::Request& req, httplib::Response& res) {
        const std::string file = req.matches[1];
        if (!is_safe_filename(file)) {
            res.status = 400;
            res.set_content(json_error("invalid file").dump(), "application/json");
            return;
        }
        res.set_content(state::ReplayLoader::load_metadata(replay_dir_ + "/" + file).dump(), "application/json");
    });

    server_.Get(R"(/api/replays/([^/]+)/ticks)", [this](const httplib::Request& req, httplib::Response& res) {
        const std::string file = req.matches[1];
        if (!is_safe_filename(file)) {
            res.status = 400;
            res.set_content(json_error("invalid file").dump(), "application/json");
            return;
        }
        int from = 0;
        int to = 0;
        if (req.has_param("from")) from = std::stoi(req.get_param_value("from"));
        if (req.has_param("to")) to = std::stoi(req.get_param_value("to"));
        const auto ticks = state::ReplayLoader::load_ticks(replay_dir_ + "/" + file, from, to);
        res.set_content(nlohmann::json(ticks).dump(), "application/json");
    });

    server_.Get("/api/mcl-replays", [this](const httplib::Request&, httplib::Response& res) {
        const auto files = state::ReplayLoader::list_mcl_replays(mcl_replay_dir_);
        nlohmann::json out = nlohmann::json::array();
        for (const auto& f : files) {
            out.push_back(f);
        }
        res.set_content(out.dump(), "application/json");
    });

    server_.Get(R"(/api/mcl-replays/([^/]+)/meta)", [this](const httplib::Request& req, httplib::Response& res) {
        const std::string file = req.matches[1];
        if (!is_safe_filename(file)) {
            res.status = 400;
            res.set_content(json_error("invalid file").dump(), "application/json");
            return;
        }
        res.set_content(state::ReplayLoader::load_mcl_metadata(mcl_replay_dir_ + "/" + file).dump(), "application/json");
    });

    server_.Get(R"(/api/mcl-replays/([^/]+)/ticks)", [this](const httplib::Request& req, httplib::Response& res) {
        const std::string file = req.matches[1];
        if (!is_safe_filename(file)) {
            res.status = 400;
            res.set_content(json_error("invalid file").dump(), "application/json");
            return;
        }
        int from = 0;
        int to = 0;
        if (req.has_param("from")) from = std::stoi(req.get_param_value("from"));
        if (req.has_param("to")) to = std::stoi(req.get_param_value("to"));
        const auto ticks = state::ReplayLoader::load_mcl_ticks(mcl_replay_dir_ + "/" + file, from, to);
        res.set_content(nlohmann::json(ticks).dump(), "application/json");
    });
}

void SimServer::run() {
    server_.listen("0.0.0.0", port_);
}

} // namespace server
