#include "server/api.hpp"

#include "noise/failure_injector.hpp"
#include "state/replay_loader.hpp"

#include "nlohmann/json.hpp"

#include <sstream>

namespace server {

namespace {
bool is_safe_filename(const std::string& name) {
    return !name.empty() && name.find("..") == std::string::npos && name.find('/') == std::string::npos;
}

noise::FailureType failure_type_from_string(const std::string& v) {
    if (v == "sensor_stuck") return noise::FailureType::SensorStuck;
    if (v == "odom_spike") return noise::FailureType::OdomSpike;
    if (v == "heading_bias") return noise::FailureType::HeadingBias;
    return noise::FailureType::SensorDead;
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
                sim::AABB b;
                b.min_x = o.value("min_x", 0.0);
                b.min_y = o.value("min_y", 0.0);
                b.max_x = o.value("max_x", 0.0);
                b.max_y = o.value("max_y", 0.0);
                cfg.field.obstacles.push_back(b);
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
                obstacles.push_back({
                    { "min_x", o.min_x },
                    { "min_y", o.min_y },
                    { "max_x", o.max_x },
                    { "max_y", o.max_y },
                });
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
        sim::Action action = sim::Action::NONE;
        if (!state::action_from_string(body.value("action", std::string("none")), action)) {
            res.status = 400;
            res.set_content(json_error("invalid action").dump(), "application/json");
            return;
        }

        std::lock_guard<std::mutex> lock(mutex_);
        auto it = sessions_.find(session_id);
        auto rec_it = recorders_.find(session_id);
        if (it == sessions_.end() || rec_it == recorders_.end()) {
            res.status = 404;
            res.set_content(json_error("session not found").dump(), "application/json");
            return;
        }
        state::TickState tick = it->second->tick(action);
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
            out["obstacles"].push_back({
                { "min_x", o.min_x },
                { "min_y", o.min_y },
                { "max_x", o.max_x },
                { "max_y", o.max_y },
            });
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
        const int tick_count = rec_it->second->tick_count();
        const std::string path = rec_it->second->output_path();
        recorders_.erase(rec_it);
        sessions_.erase(it);

        res.set_content(nlohmann::json{
            { "saved", wrote },
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
}

void SimServer::run() {
    server_.listen("0.0.0.0", port_);
}

} // namespace server
