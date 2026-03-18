#include "state/session_recorder.hpp"

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <utility>

namespace state {

SessionRecorder::SessionRecorder(std::string directory, std::string session_id)
    : directory_(std::move(directory)), session_id_(std::move(session_id)) {}

void SessionRecorder::set_config(const nlohmann::json& config_json) {
    config_json_ = config_json;
}

void SessionRecorder::set_obstacles(const nlohmann::json& obstacles_json) {
    obstacles_json_ = obstacles_json;
}

void SessionRecorder::record(const TickState& tick) {
    ticks_.push_back(tick);
}

std::string SessionRecorder::output_path() const {
    return directory_ + "/" + session_id_ + ".json";
}

int SessionRecorder::tick_count() const {
    return static_cast<int>(ticks_.size());
}

bool SessionRecorder::write_atomic() const {
    std::error_code ec;
    std::filesystem::create_directories(directory_, ec);
    if (ec) return false;

    nlohmann::json j;
    j["session_id"] = session_id_;
    j["config"] = config_json_;
    j["obstacles"] = obstacles_json_;
    j["total_ticks"] = ticks_.size();
    j["ticks"] = ticks_;

    const std::string final_path = output_path();
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

} // namespace state
