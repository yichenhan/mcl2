#pragma once

#include "state/session_analytics.hpp"
#include "state/tick_state.hpp"

#include "nlohmann/json.hpp"

#include <string>
#include <vector>

namespace state {

class SessionRecorder {
public:
    SessionRecorder(std::string directory, std::string session_id);
    static std::string generate_session_name(const std::string& mode, const std::string& optional_label = "");

    void set_config(const nlohmann::json& config_json);
    void set_obstacles(const nlohmann::json& obstacles_json);
    void set_analytics(const SessionAnalytics& analytics);
    void record(const TickState& tick);
    bool write_atomic() const;
    bool write_log() const;

    std::string output_path() const;
    std::string log_path() const;
    int tick_count() const;

private:
    std::string directory_;
    std::string session_id_;
    nlohmann::json config_json_ = nlohmann::json::object();
    nlohmann::json obstacles_json_ = nlohmann::json::array();
    nlohmann::json analytics_json_ = nlohmann::json::object();
    bool has_analytics_ = false;
    std::vector<TickState> ticks_;
};

} // namespace state
