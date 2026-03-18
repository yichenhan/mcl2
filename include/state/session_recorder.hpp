#pragma once

#include "state/tick_state.hpp"

#include "nlohmann/json.hpp"

#include <string>
#include <vector>

namespace state {

class SessionRecorder {
public:
    SessionRecorder(std::string directory, std::string session_id);

    void set_config(const nlohmann::json& config_json);
    void set_obstacles(const nlohmann::json& obstacles_json);
    void record(const TickState& tick);
    bool write_atomic() const;

    std::string output_path() const;
    int tick_count() const;

private:
    std::string directory_;
    std::string session_id_;
    nlohmann::json config_json_ = nlohmann::json::object();
    nlohmann::json obstacles_json_ = nlohmann::json::array();
    std::vector<TickState> ticks_;
};

} // namespace state
