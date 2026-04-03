#pragma once

#include "pursuit/waypoint_follower.hpp"
#include "sim/sim_harness.hpp"
#include "state/session_recorder.hpp"

#include "httplib/httplib.h"

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace server {

// ---------------------------------------------------------------------------
// NavigationState
// ---------------------------------------------------------------------------
// Per-session navigation state machine.
// Idle      -> POST /navigate -> Navigating
// Navigating -> POST /tick (empty body) -> auto-compute velocity from follower
// Navigating -> POST /tick (with velocity) -> ManualOverride
// ManualOverride -> POST /tick (empty body) -> Navigating (resume)
// Navigating -> all waypoints reached or DELETE /navigate -> Idle
// ---------------------------------------------------------------------------
struct NavigationState {
    enum class Status { Idle, Navigating, ManualOverride };
    Status status = Status::Idle;
    std::vector<pursuit::Waypoint> waypoints;
    int current_waypoint_idx = 0;
    pursuit::FollowerConfig follower_config{};
};

class SimServer {
public:
    SimServer(int port, std::string replay_dir);
    void run();

private:
    void setup_routes();
    static void set_common_headers(httplib::Response& res);
    static nlohmann::json json_error(const std::string& message);

    int port_;
    std::string replay_dir_;
    httplib::Server server_;
    std::mutex mutex_;
    int next_session_id_ = 1;

    std::unordered_map<std::string, std::unique_ptr<sim::SimHarness>> sessions_;
    std::unordered_map<std::string, std::unique_ptr<state::SessionRecorder>> recorders_;
    std::unordered_map<std::string, NavigationState> nav_states_;
};

} // namespace server
