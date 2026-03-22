#pragma once

#include "state/sim_session.hpp"

#include "httplib/httplib.h"

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

namespace server {

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
    std::string route_dir_ = "../routes";
    httplib::Server server_;
    std::mutex mutex_;
    int next_session_id_ = 1;

    std::unordered_map<std::string, std::unique_ptr<state::SimSession>> sessions_;
};

} // namespace server
