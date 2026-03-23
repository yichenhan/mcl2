#pragma once

#include "pursuit/route.hpp"
#include "state/sim_session.hpp"

#include <string>

namespace pursuit {

struct RouteResult {
    std::string route_name;
    uint64_t seed = 0;
    int total_ticks = 0;
    int waypoints_reached = 0;
    bool completed = false;
    double final_mcl_error = 0.0;
    double mean_mcl_error = 0.0;
    std::string replay_file;
};

class RouteRunner {
public:
    RouteResult run(
        const RouteDefinition& route,
        const std::string& replay_dir,
        const std::string& mcl_replay_dir = std::string{});
};

} // namespace pursuit

