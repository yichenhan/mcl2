#pragma once

#include "mcl/mcl_controller.hpp"
#include "state/tick_state.hpp"

#include "nlohmann/json.hpp"

#include <string>
#include <vector>

namespace state {

class ReplayLoader {
public:
    static std::vector<std::string> list_replays(const std::string& directory);
    static std::vector<std::string> list_mcl_replays(const std::string& directory);
    static nlohmann::json load_metadata(const std::string& path);
    static nlohmann::json load_mcl_metadata(const std::string& path);
    static std::vector<TickState> load_ticks(const std::string& path, int from, int to);
    static std::vector<mcl::MCLTickResult> load_mcl_ticks(const std::string& path, int from, int to);
};

} // namespace state
