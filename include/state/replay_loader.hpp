#pragma once

#include "state/tick_state.hpp"

#include "nlohmann/json.hpp"

#include <string>
#include <vector>

namespace state {

class ReplayLoader {
public:
    static std::vector<std::string> list_replays(const std::string& directory);
    static nlohmann::json load_metadata(const std::string& path);
    static std::vector<TickState> load_ticks(const std::string& path, int from, int to);
};

} // namespace state
