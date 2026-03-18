#include "state/replay_loader.hpp"

#include <algorithm>
#include <filesystem>
#include <fstream>

namespace state {

namespace {
nlohmann::json read_json_file(const std::string& path) {
    std::ifstream in(path);
    if (!in.is_open()) return nlohmann::json::object();
    nlohmann::json j;
    in >> j;
    return j;
}
} // namespace

std::vector<std::string> ReplayLoader::list_replays(const std::string& directory) {
    std::vector<std::string> out;
    std::error_code ec;
    if (!std::filesystem::exists(directory, ec)) return out;
    for (const auto& entry : std::filesystem::directory_iterator(directory, ec)) {
        if (ec) break;
        if (!entry.is_regular_file()) continue;
        if (entry.path().extension() == ".json") {
            out.push_back(entry.path().filename().string());
        }
    }
    std::sort(out.begin(), out.end());
    return out;
}

nlohmann::json ReplayLoader::load_metadata(const std::string& path) {
    const nlohmann::json j = read_json_file(path);
    if (!j.is_object()) return nlohmann::json::object();
    nlohmann::json out;
    out["session_id"] = j.value("session_id", "");
    out["total_ticks"] = j.value("total_ticks", 0);
    out["config"] = j.value("config", nlohmann::json::object());
    out["obstacles"] = j.value("obstacles", nlohmann::json::array());
    return out;
}

std::vector<TickState> ReplayLoader::load_ticks(const std::string& path, int from, int to) {
    std::vector<TickState> out;
    const nlohmann::json j = read_json_file(path);
    if (!j.is_object() || !j.contains("ticks") || !j["ticks"].is_array()) return out;

    const auto& ticks = j["ticks"];
    const int size = static_cast<int>(ticks.size());
    if (size == 0) return out;

    from = std::max(0, from);
    to = std::min(size, to);
    if (from >= to) return out;

    out.reserve(static_cast<size_t>(to - from));
    for (int i = from; i < to; ++i) {
        out.push_back(ticks[static_cast<size_t>(i)].get<TickState>());
    }
    return out;
}

} // namespace state
