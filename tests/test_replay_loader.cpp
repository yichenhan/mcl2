#include "doctest/doctest.h"

#include "state/replay_loader.hpp"
#include "nlohmann/json.hpp"

#include <filesystem>
#include <fstream>

namespace {
std::string temp_replay_dir() {
    return "tmp_replays_loader";
}

void write_fixture(const std::string& dir, const std::string& name,
                   const nlohmann::json& config, int num_ticks) {
    std::filesystem::create_directories(dir);
    nlohmann::json out;
    out["session_id"] = name;
    out["config"] = config;
    out["obstacles"] = nlohmann::json::array();
    out["total_ticks"] = num_ticks;
    nlohmann::json ticks = nlohmann::json::array();
    for (int i = 0; i < num_ticks; ++i) {
        state::TickState t;
        t.tick = i;
        ticks.push_back(t);
    }
    out["ticks"] = ticks;
    std::ofstream f(dir + "/" + name + ".json", std::ios::trunc);
    f << out.dump(2);
}
}

TEST_CASE("ReplayLoader lists replay files and loads metadata") {
    const std::string dir = temp_replay_dir();
    std::filesystem::remove_all(dir);

    write_fixture(dir, "session_b", {{"num_particles", 77}}, 3);

    const auto files = state::ReplayLoader::list_replays(dir);
    REQUIRE(files.size() == 1);
    CHECK(files[0] == "session_b.json");

    const auto meta = state::ReplayLoader::load_metadata(dir + "/session_b.json");
    CHECK(meta["session_id"] == "session_b");
    CHECK(meta["total_ticks"] == 3);
}

TEST_CASE("ReplayLoader paginates tick ranges") {
    const std::string dir = temp_replay_dir();
    std::filesystem::remove_all(dir);

    write_fixture(dir, "session_c", nlohmann::json::object(), 10);

    const auto slice = state::ReplayLoader::load_ticks(dir + "/session_c.json", 3, 7);
    REQUIRE(slice.size() == 4);
    CHECK(slice[0].tick == 3);
    CHECK(slice[3].tick == 6);
}
