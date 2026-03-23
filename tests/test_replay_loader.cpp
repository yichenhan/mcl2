#include "doctest/doctest.h"

#include "state/replay_loader.hpp"
#include "state/session_recorder.hpp"

#include <filesystem>
#include <fstream>

namespace {
std::string temp_replay_dir() {
    return "tmp_replays_loader";
}
}

TEST_CASE("ReplayLoader lists replay files and loads metadata") {
    const std::string dir = temp_replay_dir();
    std::filesystem::remove_all(dir);

    state::SessionRecorder recorder(dir, "session_b");
    recorder.set_config({{"num_particles", 77}});
    for (int i = 0; i < 3; ++i) {
        state::TickState t;
        t.tick = i;
        recorder.record(t);
    }
    REQUIRE(recorder.write_atomic());

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

    state::SessionRecorder recorder(dir, "session_c");
    for (int i = 0; i < 10; ++i) {
        state::TickState t;
        t.tick = i;
        recorder.record(t);
    }
    REQUIRE(recorder.write_atomic());

    const auto slice = state::ReplayLoader::load_ticks(dir + "/session_c.json", 3, 7);
    REQUIRE(slice.size() == 4);
    CHECK(slice[0].tick == 3);
    CHECK(slice[3].tick == 6);
}

TEST_CASE("ReplayLoader lists and paginates MCL replays") {
    const std::string dir = "tmp_replay_mcl_loader";
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir);

    nlohmann::json replay_json;
    replay_json["session_id"] = "mcl_session_a";
    replay_json["total_ticks"] = 3;
    replay_json["field_half"] = 72.0;
    replay_json["obstacles"] = nlohmann::json::array();
    replay_json["ticks"] = nlohmann::json::array();
    for (int i = 0; i < 3; ++i) {
        nlohmann::json tick;
        tick["raw_estimate"] = {{"x", static_cast<double>(i)}, {"y", 0.0}, {"theta", 0.0}};
        tick["gate"] = {
            {"accepted", true},
            {"failed_velocity", false},
            {"failed_r90", false},
            {"failed_passability", false},
            {"failed_residual", false},
            {"failed_wall_sum", false},
            {"jump_in", 0.0},
            {"radius_90_in", 0.1},
            {"spread_in", 0.2},
            {"reason", ""},
        };
        tick["valid_sensor_count"] = 4;
        tick["update_skipped"] = false;
        tick["cluster_stats"] = {
            {"spread", 0.2},
            {"radius_90", 0.1},
            {"centroid", {{"x", static_cast<double>(i)}, {"y", 0.0}}},
        };
        tick["n_eff"] = 10.0;
        const nlohmann::json snap = {
            {"particles", nlohmann::json::array({{{"x", 0.0}, {"y", 0.0}, {"weight", 1.0}}})},
            {"estimate", {{"x", static_cast<double>(i)}, {"y", 0.0}}},
            {"n_eff", 10.0},
            {"spread", 0.2},
            {"radius_90", 0.1},
        };
        tick["post_predict"] = snap;
        tick["post_update"] = snap;
        tick["post_resample"] = snap;
        replay_json["ticks"].push_back(tick);
    }

    {
        std::ofstream out(dir + "/mcl_session_a.json", std::ios::trunc);
        REQUIRE(out.is_open());
        out << replay_json.dump(2);
    }

    const auto files = state::ReplayLoader::list_mcl_replays(dir);
    REQUIRE(files.size() == 1);
    CHECK(files[0] == "mcl_session_a.json");

    const auto meta = state::ReplayLoader::load_mcl_metadata(dir + "/mcl_session_a.json");
    CHECK(meta["session_id"] == "mcl_session_a");
    CHECK(meta["total_ticks"] == 3);
    CHECK(meta["field_half"] == 72.0);

    const auto ticks = state::ReplayLoader::load_mcl_ticks(dir + "/mcl_session_a.json", 1, 3);
    REQUIRE(ticks.size() == 2);
    CHECK(ticks[0].raw_estimate.x == doctest::Approx(1.0));
    CHECK(ticks[1].raw_estimate.x == doctest::Approx(2.0));
}
