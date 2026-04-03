#include "doctest/doctest.h"

#include "state/replay_loader.hpp"
#include "state/session_recorder.hpp"

#include <filesystem>
#include <fstream>

namespace {
std::string temp_replay_dir() {
    return "/tmp/mcl_test_tmp_replays_loader";
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

TEST_CASE("ReplayLoader load_metadata returns analytics when present") {
    const std::string dir = temp_replay_dir();
    std::filesystem::remove_all(dir);

    state::SessionRecorder recorder(dir, "meta_analytics");
    state::SessionAnalytics a;
    a.gate_precision = 0.9123;
    recorder.set_analytics(a);
    state::TickState t;
    t.tick = 0;
    recorder.record(t);
    REQUIRE(recorder.write_atomic());

    const auto meta = state::ReplayLoader::load_metadata(dir + "/meta_analytics.json");
    REQUIRE(meta.contains("analytics"));
    CHECK(meta["analytics"]["gate_precision"].get<double>() == doctest::Approx(0.9123));
}

