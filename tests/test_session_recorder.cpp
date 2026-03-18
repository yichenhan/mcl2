#include "doctest/doctest.h"

#include "state/session_recorder.hpp"

#include <filesystem>
#include <fstream>

namespace {
std::string temp_replay_dir() {
    return "tmp_replays_session_recorder";
}
}

TEST_CASE("SessionRecorder writes valid replay JSON atomically") {
    const std::string dir = temp_replay_dir();
    std::filesystem::remove_all(dir);

    state::SessionRecorder recorder(dir, "session_a");
    recorder.set_config({{"num_particles", 100}});

    state::TickState t;
    t.tick = 0;
    recorder.record(t);
    t.tick = 1;
    recorder.record(t);

    REQUIRE(recorder.write_atomic());
    REQUIRE(std::filesystem::exists(recorder.output_path()));

    std::ifstream in(recorder.output_path());
    nlohmann::json j;
    in >> j;
    CHECK(j["session_id"] == "session_a");
    CHECK(j["total_ticks"] == 2);
    CHECK(j["ticks"].size() == 2);
}
