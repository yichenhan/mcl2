#include "doctest/doctest.h"

#include "mcl/particle.hpp"
#include "state/session_recorder.hpp"

#include <filesystem>
#include <fstream>
#include <string>

namespace {
std::string temp_replay_dir() {
    return "/tmp/mcl_test_tmp_replays_session_recorder";
}
} // namespace

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

TEST_CASE("SessionRecorder omits particle clouds from replay JSON") {
    const std::string dir = temp_replay_dir();
    std::filesystem::remove_all(dir);

    state::SessionRecorder recorder(dir, "no_particles");
    state::TickState t;
    t.tick = 0;
    t.post_predict.particles.push_back(mcl::Particle{ 1.0f, 2.0f, 0.5f });
    t.post_update.particles.push_back(mcl::Particle{ 3.0f, 4.0f, 0.25f });
    t.post_resample.particles.push_back(mcl::Particle{ 5.0f, 6.0f, 0.125f });
    recorder.record(t);

    REQUIRE(recorder.write_atomic());
    std::ifstream in(recorder.output_path());
    nlohmann::json j;
    in >> j;
    const auto& tk = j["ticks"][0];
    REQUIRE(tk["post_predict"]["particles"].is_array());
    REQUIRE(tk["post_update"]["particles"].is_array());
    REQUIRE(tk["post_resample"]["particles"].is_array());
    CHECK(tk["post_predict"]["particles"].empty());
    CHECK(tk["post_update"]["particles"].empty());
    CHECK(tk["post_resample"]["particles"].empty());
}

TEST_CASE("SessionRecorder writes human readable log") {
    const std::string dir = temp_replay_dir();
    std::filesystem::remove_all(dir);

    state::SessionRecorder recorder(dir, "live_test_log");
    recorder.set_config({
        {"mode", "live"},
        {"seed", 42},
        {"num_particles", 120},
        {"map_name", "Empty Field"},
        {"algorithm", "arc_nav"},
    });

    state::TickState t;
    t.tick = 0;
    t.mcl_error = 1.25;
    t.odom_error = 0.75;
    t.gate_decision.accepted = true;
    t.cluster_stats.radius_90 = 2.0;
    t.active_failures = {"sensor_dead[0]"};
    recorder.record(t);

    REQUIRE(recorder.write_log());
    REQUIRE(std::filesystem::exists(recorder.log_path()));

    std::ifstream in(recorder.log_path());
    std::string contents((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    CHECK(contents.find("=== MCL Session: live_test_log ===") != std::string::npos);
    CHECK(contents.find("Mode: live") != std::string::npos);
    CHECK(contents.find("sensor_dead[0]") != std::string::npos);
}

TEST_CASE("SessionRecorder generates timestamped session names") {
    const std::string name = state::SessionRecorder::generate_session_name("route", "Slalom Run");
    CHECK(name.find("route_slalomrun_") == 0);
    CHECK(name.size() > std::string("route_slalomrun_2026-01-01_00-00-00_abc").size() - 4);
}

TEST_CASE("SessionRecorder writes analytics block into replay JSON") {
    const std::string dir = temp_replay_dir();
    std::filesystem::remove_all(dir);

    state::SessionRecorder recorder(dir, "with_analytics");
    state::SessionAnalytics a;
    a.peak_accepted_error = 7.5;
    a.total_ticks = 1;
    recorder.set_analytics(a);
    state::TickState t;
    t.tick = 0;
    recorder.record(t);
    REQUIRE(recorder.write_atomic());

    std::ifstream in(recorder.output_path());
    nlohmann::json j;
    in >> j;
    REQUIRE(j.contains("analytics"));
    CHECK(j["analytics"]["peak_accepted_error"].get<double>() == doctest::Approx(7.5));
}

TEST_CASE("SessionRecorder backward compat without set_analytics") {
    const std::string dir = temp_replay_dir();
    std::filesystem::remove_all(dir);
    state::SessionRecorder recorder(dir, "no_analytics");
    state::TickState t;
    t.tick = 0;
    recorder.record(t);
    REQUIRE(recorder.write_atomic());
    std::ifstream in(recorder.output_path());
    nlohmann::json j;
    in >> j;
    CHECK_FALSE(j.contains("analytics"));
}
