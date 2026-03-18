#include "doctest/doctest.h"

#include "chaos/chaos_runner.hpp"

#include <filesystem>

TEST_CASE("ChaosRunner deterministic for same config") {
    chaos::ChaosConfig cfg;
    cfg.num_runs = 3;
    cfg.base_seed = 123;
    cfg.max_ticks = 40;
    cfg.output_report_path = "tmp_chaos_report_a.json";

    chaos::ChaosRunner a(cfg);
    const auto ra = a.run_all();

    chaos::ChaosRunner b(cfg);
    const auto rb = b.run_all();

    REQUIRE(ra.size() == rb.size());
    for (size_t i = 0; i < ra.size(); ++i) {
        CHECK(ra[i].seed == rb[i].seed);
        CHECK(ra[i].path_type == rb[i].path_type);
        CHECK(ra[i].post_convergence_rms == doctest::Approx(rb[i].post_convergence_rms));
    }
}

TEST_CASE("ChaosRunner writes JSON report") {
    const std::string report = "tmp_chaos_report_b.json";
    std::filesystem::remove(report);

    chaos::ChaosConfig cfg;
    cfg.num_runs = 2;
    cfg.base_seed = 999;
    cfg.max_ticks = 20;
    cfg.output_report_path = report;

    chaos::ChaosRunner runner(cfg);
    const auto results = runner.run_all();
    REQUIRE(results.size() == 2);
    CHECK(std::filesystem::exists(report));
}
