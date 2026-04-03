#include "doctest/doctest.h"

#include "state/session_analytics.hpp"

#include <cmath>

namespace {

state::TickState make_tick(
    int tick,
    bool accepted,
    double mcl_error,
    double accepted_error,
    double radius_90 = 1.0,
    const mcl::GateDecision& gate = {}) {
    state::TickState t{};
    t.tick = tick;
    t.gate_decision = gate;
    t.gate_decision.accepted = accepted;
    t.mcl_error = mcl_error;
    t.accepted_error = accepted_error;
    t.cluster_stats.radius_90 = radius_90;
    t.observed_readings = {10.0, 10.0, 10.0, 10.0};
    t.post_resample.n_eff = 100.0;
    return t;
}

} // namespace

TEST_CASE("compute_analytics: all-accepted perfect run") {
    std::vector<state::TickState> ticks;
    for (int i = 0; i < 100; ++i) {
        ticks.push_back(make_tick(i, true, 0.5, 0.5, 1.0));
    }
    const auto a = state::compute_analytics(ticks);
    CHECK(a.peak_accepted_error == doctest::Approx(0.5));
    CHECK(a.mean_accepted_error == doctest::Approx(0.5));
    CHECK(a.p95_accepted_error == doctest::Approx(0.5));
    CHECK(a.false_accept_count == 0);
    CHECK(a.false_reject_count == 0);
    CHECK(a.gate_availability == doctest::Approx(1.0));
    CHECK(a.gate_precision == doctest::Approx(1.0));
    CHECK(a.gate_recall == doctest::Approx(1.0));
    CHECK(a.cold_start_convergence_ticks == 0);
    CHECK(a.total_ticks == 100);
}

TEST_CASE("compute_analytics: single false accept") {
    std::vector<state::TickState> ticks;
    for (int i = 0; i < 100; ++i) {
        const double ae = (i == 50) ? 5.0 : 0.5;
        const double me = (i == 50) ? 5.0 : 0.5;
        ticks.push_back(make_tick(i, true, me, ae, 1.0));
    }
    const auto a = state::compute_analytics(ticks, {}, 3.0, 3.0, 3.0);
    CHECK(a.false_accept_count == 1);
    CHECK(a.longest_false_accept_streak == 1);
    CHECK(a.peak_error_during_false_accept == doctest::Approx(5.0));
    CHECK(a.gate_precision == doctest::Approx(99.0 / 100.0));
}

TEST_CASE("compute_analytics: sustained false accept streak") {
    std::vector<state::TickState> ticks;
    for (int i = 0; i < 100; ++i) {
        const bool fa = (i >= 40 && i <= 49);
        ticks.push_back(make_tick(i, true, fa ? 2.0 : 0.5, fa ? 4.0 : 0.5, 1.0));
    }
    const auto a = state::compute_analytics(ticks, {}, 3.0, 3.0, 3.0);
    CHECK(a.false_accept_count == 10);
    CHECK(a.longest_false_accept_streak == 10);
    CHECK(a.mean_false_accept_duration == doctest::Approx(10.0));
}

TEST_CASE("compute_analytics: false reject with refined filter") {
    mcl::GateDecision g_r90;
    g_r90.accepted = false;
    g_r90.failed_r90 = true;
    g_r90.radius_90_in = 3.5;

    mcl::GateDecision g_big;
    g_big.accepted = false;
    g_big.failed_r90 = true;
    g_big.radius_90_in = 12.0;

    std::vector<state::TickState> ticks;
    for (int i = 0; i < 30; ++i) ticks.push_back(make_tick(i, true, 0.5, 0.5));
    ticks.push_back(make_tick(30, false, 1.0, 0.0, 3.5, g_r90));
    ticks.push_back(make_tick(31, false, 1.0, 0.0, 12.0, g_big));
    for (int i = 32; i < 50; ++i) ticks.push_back(make_tick(i, true, 0.5, 0.5));

    mcl::GateConfig gcfg;
    gcfg.max_radius_90_in = 4.0;
    const auto a = state::compute_analytics(ticks, gcfg, 3.0, 3.0, 3.0);
    CHECK(a.false_reject_count == 1);
}

TEST_CASE("compute_analytics: per-gate rejection breakdown") {
    mcl::GateDecision gr;
    gr.accepted = false;
    gr.failed_r90 = true;
    mcl::GateDecision gs;
    gs.accepted = false;
    gs.failed_residual = true;
    std::vector<state::TickState> ticks;
    ticks.push_back(make_tick(10, false, 5.0, 0.0, 10.0, gr));
    ticks.push_back(make_tick(20, false, 5.0, 0.0, 1.0, gs));
    const auto a = state::compute_analytics(ticks);
    CHECK(a.r90_reject_count == 1);
    CHECK(a.residual_reject_count == 1);
}

TEST_CASE("compute_analytics: oracle precision and recall") {
    std::vector<state::TickState> ticks;
    for (int i = 0; i < 5; ++i) ticks.push_back(make_tick(i, true, 1.0, 0.5));
    for (int i = 0; i < 2; ++i) ticks.push_back(make_tick(10 + i, true, 5.0, 0.5));
    for (int i = 0; i < 2; ++i) ticks.push_back(make_tick(20 + i, false, 1.0, 0.0));
    ticks.push_back(make_tick(99, false, 5.0, 0.0));
    const auto a = state::compute_analytics(ticks, {}, 3.0, 3.0, 3.0);
    CHECK(a.gate_precision == doctest::Approx(5.0 / 7.0));
    CHECK(a.gate_recall == doctest::Approx(5.0 / 7.0));
}

TEST_CASE("compute_analytics: cold start convergence") {
    std::vector<state::TickState> ticks;
    for (int i = 0; i < 20; ++i) {
        mcl::GateDecision g;
        g.accepted = false;
        ticks.push_back(make_tick(i, false, 5.0, 0.0, 10.0, g));
    }
    ticks.push_back(make_tick(20, true, 1.2, 0.5, 1.0));
    const auto a = state::compute_analytics(ticks, {}, 3.0, 3.0, 3.0);
    CHECK(a.cold_start_convergence_ticks == 20);
    CHECK(a.cold_start_convergence_error == doctest::Approx(1.2));
}

TEST_CASE("compute_analytics: post-convergence stability") {
    std::vector<state::TickState> ticks;
    for (int i = 0; i < 10; ++i) ticks.push_back(make_tick(i, false, 5.0, 0.0));
    for (int i = 10; i < 60; ++i) {
        const double ae = (i % 2 == 0) ? 0.5 : 1.5;
        ticks.push_back(make_tick(i, true, 0.5, ae, 1.0));
    }
    const auto a = state::compute_analytics(ticks, {}, 3.0, 3.0, 3.0);
    CHECK(a.post_convergence_error_stddev > 0.0);
}

TEST_CASE("compute_analytics: reacquisition") {
    mcl::GateDecision gr;
    gr.accepted = false;
    gr.failed_r90 = true;
    std::vector<state::TickState> ticks;
    for (int i = 0; i < 50; ++i) ticks.push_back(make_tick(i, true, 0.5, 0.5));
    for (int t = 50; t <= 55; ++t) ticks.push_back(make_tick(t, false, 1.0, 0.0, 10.0, gr));
    ticks.push_back(make_tick(56, true, 4.0, 0.5));
    ticks.push_back(make_tick(57, true, 4.0, 0.5));
    ticks.push_back(make_tick(58, true, 0.5, 0.5));
    for (int i = 59; i < 80; ++i) ticks.push_back(make_tick(i, true, 0.5, 0.5));
    for (int t = 80; t <= 90; ++t) ticks.push_back(make_tick(t, false, 1.0, 0.0, 10.0, gr));
    ticks.push_back(make_tick(91, true, 4.0, 0.5));
    ticks.push_back(make_tick(92, true, 4.0, 0.5));
    ticks.push_back(make_tick(93, true, 4.0, 0.5));
    ticks.push_back(make_tick(94, true, 4.0, 0.5));
    ticks.push_back(make_tick(95, true, 0.5, 0.5));
    const auto a = state::compute_analytics(ticks, {}, 3.0, 3.0, 3.0);
    CHECK(a.reacquisition_count == 2);
    CHECK(a.worst_reacquisition_ticks == 4);
    CHECK(a.mean_reacquisition_ticks == doctest::Approx(3.0));
}

TEST_CASE("compute_analytics: empty tick vector") {
    const auto a = state::compute_analytics({});
    CHECK(a.total_ticks == 0);
}

TEST_CASE("SessionAnalytics JSON roundtrip") {
    state::SessionAnalytics a;
    a.peak_accepted_error = 3.3;
    a.peak_accepted_error_tick = 42;
    a.gate_precision = 0.88;
    a.false_accept_count = 7;
    nlohmann::json j;
    to_json(j, a);
    state::SessionAnalytics b;
    from_json(j, b);
    CHECK(b.peak_accepted_error == doctest::Approx(3.3));
    CHECK(b.peak_accepted_error_tick == 42);
    CHECK(b.gate_precision == doctest::Approx(0.88));
    CHECK(b.false_accept_count == 7);
}
