#include "state/session_analytics.hpp"
#include "state/session_recorder.hpp"
#include "stress/stress_generators.hpp"
#include "sim/sim_harness.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <numeric>
#include <string>
#include <vector>

#if defined(_WIN32)
#include <io.h>
#else
#include <unistd.h>
#endif

namespace {

bool stdout_is_tty() {
#if defined(_WIN32)
    return _isatty(_fileno(stdout)) != 0;
#else
    return isatty(STDOUT_FILENO) != 0;
#endif
}

// ---------------------------------------------------------------------------
// Shared scenario runner: simulate one scenario, return analytics + history
// ---------------------------------------------------------------------------
struct ScenarioResult {
    uint64_t seed = 0;
    int profile_id = 0;
    state::SessionAnalytics analytics{};
};

ScenarioResult run_scenario(
    uint64_t seed,
    int profile_id,
    int total_ticks,
    std::vector<state::TickState>* out_history = nullptr)
{
    sim::Field field;
    const int n_obs = 1 + static_cast<int>(seed % 5u);
    field.obstacles = stress::generate_random_obstacles(seed, field, n_obs);

    stress::RouteResult route = stress::generate_route(field, total_ticks, seed);
    stress::StressNoiseBundle noise =
        stress::generate_stress_noise(profile_id, total_ticks, seed, field.field_half);

    sim::SimHarness::Config cfg;
    cfg.initial_state = route.start_pose;
    cfg.controller_config.field = field;
    cfg.odom_noise = noise.odom;
    cfg.sensor_noise = noise.sensor;
    cfg.controller_config.seed = static_cast<uint32_t>(seed & 0xFFFFFFFFu);
    sim::SimHarness harness(cfg);
    for (const auto& ev : noise.failure_events) harness.schedule_failure(ev);
    for (const auto& cmd : route.velocity_commands) harness.tick(cmd.first, cmd.second);

    ScenarioResult r;
    r.seed = seed;
    r.profile_id = profile_id;
    r.analytics = state::compute_analytics(harness.history());
    if (out_history) *out_history = harness.history();
    return r;
}

// Write a single replay to disk immediately.
void write_replay(
    const ScenarioResult& r,
    const std::vector<state::TickState>& history,
    const std::string& replay_dir)
{
    const std::string sid = "stress_" + std::to_string(r.seed);
    state::SessionRecorder recorder(replay_dir, sid);
    recorder.set_config(nlohmann::json{
        {"seed", r.seed},
        {"profile_id", r.profile_id},
        {"mode", "stress"},
    });
    recorder.set_analytics(r.analytics);
    for (const auto& tick : history) recorder.record(tick);
    recorder.write_atomic();
}

// ---------------------------------------------------------------------------
// Diagnostic aggregation
// ---------------------------------------------------------------------------
struct KpiAgg {
    std::vector<double> samples;
    int failing_good  = 0;
    int failing_great = 0;
};

struct DiagAgg {
    std::array<KpiAgg, 6> kpis{};  // index matches kpi_thresholds() order

    // Rejection / false-reject breakdown totals
    long long r90_reject    = 0;
    long long residual_reject = 0;
    long long centroid_jump_reject = 0;
    long long passability_reject = 0;

    long long r90_false_reject         = 0;
    long long residual_false_reject    = 0;
    long long centroid_jump_false_reject    = 0;
    long long passability_false_reject = 0;

    long long false_accept_r90      = 0;
    long long false_accept_residual = 0;
    long long false_accept_centroid_jump = 0;

    double sensor_dropout_sum = 0.0;
    double neff_collapse_sum  = 0.0;
    int    n = 0;
};

void agg_push(DiagAgg& d, const state::SessionAnalytics& a) {
    const auto& thresholds = state::kpi_thresholds();
    for (size_t i = 0; i < thresholds.size(); ++i) {
        const auto& [id, thresh] = thresholds[i];
        double v = state::kpi_value(id, a);
        // Store raw value (use total_ticks as proxy for "never" on conv/reacq)
        if (id == state::KpiId::ReacqLatency && a.worst_reacquisition_ticks < 0)
            v = 0.0;  // no event = 0 latency for distribution purposes
        if (id == state::KpiId::ConvergenceLatency && a.cold_start_convergence_ticks < 0)
            v = static_cast<double>(a.total_ticks);
        d.kpis[i].samples.push_back(v);
        if (state::rate_kpi(id, a) == state::KpiRating::Failing) {
            d.kpis[i].failing_good++;
            d.kpis[i].failing_great++;
        } else if (state::rate_kpi(id, a) == state::KpiRating::Good) {
            d.kpis[i].failing_great++;
        }
    }

    d.r90_reject         += a.r90_reject_count;
    d.residual_reject    += a.residual_reject_count;
    d.centroid_jump_reject += a.centroid_jump_reject_count;
    d.passability_reject += a.passability_reject_count;

    d.r90_false_reject          += a.r90_false_reject_count;
    d.residual_false_reject     += a.residual_false_reject_count;
    d.centroid_jump_false_reject += a.centroid_jump_false_reject_count;
    d.passability_false_reject  += a.passability_false_reject_count;

    d.false_accept_r90           += a.false_accept_nearest_gate_r90;
    d.false_accept_residual      += a.false_accept_nearest_gate_residual;
    d.false_accept_centroid_jump += a.false_accept_nearest_gate_centroid_jump;

    d.sensor_dropout_sum += a.sensor_dropout_rate;
    d.neff_collapse_sum  += static_cast<double>(a.neff_collapse_count);
    d.n++;
}

double percentile(std::vector<double> v, double pct) {
    if (v.empty()) return 0.0;
    std::sort(v.begin(), v.end());
    const size_t idx = static_cast<size_t>(std::floor(pct * static_cast<double>(v.size())));
    return v[std::min(idx, v.size() - 1)];
}

double mean_of(const std::vector<double>& v) {
    if (v.empty()) return 0.0;
    return std::accumulate(v.begin(), v.end(), 0.0) / static_cast<double>(v.size());
}

static const char* kpi_name(state::KpiId id) {
    switch (id) {
        case state::KpiId::ConvergenceLatency: return "Convergence Latency (ticks)  ";
        case state::KpiId::PeakAcceptedError:  return "Peak Accepted Error (in)     ";
        case state::KpiId::GatePrecision:      return "Gate Precision               ";
        case state::KpiId::PostConvStdDev:     return "Post-Conv Std Dev (in)       ";
        case state::KpiId::GateRecall:         return "Gate Recall                  ";
        case state::KpiId::ReacqLatency:       return "Reacq Latency (ticks)        ";
    }
    return "Unknown";
}

static bool kpi_uses_p10(state::KpiId id) {
    // For "higher is better" KPIs, p10 is the informative tail
    return id == state::KpiId::GatePrecision || id == state::KpiId::GateRecall;
}

void print_diagnosis(const DiagAgg& d, int n_routes, int n_profiles) {
    const int n = d.n;
    std::printf(
        "\n=== DIAGNOSTIC REPORT (%d scenarios, %d routes × %d profiles) ===\n\n",
        n, n_routes, n_profiles);

    const auto& thresholds = state::kpi_thresholds();
    for (size_t i = 0; i < thresholds.size(); ++i) {
        const auto& [id, thresh] = thresholds[i];
        const auto& agg = d.kpis[i];
        const double mn  = mean_of(agg.samples);
        const double tail = kpi_uses_p10(id)
            ? percentile(agg.samples, 0.10)
            : percentile(agg.samples, 0.90);
        const int pct_good  = n > 0 ? static_cast<int>(100.0 * agg.failing_good  / n) : 0;
        const int pct_great = n > 0 ? static_cast<int>(100.0 * agg.failing_great / n) : 0;
        std::printf(
            "  %s  mean=%6.2f  %s=%6.2f  failing_good=%3d%%  failing_great=%3d%%\n",
            kpi_name(id), mn,
            kpi_uses_p10(id) ? "p10" : "p90",
            tail, pct_good, pct_great);
    }

    // Rejection breakdown
    const long long total_rej = d.r90_reject + d.residual_reject + d.centroid_jump_reject + d.passability_reject;
    const long long total_fr  = d.r90_false_reject + d.residual_false_reject
                               + d.centroid_jump_false_reject + d.passability_false_reject;
    std::printf("\n");
    if (total_rej > 0) {
        std::printf(
            "  Rejection causes:   r90=%3d%%  residual=%3d%%  centroid_jump=%3d%%  passability=%3d%%\n",
            static_cast<int>(100 * d.r90_reject              / total_rej),
            static_cast<int>(100 * d.residual_reject         / total_rej),
            static_cast<int>(100 * d.centroid_jump_reject    / total_rej),
            static_cast<int>(100 * d.passability_reject      / total_rej));
    }
    if (total_fr > 0) {
        std::printf(
            "  False-rej causes:   r90=%3d%%  residual=%3d%%  centroid_jump=%3d%%  passability=%3d%%\n",
            static_cast<int>(100 * d.r90_false_reject              / total_fr),
            static_cast<int>(100 * d.residual_false_reject         / total_fr),
            static_cast<int>(100 * d.centroid_jump_false_reject    / total_fr),
            static_cast<int>(100 * d.passability_false_reject      / total_fr));
    }
    const long long total_fa = d.false_accept_r90 + d.false_accept_residual + d.false_accept_centroid_jump;
    if (total_fa > 0) {
        std::printf(
            "  False-acc near:     r90=%3d%%  residual=%3d%%  centroid_jump=%3d%%\n",
            static_cast<int>(100 * d.false_accept_r90           / total_fa),
            static_cast<int>(100 * d.false_accept_residual      / total_fa),
            static_cast<int>(100 * d.false_accept_centroid_jump / total_fa));
    }
    const double mean_dropout = n > 0 ? d.sensor_dropout_sum / static_cast<double>(n) : 0.0;
    const double mean_neff_collapse = n > 0 ? d.neff_collapse_sum / static_cast<double>(n) : 0.0;
    std::printf("  Sensor dropout rate (mean): %.3f\n", mean_dropout);
    std::printf("  N_eff collapses   (mean):   %.1f\n", mean_neff_collapse);

    // Auto-diagnosis
    std::printf("\nDIAGNOSES:\n");
    int diag_n = 0;

    // Convergence latency
    {
        const double mn = mean_of(d.kpis[0].samples);
        const int pct_failing_good = n > 0 ? 100 * d.kpis[0].failing_good / n : 0;
        if (pct_failing_good > 30) {
            std::printf(
                "  [%d] Convergence latency failing_good=%d%%: slow cold-start.\n"
                "       → Check particle count, initial distribution width, or gate r90 ceiling.\n",
                ++diag_n, pct_failing_good);
        } else if (mn > 8.0) {
            std::printf(
                "  [%d] Convergence latency mean=%.1f (high but not majority failing).\n"
                "       → Consider increasing num_particles or widening initial particle spread.\n",
                ++diag_n, mn);
        }
    }

    // Gate r90 dominating rejections → too tight
    if (total_rej > 0 && d.r90_reject * 100 / total_rej > 50) {
        std::printf(
            "  [%d] r90 causes %lld%% of all rejections → gate r90 ceiling too tight.\n"
            "       → Loosen max_radius_90_in (currently 4.0 in) or run more MCL iterations.\n",
            ++diag_n,
            d.r90_reject * 100 / total_rej);
    }

    // Residual dominating false-rejects → sensor noise exceeds tolerance
    if (total_fr > 0 && d.residual_false_reject * 100 / total_fr > 40) {
        std::printf(
            "  [%d] Residual causes %lld%% of false rejects → sensor noise floor too high.\n"
            "       → Widen sensor_close_tolerance_in / sensor_far_tolerance_pct, or reduce gaussian_stddev_mm.\n",
            ++diag_n,
            d.residual_false_reject * 100 / total_fr);
    }

    // Gate precision failing
    {
        const int pct = n > 0 ? 100 * d.kpis[2].failing_good / n : 0;
        if (pct > 20) {
            std::printf(
                "  [%d] Gate precision failing_good=%d%% → gate passes bad estimates.\n"
                "       → Tighten max_radius_90_in or residual tolerance; check for glancing sensor geometry.\n",
                ++diag_n, pct);
        }
    }

    // Gate recall failing
    {
        const int pct = n > 0 ? 100 * d.kpis[4].failing_good / n : 0;
        if (pct > 20) {
            std::printf(
                "  [%d] Gate recall failing_good=%d%% → gate blocks many good estimates.\n"
                "       → Loosen one or more gate criteria; dominant false-reject cause shown above.\n",
                ++diag_n, pct);
        }
    }

    // Post-convergence stddev failing
    {
        const int pct = n > 0 ? 100 * d.kpis[3].failing_good / n : 0;
        if (pct > 25) {
            std::printf(
                "  [%d] Post-conv std dev failing_good=%d%% → estimate is jittery after lock.\n"
                "       → Check sensor noise levels, odom trans_noise_frac, or neff collapse rate (%.1f/run).\n",
                ++diag_n, pct, mean_neff_collapse);
        }
    }

    // Reacquisition latency
    {
        const int pct = n > 0 ? 100 * d.kpis[5].failing_good / n : 0;
        if (pct > 15) {
            std::printf(
                "  [%d] Reacquisition latency failing_good=%d%% → slow to recover after rejection streak.\n"
                "       → Increase injection fraction, widen recovery spread, or increase num_particles.\n",
                ++diag_n, pct);
        }
    }

    // High sensor dropout
    if (mean_dropout > 0.12) {
        std::printf(
            "  [%d] Mean sensor dropout rate=%.3f (>12%%) → sensors die too often.\n"
            "       → Reduce sensor_dead_prob or sensor_dead_max_duration_ticks.\n",
            ++diag_n, mean_dropout);
    }

    // r90 gate also dominates false-accept near-misses
    if (total_fa > 0 && d.false_accept_r90 * 100 / total_fa > 50) {
        std::printf(
            "  [%d] r90 is closest gate criterion on %lld%% of false accepts.\n"
            "       → The r90 ceiling lets through spread estimates; consider tightening it.\n",
            ++diag_n,
            d.false_accept_r90 * 100 / total_fa);
    }

    if (diag_n == 0) {
        std::printf("  All KPIs within target ranges — no dominant issues found.\n");
    }
    std::printf("\n");
}

} // anonymous namespace

int main(int argc, char** argv) {
    int n_scenarios      = 200;
    int total_ticks      = 2400;
    std::string replay_dir = "replays/stress";
    uint64_t base_seed   = 42;

    // Diag mode flags
    bool diag_mode        = false;
    int  diag_routes      = 10;
    int  diag_profiles    = 100;

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if      (a == "--scenarios"         && i + 1 < argc) n_scenarios  = std::atoi(argv[++i]);
        else if (a == "--ticks"             && i + 1 < argc) total_ticks  = std::atoi(argv[++i]);
        else if (a == "--replay-dir"        && i + 1 < argc) replay_dir   = argv[++i];
        else if (a == "--seed"              && i + 1 < argc) base_seed    = static_cast<uint64_t>(std::strtoull(argv[++i], nullptr, 10));
        else if (a == "--diag")                              diag_mode    = true;
        else if (a == "--routes"            && i + 1 < argc) diag_routes  = std::atoi(argv[++i]);
        else if (a == "--profiles-per-route"&& i + 1 < argc) diag_profiles= std::atoi(argv[++i]);
    }

    const bool tty = stdout_is_tty();

    // -----------------------------------------------------------------------
    // DIAGNOSTIC MODE: run N routes × M profiles, aggregate, print diagnosis
    // -----------------------------------------------------------------------
    if (diag_mode) {
        const int total = diag_routes * diag_profiles;
        std::printf(
            "diag: %d routes × %d profiles = %d scenarios, %d ticks each, base_seed=%llu\n",
            diag_routes, diag_profiles, total, total_ticks,
            static_cast<unsigned long long>(base_seed));
        std::fflush(stdout);

        DiagAgg agg;
        const auto t0 = std::chrono::steady_clock::now();
        const int report_every = tty ? 1 : std::max(1, total / 25);

        for (int ri = 0; ri < diag_routes; ++ri) {
            for (int pi = 0; pi < diag_profiles; ++pi) {
                const int si = ri * diag_profiles + pi;
                const uint64_t seed = base_seed
                    + static_cast<uint64_t>(ri) * 1000u
                    + static_cast<uint64_t>(pi);
                const int profile_id = pi % 6;
                ScenarioResult r = run_scenario(seed, profile_id, total_ticks);
                agg_push(agg, r.analytics);

                const int done = si + 1;
                const auto now = std::chrono::steady_clock::now();
                const double elapsed = std::chrono::duration<double>(now - t0).count();
                const double rate = elapsed / static_cast<double>(done);
                const double eta  = rate * static_cast<double>(total - done);

                if (tty) {
                    std::printf(
                        "\rdiag  %d/%d (%d%%)  %ds elapsed  ETA ~%ds",
                        done, total,
                        static_cast<int>(100.0 * done / total),
                        static_cast<int>(elapsed + 0.5),
                        static_cast<int>(eta + 0.5));
                    std::fflush(stdout);
                } else if (done == total || done % report_every == 0) {
                    std::printf(
                        "diag %d/%d (%d%%) %ds elapsed ETA ~%ds\n",
                        done, total,
                        static_cast<int>(100.0 * done / total),
                        static_cast<int>(elapsed + 0.5),
                        static_cast<int>(eta + 0.5));
                    std::fflush(stdout);
                }
            }
        }
        if (tty) { std::printf("\n"); std::fflush(stdout); }

        print_diagnosis(agg, diag_routes, diag_profiles);
        return 0;
    }

    // -----------------------------------------------------------------------
    // NORMAL MODE: single-pass generate + immediately write if not all-great
    // -----------------------------------------------------------------------
    std::printf(
        "stress: %d scenarios, %d ticks each, dir=%s, base_seed=%llu\n",
        n_scenarios, total_ticks, replay_dir.c_str(),
        static_cast<unsigned long long>(base_seed));
    std::fflush(stdout);

    std::error_code ec;
    std::filesystem::create_directories(replay_dir, ec);
    (void)ec;

    // Analytics-only summary (no histories kept after writing)
    std::vector<ScenarioResult> summary;
    summary.reserve(static_cast<size_t>(n_scenarios));

    int written = 0;
    int skipped = 0;
    const auto t0 = std::chrono::steady_clock::now();
    const int report_every = tty ? 1 : std::max(1, n_scenarios / 25);

    for (int si = 0; si < n_scenarios; ++si) {
        const uint64_t seed = base_seed + static_cast<uint64_t>(si);
        const int profile_id = si % 6;

        std::vector<state::TickState> history;
        ScenarioResult r = run_scenario(seed, profile_id, total_ticks, &history);

        const bool interesting = !state::is_all_great(r.analytics);
        if (interesting) {
            write_replay(r, history, replay_dir);
            written++;
        } else {
            skipped++;
        }
        // history is freed here — no bulk in-memory storage
        summary.push_back({r.seed, r.profile_id, r.analytics});

        const int done = si + 1;
        const auto now = std::chrono::steady_clock::now();
        const double elapsed = std::chrono::duration<double>(now - t0).count();
        const double rate = elapsed / static_cast<double>(done);
        const double eta  = rate * static_cast<double>(n_scenarios - done);

        if (tty) {
            std::printf(
                "\rsimulate %d/%d (%d%%)  %ds elapsed  ETA ~%ds  |  written=%d skipped=%d  last #%d p%d peak=%.2f prec=%.3f recall=%.3f",
                done, n_scenarios,
                static_cast<int>(100.0 * done / n_scenarios),
                static_cast<int>(elapsed + 0.5),
                static_cast<int>(eta + 0.5),
                written, skipped,
                si, profile_id,
                r.analytics.peak_accepted_error,
                r.analytics.gate_precision,
                r.analytics.gate_recall);
            std::fflush(stdout);
        } else if (done == n_scenarios || done % report_every == 0) {
            std::printf(
                "stress simulate %d/%d (%d%%) %ds elapsed ETA ~%ds | written=%d skipped=%d last peak=%.2f\n",
                done, n_scenarios,
                static_cast<int>(100.0 * done / n_scenarios),
                static_cast<int>(elapsed + 0.5),
                static_cast<int>(eta + 0.5),
                written, skipped,
                r.analytics.peak_accepted_error);
            std::fflush(stdout);
        }
    }
    if (tty) { std::printf("\n"); std::fflush(stdout); }

    // Write summary JSON (analytics-only, ~1 KB per entry)
    nlohmann::json jsummary = nlohmann::json::array();
    for (const auto& r : summary) {
        jsummary.push_back(nlohmann::json{
            {"seed", r.seed},
            {"profile_id", r.profile_id},
            {"all_great", state::is_all_great(r.analytics)},
            {"analytics", r.analytics},
        });
    }
    const std::string report_path = replay_dir + "/summary_report.json";
    std::ofstream rep(report_path);
    rep << jsummary.dump(2);

    std::printf(
        "wrote %d replays, skipped %d (all-great), summary at %s\n",
        written, skipped, report_path.c_str());
    return 0;
}
