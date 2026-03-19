#include "chaos/chaos_runner.hpp"

#include "nlohmann/json.hpp"
#include "state/session_recorder.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <numeric>
#include <random>
#include <sstream>

namespace chaos {

ChaosRunner::ChaosRunner(const ChaosConfig& config)
    : config_(config) {}

std::vector<sim::Action> ChaosRunner::generate_path(const std::string& path_type, int ticks, uint64_t seed) const {
    std::vector<sim::Action> actions;
    actions.reserve(static_cast<size_t>(ticks));
    std::mt19937 rng(static_cast<std::mt19937::result_type>(seed));
    std::uniform_int_distribution<int> pause(3, 10);
    std::uniform_int_distribution<int> step(5, 15);
    std::uniform_int_distribution<int> which(0, 3);

    if (path_type == "random_walk") {
        for (int i = 0; i < ticks;) {
            const sim::Action a = static_cast<sim::Action>(which(rng));
            const int n = std::min(step(rng), ticks - i);
            for (int j = 0; j < n; ++j) actions.push_back(a);
            i += n;
        }
        return actions;
    }

    if (path_type == "spiral") {
        for (int i = 0; i < ticks; ++i) {
            actions.push_back((i % 3 == 2) ? sim::Action::ROTATE_CW : sim::Action::FORWARD);
        }
        return actions;
    }

    if (path_type == "stop_and_go") {
        while (static_cast<int>(actions.size()) < ticks) {
            for (int i = 0; i < 10 && static_cast<int>(actions.size()) < ticks; ++i) actions.push_back(sim::Action::FORWARD);
            const int p = pause(rng);
            for (int i = 0; i < p && static_cast<int>(actions.size()) < ticks; ++i) actions.push_back(sim::Action::NONE);
        }
        return actions;
    }

    if (path_type == "wall_hugger") {
        while (static_cast<int>(actions.size()) < ticks) {
            for (int i = 0; i < 15 && static_cast<int>(actions.size()) < ticks; ++i) actions.push_back(sim::Action::FORWARD);
            for (int i = 0; i < 5 && static_cast<int>(actions.size()) < ticks; ++i) actions.push_back(sim::Action::ROTATE_CW);
        }
        return actions;
    }

    // obstacle_slalom
    while (static_cast<int>(actions.size()) < ticks) {
        for (int i = 0; i < 8 && static_cast<int>(actions.size()) < ticks; ++i) actions.push_back(sim::Action::FORWARD);
        for (int i = 0; i < 2 && static_cast<int>(actions.size()) < ticks; ++i) actions.push_back(sim::Action::ROTATE_CW);
        for (int i = 0; i < 8 && static_cast<int>(actions.size()) < ticks; ++i) actions.push_back(sim::Action::FORWARD);
        for (int i = 0; i < 2 && static_cast<int>(actions.size()) < ticks; ++i) actions.push_back(sim::Action::ROTATE_CCW);
    }
    return actions;
}

bool ChaosRunner::detect_convergence(const std::vector<state::TickState>& history, int idx) const {
    if (idx < 4) return false;
    for (int i = idx - 4; i <= idx; ++i) {
        const auto& particles = history[static_cast<size_t>(i)].post_resample.particles;
        if (particles.empty()) return false;
        double mx = 0.0;
        double my = 0.0;
        for (const auto& p : particles) {
            mx += p.x;
            my += p.y;
        }
        mx /= particles.size();
        my /= particles.size();
        double var = 0.0;
        for (const auto& p : particles) {
            const double dx = p.x - mx;
            const double dy = p.y - my;
            var += dx * dx + dy * dy;
        }
        var /= particles.size();
        if (var >= 25.0) return false;
    }
    return true;
}

RunResult ChaosRunner::run_single(
    int run_id,
    std::vector<state::TickState>* out_history,
    state::SimSessionConfig* out_config) {
    RunResult rr;
    rr.run_id = run_id;
    rr.seed = config_.base_seed + static_cast<uint64_t>(run_id);

    std::vector<std::string> path_types = {
        "random_walk", "wall_hugger", "spiral", "obstacle_slalom", "stop_and_go"
    };
    rr.path_type = path_types[static_cast<size_t>(run_id % static_cast<int>(path_types.size()))];
    const auto actions = generate_path(rr.path_type, config_.max_ticks, rr.seed);

    state::SimSessionConfig cfg;
    cfg.seed = rr.seed;

    if (rr.path_type == "obstacle_slalom") {
        cfg.field.obstacles.push_back(sim::AABB{-15.0, -10.0, -5.0, 10.0});
        cfg.field.obstacles.push_back(sim::AABB{5.0, 15.0, 15.0, 35.0});
        cfg.field.obstacles.push_back(sim::AABB{5.0, -35.0, 15.0, -15.0});
    }

    state::SimSession session(cfg);
    if ((run_id % 3) == 0) {
        noise::FailureEvent e;
        e.start_tick = 20;
        e.duration_ticks = 15;
        e.type = noise::FailureType::SensorDead;
        e.sensor_idx = run_id % 4;
        session.schedule_failure(e);
        rr.total_failures = 1;
    }

    for (sim::Action a : actions) {
        session.tick(a);
    }

    const auto& history = session.history();
    if (out_history) *out_history = history;
    if (out_config) *out_config = cfg;
    if (history.empty()) return rr;

    int convergence_tick = -1;
    for (int i = 0; i < static_cast<int>(history.size()); ++i) {
        if (detect_convergence(history, i)) {
            convergence_tick = i;
            break;
        }
    }
    rr.convergence_tick = convergence_tick;

    const int start = (convergence_tick >= 0 ? convergence_tick : std::min(100, static_cast<int>(history.size()) - 1));
    double sumsq = 0.0;
    int count = 0;
    for (int i = start; i < static_cast<int>(history.size()); ++i) {
        const double e = history[static_cast<size_t>(i)].mcl_error;
        sumsq += e * e;
        count++;
    }
    if (count > 0) rr.post_convergence_rms = std::sqrt(sumsq / count);

    const auto& last = history.back();
    rr.final_mcl_error = last.mcl_error;
    rr.final_odom_error = last.odom_error;
    return rr;
}

std::vector<RunResult> ChaosRunner::run_all() {
    std::vector<std::vector<state::TickState>> histories(static_cast<size_t>(config_.num_runs));
    std::vector<state::SimSessionConfig> configs(static_cast<size_t>(config_.num_runs));

    std::vector<RunResult> out;
    out.reserve(static_cast<size_t>(config_.num_runs));
    for (int i = 0; i < config_.num_runs; ++i) {
        out.push_back(run_single(i, &histories[static_cast<size_t>(i)], &configs[static_cast<size_t>(i)]));
    }
    std::sort(out.begin(), out.end(), [](const RunResult& a, const RunResult& b) {
        const double sa = std::isfinite(a.post_convergence_rms)
            ? a.post_convergence_rms
            : std::numeric_limits<double>::infinity();
        const double sb = std::isfinite(b.post_convergence_rms)
            ? b.post_convergence_rms
            : std::numeric_limits<double>::infinity();
        if (sa == sb) return a.run_id < b.run_id;
        return sa > sb;
    });

    const int save_count = std::min(config_.save_top_n, static_cast<int>(out.size()));
    for (int i = 0; i < save_count; ++i) {
        RunResult& r = out[static_cast<size_t>(i)];
        std::ostringstream sid;
        sid << "chaos_run_" << r.run_id << "_seed_" << r.seed;
        state::SessionRecorder recorder(config_.replay_output_dir, sid.str());
        recorder.set_config(nlohmann::json{
            {"seed", r.seed},
            {"num_particles", configs[static_cast<size_t>(r.run_id)].mcl_config.num_particles},
            {"path_type", r.path_type},
            {"convergence_tick", r.convergence_tick},
            {"post_convergence_rms", r.post_convergence_rms},
        });
        nlohmann::json obstacles = nlohmann::json::array();
        for (const auto& o : configs[static_cast<size_t>(r.run_id)].field.obstacles) {
            obstacles.push_back({
                {"min_x", o.min_x},
                {"min_y", o.min_y},
                {"max_x", o.max_x},
                {"max_y", o.max_y},
            });
        }
        recorder.set_obstacles(obstacles);
        for (const auto& t : histories[static_cast<size_t>(r.run_id)]) {
            recorder.record(t);
        }
        if (recorder.write_atomic()) {
            r.replay_file = sid.str() + ".json";
        }
    }

    nlohmann::json j;
    j["num_runs"] = config_.num_runs;
    j["base_seed"] = config_.base_seed;
    j["results"] = nlohmann::json::array();
    for (const auto& r : out) {
        j["results"].push_back({
            { "run_id", r.run_id },
            { "seed", r.seed },
            { "path_type", r.path_type },
            { "convergence_tick", r.convergence_tick },
            { "post_convergence_rms", r.post_convergence_rms },
            { "final_mcl_error", r.final_mcl_error },
            { "final_odom_error", r.final_odom_error },
            { "total_failures", r.total_failures },
            { "replay_file", r.replay_file },
        });
    }

    std::ofstream report(config_.output_report_path, std::ios::trunc);
    if (report.is_open()) {
        report << j.dump(2);
    }
    return out;
}

} // namespace chaos
