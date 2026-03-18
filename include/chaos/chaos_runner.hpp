#pragma once

#include "state/sim_session.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace chaos {

struct ChaosConfig {
    int num_runs = 100;
    uint64_t base_seed = 42;
    int max_ticks = 200;
    std::string output_report_path = "chaos_report.json";
    int save_top_n = 20;
};

struct RunResult {
    int run_id = 0;
    uint64_t seed = 0;
    std::string path_type;
    int convergence_tick = -1;
    double post_convergence_rms = 1e9;
    double final_mcl_error = 0.0;
    double final_odom_error = 0.0;
    int total_failures = 0;
};

class ChaosRunner {
public:
    explicit ChaosRunner(const ChaosConfig& config = {});
    std::vector<RunResult> run_all();

private:
    RunResult run_single(int run_id);
    std::vector<sim::Action> generate_path(const std::string& path_type, int ticks, uint64_t seed) const;
    bool detect_convergence(const std::vector<state::TickState>& history, int idx) const;

    ChaosConfig config_;
};

} // namespace chaos
