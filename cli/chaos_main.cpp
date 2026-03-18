#include "chaos/chaos_runner.hpp"

#include <cstdlib>
#include <iostream>

int main(int argc, char** argv) {
    chaos::ChaosConfig cfg;
    if (argc >= 2) cfg.num_runs = std::atoi(argv[1]);
    if (argc >= 3) cfg.base_seed = static_cast<uint64_t>(std::strtoull(argv[2], nullptr, 10));
    if (argc >= 4) cfg.output_report_path = argv[3];

    chaos::ChaosRunner runner(cfg);
    const auto results = runner.run_all();
    std::cout << "Chaos runs: " << results.size() << "\n";
    if (!results.empty()) {
        std::cout << "Worst RMS: " << results.front().post_convergence_rms << "\n";
    }
    std::cout << "Report: " << cfg.output_report_path << "\n";
    return 0;
}
