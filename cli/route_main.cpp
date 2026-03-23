#include "pursuit/route.hpp"
#include "pursuit/route_runner.hpp"

#include <cstdint>
#include <cstdlib>
#include <exception>
#include <iostream>

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: ./route_bin <route.json> [--seed N] [--replay-dir DIR] [--mcl-replay-dir DIR]\n";
        return 1;
    }

    std::string route_path = argv[1];
    std::string replay_dir = "../replays";
    std::string mcl_replay_dir = "../replay_mcl";
    bool seed_override = false;
    uint64_t seed_value = 42;

    for (int i = 2; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--seed" && i + 1 < argc) {
            seed_value = static_cast<uint64_t>(std::strtoull(argv[++i], nullptr, 10));
            seed_override = true;
            continue;
        }
        if (arg == "--replay-dir" && i + 1 < argc) {
            replay_dir = argv[++i];
            continue;
        }
        if (arg == "--mcl-replay-dir" && i + 1 < argc) {
            mcl_replay_dir = argv[++i];
            continue;
        }
    }

    try {
        pursuit::RouteDefinition route = pursuit::load_route(route_path);
        if (seed_override) route.failure_seed = seed_value;

        pursuit::RouteRunner runner;
        const pursuit::RouteResult result = runner.run(route, replay_dir, mcl_replay_dir);
        std::cout << "Route: " << result.route_name << "\n";
        std::cout << "Seed: " << result.seed << "\n";
        std::cout << "Ticks: " << result.total_ticks << "\n";
        std::cout << "Waypoints reached: " << result.waypoints_reached << "\n";
        std::cout << "Completed: " << (result.completed ? "true" : "false") << "\n";
        std::cout << "Final MCL error: " << result.final_mcl_error << "\n";
        std::cout << "Mean MCL error: " << result.mean_mcl_error << "\n";
        std::cout << "Replay: " << result.replay_file << "\n";
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << "route run failed: " << ex.what() << "\n";
        return 2;
    }
}

