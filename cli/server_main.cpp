#include "server/api.hpp"

#include <cstdlib>
#include <iostream>

int main(int argc, char** argv) {
    int port = 8080;
    std::string replay_dir = "replays";
    std::string mcl_replay_dir = "replay_mcl";

    if (argc >= 2) port = std::atoi(argv[1]);
    if (argc >= 3) replay_dir = argv[2];
    if (argc >= 4) mcl_replay_dir = argv[3];

    std::cout << "Starting server on port " << port
              << " with replay dir '" << replay_dir
              << "' and mcl replay dir '" << mcl_replay_dir << "'\n";
    server::SimServer sim_server(port, replay_dir, mcl_replay_dir);
    sim_server.run();
    return 0;
}
