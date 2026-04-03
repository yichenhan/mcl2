#include "server/api.hpp"

#include <cstdlib>
#include <iostream>

int main(int argc, char** argv) {
    int port = 8080;
    std::string replay_dir = "sessions";

    if (argc >= 2) port = std::atoi(argv[1]);
    if (argc >= 3) replay_dir = argv[2];

    std::cout << "Starting server on port " << port
              << " with replay dir '" << replay_dir
              << "' (GET /api/replays also lists replays/stress/*.json)\n";
    server::SimServer sim_server(port, replay_dir);
    sim_server.run();
    return 0;
}
