# MCL Simulation

Monte Carlo Localization (MCL) simulation engine with a physics simulator, sensor models, route pursuit, chaos testing, and a Next.js replay viewer.

## Project Structure

```
├── cli/                  # CLI entrypoints (server, chaos, route)
├── frontend/             # Next.js replay viewer & route editor
├── include/              # Public C++ headers
├── replays/              # Recorded replay JSON files
├── routes/               # Route definition JSON files
├── src/                  # C++ implementation
│   ├── mcl/              #   MCL engine & controller
│   ├── sim/              #   Physics, field, sensor model
│   ├── noise/            #   Noise generation & failure injection
│   ├── pursuit/          #   Pure pursuit & route runner
│   ├── ray/              #   Ray casting with obstacles
│   ├── state/            #   Session, recorder, replay loader
│   ├── chaos/            #   Chaos runner
│   ├── server/           #   HTTP API
│   └── Makefile
├── tests/                # doctest unit tests
└── third_party/          # Vendored headers (nlohmann/json, doctest, httplib)
```

## Prerequisites

### macOS

Install Xcode Command Line Tools (provides `clang++` as `g++` and `make`):

```bash
xcode-select --install
```

Verify installation:

```bash
g++ --version   # Apple clang 15+ with C++17 support required
make --version  # GNU Make 3.81+
```

### Ubuntu / Debian

```bash
sudo apt update
sudo apt install build-essential
```

This installs `g++` (GCC 11+) and `make`. Verify:

```bash
g++ --version   # GCC 11+ required for full C++17 support
make --version  # GNU Make 4.3+
```

### Windows (WSL)

Use Windows Subsystem for Linux and follow the Ubuntu instructions above.

### Compiler Requirements

| Tool   | Minimum Version           |
|--------|---------------------------|
| `g++`  | GCC 11 or Apple Clang 15  |
| `make` | GNU Make 3.81             |
| C++    | C++17 (`-std=c++17`)      |

All third-party C++ dependencies are vendored in `third_party/` — no package manager needed:

- [nlohmann/json](https://github.com/nlohmann/json) — JSON parsing and serialization
- [doctest](https://github.com/doctest/doctest) — Unit test framework
- [cpp-httplib](https://github.com/yhirose/cpp-httplib) — Header-only HTTP server

## Building

All build commands run from the `src/` directory:

```bash
cd src
```

### Run Tests

```bash
make test
```

This compiles all source and test files and runs `./test_runner`. Expected output:

```
[doctest] test cases:    218 |    218 passed | 0 failed | 0 skipped
[doctest] assertions: 142068 | 142068 passed | 0 failed |
[doctest] Status: SUCCESS!
```

### Build the HTTP Server

```bash
make server
```

Run it:

```bash
./server_bin [port] [replay_dir]
# defaults: port=8080, replay_dir=replays
```

### Build the Route Runner

```bash
make route
```

Run a route:

```bash
./route_bin <route.json> [--seed N] [--replay-dir DIR]
```

Example:

```bash
./route_bin ../routes/figure_eight.json --seed 44 --replay-dir ../replays
```

### Build the Chaos Runner

```bash
make chaos
```

Run chaos testing:

```bash
./chaos_bin [num_runs] [base_seed] [report_path] [replay_dir] [save_top_n]
```

### Clean Build Artifacts

```bash
make clean
```

Removes all `.o` files and binaries (`test_runner`, `server_bin`, `chaos_bin`, `route_bin`).

### Full Clean Rebuild

```bash
make clean && make test
```

## Build Targets Summary

| Command      | Binary        | Description                              |
|--------------|---------------|------------------------------------------|
| `make test`  | `test_runner` | Compile and run all unit tests           |
| `make server`| `server_bin`  | HTTP API server for sim sessions         |
| `make route` | `route_bin`   | Run a single route with failure injection|
| `make chaos` | `chaos_bin`   | Batch chaos/robustness testing           |
| `make clean` | —             | Remove all build artifacts               |

## Compiler Flags

```
-std=c++17 -O2 -Wall -Wextra -pedantic
```

The `server`, `chaos`, and `route` binaries additionally link with `-lpthread`.

## Frontend (Replay Viewer)

### Prerequisites

- [Node.js](https://nodejs.org/) 18+ and npm

### Setup

```bash
cd frontend
npm install
```

### Development

```bash
npm run dev
```

Opens at [http://localhost:3000](http://localhost:3000). The frontend expects the C++ server running on port 8080.

### Production Build

```bash
npm run build
npm run start
```

## Tests

The test suite uses [doctest](https://github.com/doctest/doctest) and covers:

| Area                    | Test Files                                                   |
|-------------------------|--------------------------------------------------------------|
| MCL core                | `test_mcl_stationary`, `test_mcl_moving`, `test_mcl_obstacles` |
| MCL noise               | `test_mcl_odom_noise`, `test_mcl_noisy_sensors`             |
| MCL controller          | `test_mcl_controller`, `test_confidence_gate`, `test_cluster_stats` |
| Ray casting             | `test_ray_cast`, `test_ray_cast_obstacles`                   |
| Physics                 | `test_physics`                                               |
| Sensors                 | `test_sensor_model`, `test_noise_generator`                  |
| Session & replay        | `test_tick_state`, `test_session_recorder`, `test_replay_loader` |
| Route pursuit           | `test_pure_pursuit`, `test_route_def`, `test_route_runner`   |
| Failure injection       | `test_failure_injector`, `test_failure_injector_ext`, `test_random_failures` |
| Chaos                   | `test_chaos_runner`                                          |
| Stepping modes          | `test_step_continuous`, `test_session_continuous`            |

Run a single test case by name:

```bash
./test_runner -tc="name of test case"
```

List all test cases:

```bash
./test_runner -ltc
```

## Troubleshooting

**`g++: command not found`**
Install a C++ compiler. On macOS: `xcode-select --install`. On Ubuntu: `sudo apt install build-essential`.

**`make: command not found`**
Same fix as above — both `make` and `g++` come with the system build tools.

**Linker errors about `pthread`**
Only the `server`, `chaos`, and `route` targets need pthreads. The `test` target does not link it. If you see pthread errors on `make test`, your system may need `-lpthread` added to the test link step.

**`fatal error: 'nlohmann/json.hpp' file not found`**
Make sure you're running `make` from inside the `src/` directory, not the project root. The include paths are relative to `src/`.

**Frontend: `ECONNREFUSED` on API calls**
Start the C++ server first: `cd src && make server && ./server_bin`.
