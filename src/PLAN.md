# MCL Simulator — Implementation Plan

## Context

Building a standalone C++ MCL (Monte Carlo Localization) simulator for a VEX robot per `edd.md`. Phase 1 (ray-cast math) is committed. Phase 2 (stationary MCL) is written but has 5 failing tests due to a scoring bug. Phases 3–8 are not started.

**Current state:** 28/33 tests pass. 5 failures all in Phase 2 MCL tests.

---

## Phase 2 Fix: Stationary MCL Scoring Bug

### Root Cause

When ALL sensors exceed the 6" error cap for a particle, it gets **neutral weight (1/N)** instead of near-zero weight. With 150 particles over a 144×144" field, most particles are far from truth → all sensor errors exceed cap → neutral weight → no differentiation → no convergence.

Concrete example (N=2): truth particle gets exp(0)=1.0, far particle gets neutral 0.5. After normalization: 0.667 vs 0.333. Tests expect >0.9 and <0.1.

### Fix

In `src/mcl/mcl_engine.cpp`, change the `inlier_count == 0` case from neutral weight to **near-zero weight**:

```cpp
if (inlier_count == 0) {
    // All sensors exceeded cap — this particle is far from truth.
    // Give near-zero weight (not neutral) so it gets eliminated in resampling.
    // Only truly invalid readings (-1) get uniform weights (handled above).
    p.weight = 1e-30f;
}
```

This preserves the outlier-robust scoring for obstacle handling (Phase 6) while allowing convergence: far particles get crushed, truth-adjacent particles dominate.

### Files to Modify
- `src/mcl/mcl_engine.cpp` — change neutral weight to epsilon weight (line ~131)

### Expected Result
All 12 Phase 2 tests pass (33/33 total).

---

## Phase 3: MCL with Movement (No Noise)

### Goal
MCL tracks a moving robot using perfect odometry and perfect sensors.

### New Files
| File | Purpose |
|------|---------|
| `include/sim/field.hpp` | Field class — boundary check, no obstacles yet |
| `src/sim/field.cpp` | Field implementation |
| `include/sim/physics.hpp` | Physics module — actions → state + MotionDelta |
| `src/sim/physics.cpp` | Physics implementation |
| `tests/test_physics.cpp` | Physics unit tests |
| `tests/test_mcl_moving.cpp` | MCL tracking tests with movement |

### Modified Files
| File | Change |
|------|--------|
| `src/mcl/mcl_engine.cpp` | Implement predict step (apply odom delta to particles) |
| `Makefile` | Add new source and test files |

### Key Structures

```cpp
// include/sim/field.hpp
namespace sim {
struct Field {
    double field_half = 72.0;
    double robot_radius = 8.0;
    bool is_inside(double x, double y) const;
    // Clamp position to stay within bounds (accounting for robot radius)
    void clamp(double& x, double& y) const;
};
}

// include/sim/physics.hpp
namespace sim {
enum class Action { FORWARD, BACKWARD, ROTATE_CW, ROTATE_CCW, NONE };

struct RobotState {
    double x, y;        // field coordinates (inches)
    double heading_deg;  // VEX convention: 0=+Y, CW positive
};

struct MotionDelta {
    double forward_in;    // distance moved forward (inches)
    double rotation_deg;  // heading change (degrees)
};

struct PhysicsConfig {
    double max_velocity = 36.0;     // in/s
    double max_angular_vel = 360.0; // deg/s
    double dt = 0.05;               // tick duration (seconds)
};

struct StepResult {
    MotionDelta delta;
    bool colliding;
};

class Physics {
public:
    Physics(const Field& field, const PhysicsConfig& config = {});
    StepResult step(Action action); // updates internal state, returns delta
    const RobotState& state() const;
    void set_state(const RobotState& s);
private:
    Field field_;
    PhysicsConfig config_;
    RobotState state_;
};
}
```

### Predict Step Implementation

In `MCLEngine::predict()`, when `delta_forward != 0`:
```cpp
// Convert odom delta from local frame to field frame using heading
double heading_rad = distance_loc::deg2rad(heading_deg);
// VEX convention: 0°=+Y, CW positive
// dx = forward * sin(heading), dy = forward * cos(heading)
double dx = delta_forward * std::sin(heading_rad);
double dy = delta_forward * std::cos(heading_rad);

for (auto& p : particles_) {
    p.x += static_cast<float>(dx);
    p.y += static_cast<float>(dy);
}
```

### Heading Wrap Helper
```cpp
static double wrap_heading(double deg) {
    deg = std::fmod(deg, 360.0);
    if (deg < 0) deg += 360.0;
    return deg;
}
```

### Tests — Physics
- Forward at center heading 0°: y increases by 36 * 0.05 = 1.8 in/tick
- Forward into wall: snaps back, collision flag true, forward_in = actual distance
- Rotation at 0°/360° boundary: wraps correctly
- No action: state unchanged, delta is (0, 0)

### Tests — MCL Tracking
- Robot moves forward 10 ticks: MCL estimate tracks within 2"
- Robot rotates 90° then moves forward: MCL tracks through the turn
- Robot moves to wall and collides: MCL handles stopped motion
- Robot moves in a square: MCL tracks the full path

---

## Phase 4: Odometry Noise

### Goal
MCL still converges when odometry is noisy.

### New Files
| File | Purpose |
|------|---------|
| `include/noise/noise_generator.hpp` | Deterministic seeded noise wrapper |
| `src/noise/noise_generator.cpp` | Implementation |
| `tests/test_noise_generator.cpp` | Noise generator tests |
| `include/sim/sensor_model.hpp` | SensorModel class (odom noise part) |
| `src/sim/sensor_model.cpp` | Implementation (odom noise pipeline) |
| `tests/test_mcl_odom_noise.cpp` | MCL with noisy odom tests |

### Modified Files
| File | Change |
|------|--------|
| `include/mcl/mcl_engine.hpp` | Add predict noise parameters to MCLConfig |
| `src/mcl/mcl_engine.cpp` | Add gaussian noise to predict step |
| `Makefile` | Add new sources |

### Key Structures

```cpp
// include/noise/noise_generator.hpp
namespace noise {
class NoiseGenerator {
public:
    explicit NoiseGenerator(uint64_t seed);
    double gaussian(double mean, double stddev);
    double uniform(double lo, double hi);
    bool bernoulli(double probability);
    void reseed(uint64_t seed);
private:
    std::mt19937 rng_;
};
}

// include/sim/sensor_model.hpp
namespace sim {
struct OdomNoiseConfig {
    double trans_noise_frac = 0.03;    // forward noise proportional to distance
    double drift_noise_frac = 0.01;    // lateral noise proportional to distance
    double drift_per_tick_in = 0.01;   // constant drift bias per tick
};

class SensorModel {
public:
    SensorModel(uint64_t seed, const OdomNoiseConfig& odom_config = {});

    // Apply odom noise to a perfect delta, return noisy delta
    MotionDelta apply_odom_noise(const MotionDelta& true_delta, double heading_deg);

    // ... sensor noise added in Phase 5 ...
private:
    noise::NoiseGenerator rng_;
    OdomNoiseConfig odom_config_;
};
}
```

### Odom Noise Pipeline
1. Receive `MotionDelta { forward_in, rotation_deg }`
2. Add proportional noise in local frame:
   - `noisy_forward = forward_in + N(0, (α × |forward_in|)²)`
   - `lateral = N(0, (α_drift × |forward_in|)²)`
3. Add constant drift bias: `lateral += drift_per_tick`
4. Rotate (forward, lateral) to field frame via IMU heading
5. Accumulate into odom pose

### MCL Predict Noise
Add to `MCLConfig`:
```cpp
double predict_noise_fwd = 0.02;   // per-particle forward noise fraction
double predict_noise_lat = 0.01;   // per-particle lateral noise fraction
```

In predict, for each particle: add gaussian noise proportional to `|delta_forward|`.

---

## Phase 5: Sensor Noise & Dropouts

### Goal
MCL works with realistic sensor noise, dropouts, range limits, spurious reflections.

### New Files
| File | Purpose |
|------|---------|
| `tests/test_sensor_model.cpp` | Sensor model unit tests |
| `tests/test_mcl_noisy_sensors.cpp` | MCL with full noise tests |

### Modified Files
| File | Change |
|------|--------|
| `include/sim/sensor_model.hpp` | Add sensor noise config, distance sensor pipeline |
| `src/sim/sensor_model.cpp` | Implement full sensor pipeline |
| `Makefile` | Add new test files |

### Sensor Pipeline (per sensor, per tick)
```
1. Long dropout check → if active, return -1, decrement counter
2. Random dropout (bernoulli) → -1, optionally start long dropout
3. Ray-cast from ground-truth pose (walls only, no obstacles yet)
4. Convert to mm
5. Range gate: > 2000 mm → -1
6. Range noise ramp: > 1700 mm → add extra gaussian
7. Base gaussian noise: N(0, σ²)
8. Spurious reflection: with prob p, replace with true_dist × uniform(0.3, 0.9)
9. Clamp to [0, 9999] or -1
```

### Noise Profile Defaults
```cpp
struct SensorNoiseConfig {
    double gaussian_stddev_mm = 15.0;
    double dropout_probability = 0.02;
    double long_dropout_probability = 0.005;
    int long_dropout_min_ticks = 10;
    int long_dropout_max_ticks = 60;
    double range_noise_start_mm = 1700.0;
    double range_max_mm = 2000.0;
    double range_noise_slope = 0.05;
    double spurious_reflection_probability = 0.01;
    double imu_noise_stddev_deg = 0.1;
    int collision_stall_ticks = 5;
};
```

### IMU Noise
`observed_heading = ground_truth_heading + N(0, imu_noise_stddev²)`
No cumulative drift — just per-tick gaussian.

### Collision Stall
After physics collision, odom delta → 0 for `collision_stall_ticks` ticks.

---

## Phase 6: Obstacles

### Goal
Obstacles affect physics and ground-truth sensors but MCL remains obstacle-blind.

### New Files
| File | Purpose |
|------|---------|
| `include/ray/ray_cast_obstacles.hpp` | Ray-cast with AABB obstacles |
| `src/ray/ray_cast_obstacles.cpp` | Implementation |
| `tests/test_ray_cast_obstacles.cpp` | Obstacle ray-cast tests |
| `tests/test_mcl_obstacles.cpp` | MCL with obstacles tests |

### Modified Files
| File | Change |
|------|--------|
| `include/sim/field.hpp` | Add obstacle list, is_passable check |
| `src/sim/field.cpp` | Obstacle collision logic |
| `src/sim/sensor_model.cpp` | Use ray_cast_obstacles for ground-truth readings |
| `src/sim/physics.cpp` | Collide with obstacles |
| `Makefile` | Add new sources |

### AABB Obstacle Structure
```cpp
struct AABB {
    double min_x, min_y, max_x, max_y;
};
```

### Ray-Cast with Obstacles
Extend ray-cast to intersect AABB obstacle faces. Return min(wall distance, obstacle distance).

```cpp
double ray_distance_with_obstacles(
    const distance_loc::Vec2& robot_pos,
    double theta_deg,
    const distance_loc::Vec2& sensor_offset,
    double sensor_rel_deg,
    const std::vector<AABB>& obstacles
);
```

**Critical asymmetry:** Sensor model uses `ray_distance_with_obstacles` for ground-truth. MCL stays on `distance_loc::ray_distance_with_offset` (walls only).

---

## Phase 7: State Recording, Server, Chaos Suite

### Goal
Wire everything into a Crow HTTP server, record sessions to JSON, build chaos CLI.

### Dependencies to Add
- **nlohmann/json** (header-only) → `third_party/nlohmann/json.hpp`
- **Crow** (header-only) → `third_party/crow/` (includes crow_all.h)
  - Crow requires boost::asio or standalone asio — use standalone asio
  - `third_party/asio/` for standalone ASIO headers

### New Files
| File | Purpose |
|------|---------|
| `include/state/tick_state.hpp` | TickState struct |
| `include/state/session_recorder.hpp` | SessionRecorder (accumulate → JSON) |
| `src/state/session_recorder.cpp` | Implementation with atomic writes |
| `include/state/replay_loader.hpp` | ReplayLoader (JSON → structs) |
| `src/state/replay_loader.cpp` | Paginated tick access |
| `include/noise/failure_injector.hpp` | Scheduled failure events |
| `src/noise/failure_injector.cpp` | Implementation |
| `include/server/api.hpp` | Crow server route definitions |
| `src/server/api.cpp` | Route implementations |
| `cli/server_main.cpp` | Server entry point |
| `include/chaos/chaos_runner.hpp` | Batch chaos runner |
| `src/chaos/chaos_runner.cpp` | Implementation |
| `cli/chaos_main.cpp` | Chaos CLI entry point |
| `tests/test_session_recorder.cpp` | Recorder tests |
| `tests/test_replay_loader.cpp` | Replay tests |
| `tests/test_failure_injector.cpp` | Failure injector tests |
| `tests/test_chaos_runner.cpp` | Chaos runner tests |

### TickState Structure
```cpp
struct TickState {
    int tick;
    RobotState ground_truth;
    double observed_readings[4];     // what sensors reported
    std::vector<std::string> active_failures;

    // Three MCL snapshots
    struct MCLSnapshot {
        std::vector<Particle> particles;
        Estimate estimate;
    };
    MCLSnapshot post_predict, post_update, post_resample;

    std::vector<float> per_particle_loss;
    double mcl_error;
    double odom_error;
    double n_eff;
    int valid_sensor_count;
};
```

### Server Routes
| Method | Path | Description |
|--------|------|-------------|
| POST | `/api/session/start` | Create session |
| POST | `/api/session/:id/tick` | Advance one tick |
| GET | `/api/session/:id/state` | Full history |
| GET | `/api/session/:id/config` | Session config |
| GET | `/api/replays` | List replays |
| GET | `/api/replays/:file/meta` | Session metadata |
| GET | `/api/replays/:file/ticks?from=N&to=M` | Paginated ticks |
| GET | `/api/health` | Health check |

### Chaos Runner
- Generates randomized paths (random walk, wall-hugger, spiral, obstacle slalom, stop-and-go)
- Per run: random noise profile variations, 0–5 scheduled failures
- Convergence: variance < 25 in² for 5 consecutive ticks
- Metric: post-convergence RMS error. Fallback at tick 100.
- Output: `chaos_report.json` + top 20 worst sessions
- Deterministic: run N → seed = base_seed + N

### Makefile Changes
- Add `server` target (links Crow + ASIO)
- Add `chaos` target (links chaos runner)
- Keep `test` target for unit tests

---

## Phase 8: Frontend

### Goal
Next.js app that drives the robot, visualizes particles, replays sessions.

### Stack
Next.js 14+, TypeScript, HTML5 Canvas, Tailwind CSS.

### Structure
```
frontend/
├── package.json
├── tsconfig.json
├── tailwind.config.ts
├── next.config.js
├── src/
│   ├── app/
│   │   ├── layout.tsx
│   │   ├── page.tsx          # Main sim page
│   │   └── replay/
│   │       └── page.tsx      # Replay page
│   ├── components/
│   │   ├── FieldCanvas.tsx   # Canvas renderer
│   │   ├── StageStepper.tsx  # Post-predict/update/resample toggle
│   │   ├── MetricsPanel.tsx  # MCL error, odom error, N_eff, etc.
│   │   └── ObstacleEditor.tsx
│   ├── hooks/
│   │   ├── useKeyboard.ts    # Keyboard → action mapping
│   │   ├── useSimSession.ts  # Session lifecycle + tick
│   │   └── useReplay.ts      # Replay loading + playback
│   └── lib/
│       ├── api.ts            # HTTP client for C++ server
│       └── types.ts          # TypeScript types matching TickState
```

### Build Order
1. Canvas rendering with hardcoded test data
2. Keyboard → tick loop (useKeyboard + useSimSession)
3. Stage stepper switching
4. Metrics panel
5. Obstacle editor
6. Replay page with scrubber + pagination

---

## Execution Order & Verification

### Step 1: Fix Phase 2 (minimal change)
- Edit `src/mcl/mcl_engine.cpp`: neutral weight → epsilon weight
- Run `make test` → expect 33/33 pass
- Commit

### Step 2: Phase 3
- Create Field, Physics, wire predict
- Run `make test` → all Phase 1-3 tests pass
- Commit

### Step 3: Phase 4
- Create NoiseGenerator, SensorModel (odom part), MCL predict noise
- Run `make test` → all Phase 1-4 tests pass
- Commit

### Step 4: Phase 5
- Complete SensorModel (sensor pipeline), IMU noise, collision stall
- Run `make test` → all Phase 1-5 tests pass
- Commit

### Step 5: Phase 6
- Add obstacles to Field, ray_cast_obstacles, update sensor model + physics
- Run `make test` → all Phase 1-6 tests pass
- Commit

### Step 6: Phase 7
- Add dependencies (nlohmann/json, Crow, ASIO)
- Build state recording, server, failure injector, chaos runner
- Run `make test` → unit tests pass
- Run `make server` → verify `curl /api/health`
- Run `make chaos` → verify output
- Commit

### Step 7: Phase 8
- `npx create-next-app frontend`
- Build canvas, hooks, components
- Test with running server
- Commit
