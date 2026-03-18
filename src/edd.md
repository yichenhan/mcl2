# MCL Simulator — Design & Build Guide

---

## 1. Design Summary

### What we're building

A standalone C++ simulator that tests Monte Carlo Localization against a realistic VEX robot sensor model. The robot has four distance sensors that ray-cast to field walls. MCL uses a cloud of particles to estimate (x, y) position. A Next.js frontend visualizes everything. A chaos CLI stress-tests across thousands of paths.

### Key decisions

- **Particles are (x, y) only.** Theta always comes from the IMU — MCL doesn't estimate heading.
- **MCL is obstacle-blind.** Ground-truth sensors see obstacles; MCL only knows about walls. Robust scoring handles the disagreement.
- **No PROS/VEX dependencies.** Ray-cast math ported verbatim from `ray_wall.cpp` but with globals replaced by parameters.
- **Everything is deterministic.** Same seed → same results. Every session and chaos run is reproducible.

### Coordinate system

Origin at field center. X = right, Y = forward. Heading in degrees, clockwise from +Y (VEX convention). Field: 144″ × 144″, walls at ±72″. Tick = 50 ms.

### Architecture

```
Next.js Frontend (renders JSON only)
    ↕  HTTP / JSON
Crow C++ Server
    ├── Sim Engine (physics, sensors, noise)
    ├── MCL Engine (predict, update, resample)
    ├── Ray-Cast (ported from ray_wall.cpp)
    └── State Recorder (tick → JSON)

Chaos CLI (separate binary, batch offline)
```

---

## 2. How MCL Works

### The idea

Scatter many "particles" across the field. Each particle is a guess for where the robot might be. Every tick:

1. **Predict:** Move every particle by the same amount the odometry says the robot moved (plus some noise, because odom isn't perfect).
2. **Update:** Ask each particle "if the robot were here, what would the sensors read?" Compare to actual readings. Particles whose predictions match reality get high weights; bad guesses get low weights.
3. **Resample:** Clone the high-weight particles, kill the low-weight ones. Inject a few random particles for recovery.

After a few ticks, the surviving particles cluster around the true position.

### The math

**Predict:** For each particle, add the observed odom delta plus gaussian noise proportional to distance moved. Noise is applied in local frame (forward + lateral), then rotated to field frame via IMU heading. This produces realistic heading-correlated error.

**Update (outlier-robust with hard cap):** For each particle, ray-cast each valid sensor from that particle's position using the IMU heading. Then apply a two-stage filter:

1. **Hard error cap:** Any sensor where `|predicted - measured| > max_sensor_error` (default 6″) is silently dropped — it contributes nothing (no penalty, just absent). This handles obstacle-blocked sensors, which can have 30–40″ errors that would otherwise destroy every particle's weight.
2. **Per-axis best-of:** Group the surviving (below-cap) sensors by axis: X-axis = {left, right}, Y-axis = {front, back}. If both sensors on an axis survived, pick the one with lower error as the inlier; exclude the other. If only one survived, it's the inlier. If neither survived, that axis contributes nothing.
3. **Weight:** `exp( -Σ(inlier_error²) / (2σ²) )` over the selected inliers (at most 2 — one per axis). If no sensors survive the cap on either axis, the particle gets a neutral weight (1/N) — no information available, don't penalize or reward.

**Resample:** Compute effective sample size N_eff = 1/Σ(w²). If too low, run low-variance resampling (clone winners, kill losers) and inject a small fraction of uniform random particles for kidnap recovery.

**Estimate:** Weighted mean of all particle (x, y) positions.

---

## 3. Build Phases

Each phase is self-contained and testable. Don't start a phase until the previous one passes all tests. Phases 1–6 are backend. Phases 7–8 are frontend.

---

### Phase 1: Port the ray-cast math

**Goal:** Get `ray_distance_with_offset` compiling and tested outside of PROS.

**What to do:**

- Copy `rotateOffsetByHeading`, `rayDistanceToSquareWalls`, `rayWallHit`, `ray_distance_with_offset` from `ray_wall.cpp` into `ray/ray_cast.hpp` and `ray/ray_cast.cpp`.
- Define a local `Vec2 { double x, y }` struct.
- Replace every `constants::kRayWall.field_half_in` reference with a `field_half_in` parameter.
- Remove all `#include` references to PROS, LemLib, `constants.hpp`, `distance_localization.hpp`.
- **Do not** rename functions or change any math.

**Files:** `include/ray/ray_cast.hpp`, `src/ray/ray_cast.cpp`, `tests/test_ray_cast.cpp`

**Tests — all at field_half = 72.0:**

- Robot at (0, 0) heading 0° (facing +Y): front sensor predicts ~72 − offset_y inches
- Robot at (0, 0) heading 90° (facing +X): right sensor predicts ~72 − offset_x inches
- All four cardinal headings at center: verify all four sensors give sensible distances
- Full 360° heading sweep at 10° increments at center
- Robot at (-71, 0): left sensor should read ~1″ to left wall
- Sensor ray exactly parallel to a wall: returns infinity
- Sensor origin on wall boundary

**Sensor mounts to use for testing** (hardcoded from VEX robot):

- Left: offset (-6.043, -1.811), angle -90°
- Right: offset (6.004, -2.382), angle 90°
- Front: offset (3.268, 5.512), angle 0°
- Back: offset (-3.878, -4.055), angle 180°

---

### Phase 2: Stationary MCL with perfect sensors

**Goal:** MCL converges on a stationary robot with zero noise. This is the simplest possible case — no movement, no noise, no obstacles.

**What to do:**

- Build the `Particle { float x, y, weight }` struct.
- Build `MCLEngine` with `initialize_uniform` (scatter particles across field), `predict` (no-op when delta is zero), `update` (ray-cast per particle, compute weights), `resample` (low-variance + random injection), and `weighted_mean`.
- The update step uses `ray_distance_with_offset` (walls only) from Phase 1.
- Implement the full outlier-robust scoring from the start: hard error cap (default 6″) → per-axis best-of → weight from inliers only. This is the core scoring logic and must be tested early.

**Files:** `include/mcl/particle.hpp`, `include/mcl/mcl_engine.hpp`, `src/mcl/mcl_engine.cpp`, `tests/test_mcl_stationary.cpp`

**MCL config for testing:** 150 particles, σ_sensor = 1.5″, max_sensor_error = 6.0″, random_injection = 0.05, resample_threshold = 0.5.

**Tests — robot stationary at (20, 30), heading 45°, perfect sensor readings (no noise):**

- After 1 tick: particles near (20, 30) have highest weights
- After 5 ticks: estimate within 5″ of truth
- After 15 ticks: estimate within 2″ of truth
- N_eff decreases as particles converge (weights become unequal)
- After convergence, most particles cluster within 3″ of truth

**Tests — update scoring logic:**

- All sensors valid, particle at true position: weight is high (near max)
- All sensors valid, particle at wrong position (50, 50): weight is near zero
- All sensors invalid (-1): all particles get equal weight (no update)
- Only left sensor valid: weights differentiate along X-axis but not Y-axis
- One sensor on X-axis is an outlier (error > 10″), other is accurate: outlier exceeds 6″ cap, gets dropped. Weight based on the accurate sensor only — still high.
- Both sensors on one axis exceed the 6″ cap: that axis contributes nothing. Weight based on the other axis only.
- All sensors exceed the 6″ cap: particle gets neutral weight (1/N), not penalized.

---

### Phase 3: MCL with movement (still no noise)

**Goal:** MCL tracks a moving robot using perfect odometry and perfect sensors.

**What to do:**

- Build `Field` class (field boundary, `is_inside` check — no obstacles yet).
- Build `Physics` module: takes an action (forward/backward/rotate_cw/rotate_ccw/none), updates ground-truth robot state, returns `MotionDelta { forward_in, rotation_deg }`. Fixed max velocity (36 in/s), max angular velocity (360°/s), dt = 0.05s. Collision with walls: snap to last valid position, set `colliding` flag. Robot bounding radius ~8″.
- Wire the predict step: apply odom delta to each particle (zero noise for now).
- Build a simple test harness that runs a sequence of actions and checks MCL tracking.

**Files:** `include/sim/field.hpp`, `src/sim/field.cpp`, `include/sim/physics.hpp`, `src/sim/physics.cpp`, `tests/test_physics.cpp`, `tests/test_mcl_moving.cpp`

**Tests — physics:**

- Forward at center heading 0°: y increases by velocity × dt
- Forward into wall: snaps back, collision flag true, `forward_in` reflects actual distance
- Rotation at 0°/360° boundary: wraps correctly
- No action: state unchanged, delta is (0, 0)

**Tests — MCL tracking (perfect odom + perfect sensors):**

- Robot moves forward 10 ticks: MCL estimate tracks within 2″ the whole time
- Robot rotates 90° then moves forward: MCL tracks through the turn
- Robot moves to wall and collides: MCL handles the stopped motion
- Robot moves in a square (forward, rotate, forward, rotate...): MCL tracks the full path

---

### Phase 4: Add odometry noise

**Goal:** MCL still converges when odometry is noisy. This is the first "realistic" scenario.

**What to do:**

- Build `NoiseGenerator` (deterministic wrapper around `<random>`: gaussian, uniform, bernoulli, seeded).
- Add odom noise to the sensor model. The noise pipeline: receive `MotionDelta { forward_in, rotation_deg }` → add proportional noise in local frame (`noisy_forward = forward_in + N(0, (α × |forward_in|)²)`, `lateral = N(0, (α_drift × |forward_in|)²)`) → add constant drift bias → rotate to field frame using IMU heading → accumulate into odom pose.
- Add noise to the MCL predict step: when applying the odom delta to particles, add gaussian noise proportional to distance moved.
- The sensor model now produces noisy odom deltas; MCL receives these.

**Files:** `include/noise/noise_generator.hpp`, `src/noise/noise_generator.cpp`, `tests/test_noise_generator.cpp`, `include/sim/sensor_model.hpp`, `src/sim/sensor_model.cpp` (odom part only), `tests/test_mcl_odom_noise.cpp`

**Tests — noise generator:**

- Same seed → identical sequence
- Gaussian: 10K samples, mean within 0.1σ of target, stddev within 5%
- Bernoulli: 10K trials within 2% of target probability

**Tests — odom noise:**

- Zero noise config: odom deltas match ground truth exactly
- With noise: over many ticks, odom drifts away from ground truth (accumulated error grows)
- Odom noise is heading-correlated: forward error > lateral error (verify by running many samples)

**Tests — MCL with noisy odom, perfect sensors:**

- Robot moves forward 20 ticks with noisy odom: odom drifts but MCL estimate stays within 3″ of truth
- Robot moves in a square with noisy odom: MCL tracks within 5″ throughout
- Compare MCL error vs odom error: MCL should be consistently better

---

### Phase 5: Add sensor noise and dropouts

**Goal:** MCL works with realistic sensor noise, random dropouts, range limits, and spurious reflections.

**What to do:**

- Complete the sensor model's distance sensor pipeline. Pipeline order (dropout checks first to avoid wasted ray-casts):
  1. **Long dropout check:** if in multi-tick dropout, return -1, decrement counter, done.
  2. **Random dropout:** bernoulli check → -1, optionally start long dropout, done.
  3. **Ray-cast** from ground-truth pose using `ray_distance_with_offset` (walls only — no obstacles yet). Convert to mm.
  4. **Range gate:** > 2000 mm → -1.
  5. **Range noise ramp:** > 1700 mm → add extra gaussian scaled by `(dist - 1700) × slope`.
  6. **Base gaussian noise:** N(0, σ²).
  7. **Spurious reflection:** with probability p, replace reading with `true_distance_mm × uniform(0.3, 0.9)`.
  8. **Clamp** to [0, 9999] or -1.
- Add IMU noise: ground-truth theta + per-tick gaussian (no cumulative drift).
- Add collision stall: after physics collision, odom delta → 0 for N ticks.

**Files:** Complete `src/sim/sensor_model.cpp`, `tests/test_sensor_model.cpp`, `tests/test_mcl_noisy_sensors.cpp`

**Noise profile defaults:**

| Parameter                              | Default |
| -------------------------------------- | ------- |
| sensor_gaussian_stddev_mm              | 15.0    |
| sensor_dropout_probability             | 0.02    |
| sensor_long_dropout_probability        | 0.005   |
| sensor_long_dropout_min_ticks          | 10      |
| sensor_long_dropout_max_ticks          | 60      |
| sensor_range_noise_start_mm            | 1700    |
| sensor_range_max_mm                    | 2000    |
| sensor_range_noise_slope               | 0.05    |
| sensor_spurious_reflection_probability | 0.01    |
| odom_trans_noise_frac                  | 0.03    |
| odom_drift_noise_frac                  | 0.01    |
| odom_drift_per_tick_in                 | 0.01    |
| imu_noise_stddev_deg                   | 0.1     |
| collision_encoder_spike_rpm            | 0       |
| collision_stall_ticks                  | 5       |

**Tests — sensor model:**

- Zero noise profile: readings match ground truth exactly
- Gaussian noise stats over 10K samples (mean, stddev)
- Dropout probability over 10K ticks matches configured rate (±5%)
- Long dropout persists for configured duration
- Long dropout prevents ray-cast from running (no wasted computation)
- Range gate at 2100 mm returns -1
- Readings at 1800 mm have higher stddev than at 1000 mm
- Collision stall: odom delta = 0 for configured ticks
- Spurious reflection: readings are 30%–90% of true distance when triggered
- Spurious reflection rate over 10K ticks matches configured probability

**Tests — MCL with full noise:**

- Moving robot, moderate noise: MCL tracks within 5″
- Single sensor dropout for 10 ticks: MCL degrades slightly, recovers when sensor returns
- Both sensors on one axis drop out: MCL still tracks via the other axis (less accurate)
- All sensors drop out for 10 ticks: MCL drifts with odom, recovers when sensors return
- Spurious reflection on one sensor: outlier-robust scoring excludes it, MCL unaffected
- Kidnapped robot (teleport ground truth): MCL recovers within 20 ticks via random injection

---

### Phase 6: Add obstacles

**Goal:** Obstacles affect physics (collision) and ground-truth sensors (shorter readings) but MCL remains obstacle-blind.

**What to do:**

- Extend `Field` to hold an immutable list of axis-aligned rectangle obstacles. Update `is_passable` to check obstacles (with bounding radius).
- Build `ray_cast_obstacles`: extend ray-cast to intersect AABB obstacle faces. Returns min distance across walls + obstacle faces.
- Update the sensor model to use `ray_distance_with_obstacles` for ground-truth readings (step 3 in the pipeline).
- MCL stays on `ray_distance_with_offset` (walls only). This is the critical asymmetry: sensors see obstacles, MCL doesn't.
- Update physics to collide with obstacles.

**Files:** `include/ray/ray_cast_obstacles.hpp`, `src/ray/ray_cast_obstacles.cpp`, `tests/test_ray_cast_obstacles.cpp`, update `Field`, update `sensor_model`, `tests/test_mcl_obstacles.cpp`

**Tests — ray-cast with obstacles:**

- No obstacles: results match base ray_cast exactly
- Obstacle in sensor path: distance shortens to obstacle face
- Obstacle behind sensor: no effect
- Obstacle flush against wall: no double-counting
- Ray grazes obstacle corner: numerically stable

**Tests — MCL with obstacles (the hard case):**

- Robot near obstacle, one sensor reads short (e.g., 20″ instead of 60″): error exceeds 6″ cap, sensor is silently dropped. MCL uses the other sensor on that axis. Verify MCL estimate stays accurate.
- Robot near obstacle, only one sensor on that axis is valid (the other is in dropout): the remaining sensor reads short due to obstacle, exceeds cap, gets dropped. That axis has zero information. MCL scores on the other axis only. Verify MCL accuracy degrades on the obstructed axis but remains good on the clear axis.
- Robot surrounded by obstacles on one axis: both sensors exceed cap, both dropped. MCL relies entirely on the other axis. Verify MCL tracks on the clear axis, drifts on the blocked axis.
- Robot in open area with distant obstacles: all sensors within cap, no effect on MCL accuracy.
- Robot collides with obstacle: physics clamps, odom stalls, MCL handles it.

---

### Phase 7: State recording, server, and chaos suite

**Goal:** Wire everything into a Crow HTTP server, record sessions to JSON, build the chaos batch runner.

**What to do:**

**State recording:**

- Build `TickState` struct: tick number, ground truth, observed readings, active failures, three MCL snapshots (post-predict, post-update, post-resample — each with particles + estimate), per-particle loss array, MCL error, odom error, N_eff, valid sensor count.
- Build `SessionRecorder`: accumulates ticks, serializes to JSON (nlohmann/json). Auto-saves every 20 ticks + on destruction via atomic write (temp file → rename).
- Build `ReplayLoader`: reads session JSON back into structs. Paginated access by tick range.

**Failure injector:**

- Scheduled failure events: mode + start tick + duration + description.
- Modes: individual sensor dropout (×4), pairwise (left+right, front+back), all sensors, odom drift spike, odom stall, IMU jump, high sensor noise (3×), spurious reflection burst.

**Crow server routes:**

| Method | Path                                   | Description                                                     |
| ------ | -------------------------------------- | --------------------------------------------------------------- |
| POST   | `/api/session/start`                   | Create session (particles, noise config, obstacles, start pose) |
| POST   | `/api/session/:id/tick`                | Advance one tick, return TickState JSON                         |
| GET    | `/api/session/:id/state`               | Full session history                                            |
| GET    | `/api/session/:id/config`              | Session config                                                  |
| GET    | `/api/replays`                         | List saved replay files                                         |
| GET    | `/api/replays/:file/meta`              | Session metadata without ticks (< 1KB)                          |
| GET    | `/api/replays/:file/ticks?from=N&to=M` | Paginated tick fetch (max 50 per request)                       |
| GET    | `/api/health`                          | Health check                                                    |

**Chaos runner:**

- Batch-generates thousands of randomized paths (random walk, wall-hugger, spiral, obstacle slalom, stop-and-go).
- Per run: random noise profile variations, 0–5 scheduled failure events.
- Convergence detection: particle variance (var_x + var_y) < 25 in² for 5 consecutive ticks.
- Primary error metric: post-convergence RMS position error. If no convergence by tick 100, evaluate anyway and flag `"never_converged"`.
- Output: `chaos_report.json` (sorted by rms_error, with p50/p95/p99) + top 20 worst session JSONs for replay.
- Deterministic: run N gets seed = base_seed + N.

**Files:** `state/`, `noise/failure_injector.*`, `server/api.*`, `chaos/`, `cli/chaos_main.cpp`

**Tests:**

- Session recorder: 0 ticks → empty array. 1 tick round-trip. Save + load fidelity. Atomic write on crash.
- Replay loader: round-trip match. Invalid JSON throws descriptive error.
- Failure injector: event active at correct ticks. Overlapping events both reported. Permanent events persist.
- Chaos runner: 1 run / 10 ticks / zero noise → tiny error. 5 runs → sorted results. Same seed → identical output.
- Server: curl `/health` → ok. Start session → tick → verify JSON structure.

---

### Phase 8: Frontend

**Goal:** Next.js app that drives the robot, visualizes particles, and replays sessions.

**Stack:** Next.js 14+, TypeScript, HTML5 Canvas, Tailwind CSS.

**Main page (`/`):**

- **Field canvas:** Field boundary, obstacles (gray), ground truth robot (green arrow), MCL estimate (blue circle + heading), odom position (orange dashed), particles (dots colored by loss: blue=low → red=high).
- **Keyboard controls:** ↑ forward, ↓ backward, ← rotate_ccw, → rotate_cw, Space = no movement. Each keypress = one `POST /tick`.
- **Stage stepper:** Three radio buttons (Post-Predict, Post-Update, Post-Resample). Swaps which particle snapshot renders on canvas.
- **Metrics panel:** MCL error, odom error, N_eff/N, valid sensors, active failure tags.
- **Obstacle editor (pre-session only):** Click-drag to place rectangles. Once session starts, obstacles lock.

**Replay page (`/replay`):**

- Load replay via `/meta` endpoint (instant). Fetch ticks on demand via `/ticks?from=N&to=M` in windows of ~20.
- Timeline scrubber, step-back/forward, play/pause with speed control (0.5×–10×).
- Same canvas + stage stepper, read-only.

**Build order within this phase:**

1. Canvas rendering with hardcoded test data
2. Keyboard → tick loop (`useKeyboard` + `useSimSession` hooks)
3. Stage stepper switching
4. Metrics panel
5. Obstacle editor
6. Replay page with scrubber + pagination

---

## 4. Implementation Rules

1. **Do not modify ray-cast math.** Port verbatim. Only remove `constants.hpp` globals.
2. **Particles are (x, y) only.** Theta from IMU. No theta in particle struct.
3. **MCL is obstacle-blind.** Uses `ray_distance_with_offset` (walls only). Never `ray_distance_with_obstacles`.
4. **MCL update: hard cap → per-axis best-of → weight from inliers.** Any sensor with error > 6″ is silently dropped. Of survivors, best per axis is the inlier. If no sensors survive on either axis, particle gets neutral weight (1/N).
5. **Sensor pipeline: dropout checks first.** Long dropout → random dropout → ray-cast → range gate → range noise → base noise → spurious reflection → clamp.
6. **Physics outputs MotionDelta** `{ forward_in, rotation_deg }`. Sensor model receives this directly.
7. **Odom noise in local frame.** Forward + lateral noise, then rotate to field frame via IMU heading.
8. **Spurious reflections** = `true_distance × uniform(0.3, 0.9)`. Applied after base noise, before clamp.
9. **All noise in SensorModel.** MCL only sees noisy observations.
10. **All randomness seeded.** Same seed → same results.
11. **JSON is the contract.** Frontend renders, never computes.
12. **One tick = one HTTP call.**
13. **Obstacles immutable after session start.**
14. **Sensor mounts hardcoded.** Noise profile configurable.
15. **Auto-save every 20 ticks** + on close. Atomic write.
16. **Replay endpoint paginated.** Meta + tick-range, no monolithic dumps.
17. **Chaos metric: post-convergence RMS error.** Convergence = variance < 25 in² for 5 ticks. Fallback at tick 100.
18. **Tests before moving on.** Every phase's tests must pass before starting the next phase.
19. **Compile with** `g++ -std=c++17 -O2`. Header-only deps: Crow, nlohmann/json, Catch2/doctest.
