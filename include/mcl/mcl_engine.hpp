#pragma once

#include "mcl/particle.hpp"
#include "distance_localization.hpp"
#include <cstdint>
#include <random>
#include <vector>

namespace mcl {

struct SensorMount {
    distance_loc::Vec2 offset;
    double angle_deg;
};

struct MCLConfig {
    int num_particles = 300;
    double sigma_sensor = 1.5;
    double max_sensor_error = 6.0;
    double random_injection = 0.05;
    double resample_threshold = 0.5;
    double roughening_sigma = 1.5;  // post-resample jitter for convergence and particle diversity
    double field_half = 72.0;

    // Sensor mounts: left, right, front, back
    SensorMount sensors[4] = {
        { { -6.043, -1.811 }, -90.0 },  // left
        { {  6.004, -2.382 },  90.0 },  // right
        { {  3.268,  5.512 },   0.0 },  // front
        { { -3.878, -4.055 }, 180.0 }   // back
    };
};

// Sensor indices
enum SensorIdx { LEFT = 0, RIGHT = 1, FRONT = 2, BACK = 3 };

struct Estimate {
    float x;
    float y;
};

class MCLEngine {
public:
    explicit MCLEngine(const MCLConfig& config = {});

    void initialize_uniform(uint64_t seed);

    // Predict: move particles by odom delta (no-op when delta is zero)
    void predict(double delta_forward, double delta_rotation, double heading_deg);

    // Update: score particles against sensor readings
    // readings[4]: left, right, front, back. -1 means invalid.
    void update(const double readings[4], double heading_deg);

    // Resample if N_eff is too low
    void resample();

    // Weighted mean position estimate
    Estimate estimate() const;

    // Effective sample size
    double n_eff() const;

    // Access particles (for tests)
    const std::vector<Particle>& particles() const;

private:
    MCLConfig config_;
    std::vector<Particle> particles_;
    std::mt19937 rng_;
};

} // namespace mcl
