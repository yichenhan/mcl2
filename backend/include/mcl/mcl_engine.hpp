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
    // Two-regime sensor sigma (inches): close vs far, then calibration margin.
    double sigma_close_range_in = 7.874;   // 200mm boundary
    double sigma_close_in = 0.30;          // ~15mm/2 for <200mm
    double sigma_far_fraction = 0.025;     // ~5%/2 for >=200mm
    double sigma_calibration_margin = 1.5; // safety factor
    double heading_uncertainty_deg = 1.5;  // IMU heading uncertainty for sigma inflation
    // Per-sensor outlier cap (inches): tight near wall, proportional when far.
    double outlier_close_range_in = 7.874;
    double outlier_close_cap_in = 1.8;
    double outlier_far_cap_fraction = 0.15;
    double random_injection = 0.05;
    double resample_threshold = 0.5;
    double roughening_sigma = 0.3;  // post-resample jitter for convergence and particle diversity
    // Force resampling for a short window after initialize_uniform() to
    // improve global relocalization and re-init convergence.
    int bootstrap_recovery_ticks = 300;
    // When the average raw weight per particle drops below this threshold,
    // all particles are lost.  Force resampling with higher injection.
    double lost_weight_threshold = 1e-3;
    double lost_random_injection = 0.15;
    double field_half = 72.0;
    double predict_noise_fwd = 0.03;   // per-particle forward noise fraction
    double predict_noise_lat = 0.015;  // per-particle lateral noise fraction
    double predict_noise_heading_deg = 1.5;

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

struct ClusterStats {
    double spread;          // weighted RMS distance from centroid (inches)
    double radius_90;       // radius containing 90% of weighted mass (inches)
    Estimate centroid;      // weighted mean position
};

class MCLEngine {
public:
    explicit MCLEngine(const MCLConfig& config = {});

    void initialize_uniform(uint64_t seed);

    // Predict: move particles by odom delta (with configured motion / heading noise)
    void predict(double delta_forward, double delta_rotation, double heading_deg);
    void predict(double delta_forward, double delta_rotation, double heading_deg, double delta_lateral);

    // Update: score particles against sensor readings
    // readings[4]: left, right, front, back. -1 means invalid.
    void update(const double readings[4], double heading_deg);

    // Resample if N_eff is too low
    void resample();

    // Weighted mean position estimate
    Estimate estimate() const;

    // Particle cloud clustering statistics.
    // spread = weighted RMS distance from centroid.
    // radius_90 = smallest radius around the centroid enclosing 90% of weighted mass.
    ClusterStats cluster_stats() const;

    // Effective sample size
    double n_eff() const;

    // Access particles (for tests)
    const std::vector<Particle>& particles() const;
    const MCLConfig& config() const;

private:
    MCLConfig config_;
    std::vector<Particle> particles_;
    std::mt19937 rng_;
    int bootstrap_ticks_left_ = 0;
    double last_raw_weight_sum_ = 0.0;
};

} // namespace mcl
