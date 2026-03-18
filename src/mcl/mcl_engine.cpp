#include "mcl/mcl_engine.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

namespace mcl {

MCLEngine::MCLEngine(const MCLConfig& config)
    : config_(config) {}

void MCLEngine::initialize_uniform(uint64_t seed) {
    rng_.seed(static_cast<std::mt19937::result_type>(seed));
    particles_.resize(config_.num_particles);

    std::uniform_real_distribution<float> dist_x(
        static_cast<float>(-config_.field_half),
        static_cast<float>(config_.field_half));
    std::uniform_real_distribution<float> dist_y(
        static_cast<float>(-config_.field_half),
        static_cast<float>(config_.field_half));

    const float w = 1.0f / static_cast<float>(config_.num_particles);
    for (auto& p : particles_) {
        p.x = dist_x(rng_);
        p.y = dist_y(rng_);
        p.weight = w;
    }
}

void MCLEngine::predict(double delta_forward, double delta_rotation, double heading_deg) {
    predict(delta_forward, delta_rotation, heading_deg, 0.0);
}

void MCLEngine::predict(double delta_forward,
                        double /*delta_rotation*/,
                        double heading_deg,
                        double delta_lateral) {
    if (std::fabs(delta_forward) < 1e-9 && std::fabs(delta_lateral) < 1e-9) return;

    // Noise scale follows traveled forward distance.
    const double travel = std::fabs(delta_forward);
    const double sigma_fwd = config_.predict_noise_fwd * travel;
    const double sigma_lat = config_.predict_noise_lat * travel;

    const double heading_rad = distance_loc::deg2rad(heading_deg);
    const double s = std::sin(heading_rad);
    const double c = std::cos(heading_rad);

    std::normal_distribution<double> fwd_noise(0.0, sigma_fwd);
    std::normal_distribution<double> lat_noise(0.0, sigma_lat);

    for (auto& p : particles_) {
        const double local_fwd = delta_forward + (sigma_fwd > 0.0 ? fwd_noise(rng_) : 0.0);
        const double local_lat = delta_lateral + (sigma_lat > 0.0 ? lat_noise(rng_) : 0.0);

        // Heading convention:
        // 0 deg = +Y, CW positive.
        // Forward vector  = (sin(h),  cos(h))
        // Rightward vector = (cos(h), -sin(h))
        const double dx = local_fwd * s + local_lat * c;
        const double dy = local_fwd * c - local_lat * s;
        p.x += static_cast<float>(dx);
        p.y += static_cast<float>(dy);
    }
}

void MCLEngine::update(const double readings[4], double heading_deg) {
    const int N = static_cast<int>(particles_.size());
    if (N == 0) return;

    const double sigma_sq_2 = 2.0 * config_.sigma_sensor * config_.sigma_sensor;
    const double max_err = config_.max_sensor_error;

    // Check which sensors are valid
    bool valid[4];
    for (int i = 0; i < 4; i++) {
        valid[i] = (readings[i] >= 0.0);
    }

    // If no sensors valid at all, set uniform weights
    bool any_valid = valid[0] || valid[1] || valid[2] || valid[3];
    if (!any_valid) {
        float w = 1.0f / static_cast<float>(N);
        for (auto& p : particles_) {
            p.weight = w;
        }
        return;
    }

    double weight_sum = 0.0;

    for (auto& p : particles_) {
        distance_loc::Vec2 pos = { static_cast<double>(p.x), static_cast<double>(p.y) };

        // Ray-cast each valid sensor and compute errors
        double errors[4] = { 0.0, 0.0, 0.0, 0.0 };
        bool below_cap[4] = { false, false, false, false };

        for (int i = 0; i < 4; i++) {
            if (!valid[i]) continue;

            double predicted = distance_loc::ray_distance_with_offset(
                pos, heading_deg,
                config_.sensors[i].offset,
                config_.sensors[i].angle_deg);

            // If ray returns infinity (particle outside field), treat as huge error
            if (!std::isfinite(predicted)) {
                errors[i] = max_err + 1.0;
                below_cap[i] = false;
                continue;
            }

            errors[i] = std::fabs(predicted - readings[i]);

            // Hard cap: drop sensors with error > max_sensor_error
            below_cap[i] = (errors[i] <= max_err);
        }

        // Score every valid sensor individually.
        // Inlier (below cap): contributes error to tight Gaussian.
        // Outlier (above cap): multiplies weight by a small penalty.
        double inlier_error_sq_sum = 0.0;
        int inlier_count = 0;
        int outlier_count = 0;

        for (int i = 0; i < 4; i++) {
            if (!valid[i]) continue;
            if (below_cap[i]) {
                inlier_error_sq_sum += errors[i] * errors[i];
                inlier_count++;
            } else {
                outlier_count++;
            }
        }

        // Compute weight: tight Gaussian on mean error, penalty for outliers
        constexpr double kOutlierPenalty = 0.01;
        double log_w = 0.0;
        if (inlier_count > 0) {
            double mean_err_sq = inlier_error_sq_sum / inlier_count;
            log_w = -mean_err_sq / sigma_sq_2;
        }
        log_w += outlier_count * std::log(kOutlierPenalty);
        p.weight = static_cast<float>(std::exp(log_w));

        weight_sum += p.weight;
    }

    // Normalize weights — fallback to uniform if all weights underflowed
    if (weight_sum > 1e-30) {
        float inv_sum = static_cast<float>(1.0 / weight_sum);
        for (auto& p : particles_) {
            p.weight *= inv_sum;
        }
    } else {
        // All particles scored near-zero (e.g. all readings corrupted).
        // No information available — set uniform weights.
        float w = 1.0f / static_cast<float>(N);
        for (auto& p : particles_) {
            p.weight = w;
        }
    }
}

void MCLEngine::resample() {
    const int N = static_cast<int>(particles_.size());
    if (N == 0) return;

    double neff = n_eff();
    if (neff / N >= config_.resample_threshold) return;

    // Low-variance resampling
    std::vector<Particle> new_particles;
    int num_resampled = N - static_cast<int>(config_.random_injection * N);
    if (num_resampled < 1) num_resampled = 1;
    int num_random = N - num_resampled;

    new_particles.reserve(N);

    // Build cumulative weight array
    std::vector<double> cumulative(N);
    cumulative[0] = particles_[0].weight;
    for (int i = 1; i < N; i++) {
        cumulative[i] = cumulative[i - 1] + particles_[i].weight;
    }

    // Low-variance resampling
    std::uniform_real_distribution<double> start_dist(0.0, 1.0 / num_resampled);
    double r = start_dist(rng_);
    int idx = 0;
    for (int i = 0; i < num_resampled; i++) {
        double target = r + static_cast<double>(i) / num_resampled;
        while (idx < N - 1 && cumulative[idx] < target) {
            idx++;
        }
        new_particles.push_back(particles_[idx]);
    }

    // Inject random particles
    std::uniform_real_distribution<float> dist_x(
        static_cast<float>(-config_.field_half),
        static_cast<float>(config_.field_half));
    std::uniform_real_distribution<float> dist_y(
        static_cast<float>(-config_.field_half),
        static_cast<float>(config_.field_half));

    for (int i = 0; i < num_random; i++) {
        Particle p;
        p.x = dist_x(rng_);
        p.y = dist_y(rng_);
        p.weight = 0.0f; // will be reset below
        new_particles.push_back(p);
    }

    // Roughening: add small gaussian jitter to prevent particle depletion.
    // Without this, cloned particles all sit at the same position, get
    // identical weights on the next update, and the cloud never refines.
    if (config_.roughening_sigma > 0.0) {
        std::normal_distribution<float> jitter(0.0f,
            static_cast<float>(config_.roughening_sigma));
        const float fh = static_cast<float>(config_.field_half);
        for (auto& p : new_particles) {
            p.x += jitter(rng_);
            p.y += jitter(rng_);
            // Clamp to field bounds
            p.x = std::max(-fh, std::min(fh, p.x));
            p.y = std::max(-fh, std::min(fh, p.y));
        }
    }

    // Reset weights to uniform
    float w = 1.0f / static_cast<float>(N);
    for (auto& p : new_particles) {
        p.weight = w;
    }

    particles_ = std::move(new_particles);
}

Estimate MCLEngine::estimate() const {
    double sum_x = 0.0, sum_y = 0.0;
    for (const auto& p : particles_) {
        sum_x += p.weight * p.x;
        sum_y += p.weight * p.y;
    }
    return { static_cast<float>(sum_x), static_cast<float>(sum_y) };
}

double MCLEngine::n_eff() const {
    double sum_sq = 0.0;
    for (const auto& p : particles_) {
        sum_sq += static_cast<double>(p.weight) * p.weight;
    }
    if (sum_sq < 1e-15) return 0.0;
    return 1.0 / sum_sq;
}

const std::vector<Particle>& MCLEngine::particles() const {
    return particles_;
}

} // namespace mcl
