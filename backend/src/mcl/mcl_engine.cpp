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

    // Fresh global relocalization bootstrap.
    bootstrap_ticks_left_ = config_.bootstrap_recovery_ticks;
}

void MCLEngine::predict(double delta_forward, double delta_rotation, double heading_deg) {
    predict(delta_forward, delta_rotation, heading_deg, 0.0);
}

void MCLEngine::predict(double delta_forward,
                        double /*delta_rotation*/,
                        double heading_deg,
                        double delta_lateral) {
    // Noise scale follows traveled forward distance.
    const double travel = std::fabs(delta_forward);
    const double sigma_fwd = config_.predict_noise_fwd * travel;
    const double sigma_lat = config_.predict_noise_lat * travel;

    const double heading_rad = distance_loc::deg2rad(heading_deg);
    const double heading_noise_std_rad =
        distance_loc::deg2rad(config_.predict_noise_heading_deg);

    std::normal_distribution<double> fwd_noise(0.0, std::max(sigma_fwd, 1e-12));
    std::normal_distribution<double> lat_noise(0.0, std::max(sigma_lat, 1e-12));
    std::normal_distribution<double> heading_noise(0.0,
        std::max(heading_noise_std_rad, 1e-12));

    for (auto& p : particles_) {
        const double noisy_heading = heading_rad +
            (config_.predict_noise_heading_deg > 0.0 ? heading_noise(rng_) : 0.0);
        const double ns = std::sin(noisy_heading);
        const double nc = std::cos(noisy_heading);

        const double local_fwd = delta_forward + (sigma_fwd > 0.0 ? fwd_noise(rng_) : 0.0);
        const double local_lat = delta_lateral + (sigma_lat > 0.0 ? lat_noise(rng_) : 0.0);

        // Heading convention:
        // 0 deg = +Y, CW positive.
        // Forward vector  = (sin(h),  cos(h))
        // Rightward vector = (cos(h), -sin(h))
        const double dx = local_fwd * ns + local_lat * nc;
        const double dy = local_fwd * nc - local_lat * ns;
        p.x += static_cast<float>(dx);
        p.y += static_cast<float>(dy);
    }

    // Clamp particles to field bounds after movement.
    // Without this, a single odom spike can push ALL particles outside
    // the field permanently — update() gives them equal (junk) weights,
    // n_eff stays at N, and resample never triggers the clamping code.
    const float fh = static_cast<float>(config_.field_half);
    for (auto& p : particles_) {
        p.x = std::max(-fh, std::min(fh, p.x));
        p.y = std::max(-fh, std::min(fh, p.y));
    }
}

void MCLEngine::update(const double readings[4], double heading_deg) {
    const int N = static_cast<int>(particles_.size());
    if (N == 0) return;

    const double heading_unc_rad = distance_loc::deg2rad(config_.heading_uncertainty_deg);
    const double field_span_in = 2.0 * config_.field_half;
    const double outlier_huge_fallback =
        std::max(config_.outlier_close_cap_in,
                 config_.outlier_far_cap_fraction * field_span_in) + 1.0;

    // Check which sensors are valid
    bool valid[4];
    for (int i = 0; i < 4; i++) {
        valid[i] = (readings[i] >= 0.0);
    }

    // If no sensors valid at all, set uniform weights
    bool any_valid = valid[0] || valid[1] || valid[2] || valid[3];
    if (!any_valid) {
        last_raw_weight_sum_ = 1.0;
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
        double sigma_i[4] = { 0.0, 0.0, 0.0, 0.0 };
        bool below_cap[4] = { false, false, false, false };

        for (int i = 0; i < 4; i++) {
            if (!valid[i]) continue;

            double predicted = distance_loc::ray_distance_with_offset(
                pos, heading_deg,
                config_.sensors[i].offset,
                config_.sensors[i].angle_deg);

            const double max_err_i = (readings[i] < config_.outlier_close_range_in)
                ? config_.outlier_close_cap_in
                : readings[i] * config_.outlier_far_cap_fraction;

            // If ray returns infinity (particle outside field), treat as huge error
            if (!std::isfinite(predicted)) {
                errors[i] = outlier_huge_fallback;
                below_cap[i] = false;
                continue;
            }

            errors[i] = std::fabs(predicted - readings[i]);
            below_cap[i] = (errors[i] <= max_err_i);

            const double base_sigma = (readings[i] < config_.sigma_close_range_in)
                ? config_.sigma_close_in
                : readings[i] * config_.sigma_far_fraction;
            const double sigma_heading =
                predicted * std::fabs(std::sin(heading_unc_rad));
            sigma_i[i] = std::sqrt(base_sigma * base_sigma + sigma_heading * sigma_heading)
                * config_.sigma_calibration_margin;
        }

        // Score every valid sensor individually.
        // Inlier (below cap): contributes normalized squared error.
        // Outlier (above cap): multiplies weight by a small penalty.
        double inlier_norm_err_sq_sum = 0.0;
        int inlier_count = 0;
        int outlier_count = 0;

        for (int i = 0; i < 4; i++) {
            if (!valid[i]) continue;
            if (below_cap[i]) {
                const double inv_sigma = 1.0 / std::max(sigma_i[i], 1e-9);
                const double z = errors[i] * inv_sigma;
                inlier_norm_err_sq_sum += z * z;
                inlier_count++;
            } else {
                outlier_count++;
            }
        }

        // Compute weight: Gaussian on mean normalized error, penalty for outliers
        constexpr double kOutlierPenalty = 0.01;
        double log_w = 0.0;
        if (inlier_count > 0) {
            const double mean_norm_err_sq = inlier_norm_err_sq_sum / inlier_count;
            log_w = -mean_norm_err_sq / 2.0;
        }
        log_w += outlier_count * std::log(kOutlierPenalty);
        p.weight = static_cast<float>(std::exp(log_w));

        weight_sum += p.weight;
    }

    last_raw_weight_sum_ = weight_sum;

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

    double neff_ratio = n_eff() / N;
    const bool bootstrap_active = (bootstrap_ticks_left_ > 0);
    if (bootstrap_active) {
        bootstrap_ticks_left_--;
    }

    const double avg_raw_weight = last_raw_weight_sum_ / N;
    // Only flag "lost" when at least one update() has run (sum > 0 means
    // update has scored particles).  Before any update, sum == 0 and we
    // should NOT treat the freshly initialized filter as lost.
    const bool filter_lost = (last_raw_weight_sum_ > 0.0)
        && (avg_raw_weight < config_.lost_weight_threshold);

    // Always resample during bootstrap or when filter is lost (all particles
    // scored near-zero).  The lost case is critical: when every particle is
    // outside the field they all receive equal junk weights, n_eff = N, and
    // the normal threshold check would skip resampling — leaving particles
    // stuck outside the field permanently.
    if (filter_lost) {
        // Force resampling regardless of n_eff
    } else {
        const double active_threshold = bootstrap_active
            ? 0.999999
            : config_.resample_threshold;
        if (neff_ratio >= active_threshold) return;
    }

    // Low-variance resampling
    std::vector<Particle> new_particles;
    const double injection = filter_lost
        ? std::max(config_.random_injection, config_.lost_random_injection)
        : config_.random_injection;
    int num_resampled = N - static_cast<int>(injection * N);
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

ClusterStats MCLEngine::cluster_stats() const {
    const Estimate c = estimate();
    const int N = static_cast<int>(particles_.size());
    if (N == 0) return {0.0, 0.0, c};

    // Weighted RMS spread and (distance, weight) pairs for percentile calc
    double wt_dist_sq_sum = 0.0;
    std::vector<std::pair<double, double>> dist_weight;
    dist_weight.reserve(static_cast<size_t>(N));

    for (const auto& p : particles_) {
        const double dx = p.x - c.x;
        const double dy = p.y - c.y;
        const double d2 = dx * dx + dy * dy;
        wt_dist_sq_sum += p.weight * d2;
        dist_weight.push_back({std::sqrt(d2), static_cast<double>(p.weight)});
    }

    const double spread = std::sqrt(wt_dist_sq_sum);

    // Sort by distance and walk cumulative weight to find 90th percentile
    std::sort(dist_weight.begin(), dist_weight.end());
    double cum = 0.0;
    double r90 = dist_weight.back().first;
    for (const auto& [d, w] : dist_weight) {
        cum += w;
        if (cum >= 0.9) {
            r90 = d;
            break;
        }
    }

    return {spread, r90, c};
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

const MCLConfig& MCLEngine::config() const {
    return config_;
}

} // namespace mcl
