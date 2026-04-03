#include "noise/noise_generator.hpp"

#include <algorithm>

namespace noise {

NoiseGenerator::NoiseGenerator(uint64_t seed)
    : rng_(static_cast<std::mt19937::result_type>(seed)) {}

double NoiseGenerator::gaussian(double mean, double stddev) {
    if (stddev <= 0.0) return mean;
    std::normal_distribution<double> dist(mean, stddev);
    return dist(rng_);
}

double NoiseGenerator::uniform(double lo, double hi) {
    if (hi < lo) std::swap(lo, hi);
    if (lo == hi) return lo;
    std::uniform_real_distribution<double> dist(lo, hi);
    return dist(rng_);
}

bool NoiseGenerator::bernoulli(double probability) {
    probability = std::clamp(probability, 0.0, 1.0);
    std::bernoulli_distribution dist(probability);
    return dist(rng_);
}

void NoiseGenerator::reseed(uint64_t seed) {
    rng_.seed(static_cast<std::mt19937::result_type>(seed));
}

} // namespace noise
