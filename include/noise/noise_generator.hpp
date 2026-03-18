#pragma once

#include <cstdint>
#include <random>

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

} // namespace noise
