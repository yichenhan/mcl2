#include "doctest/doctest.h"
#include "noise/noise_generator.hpp"

#include <cmath>
#include <vector>

using namespace noise;

TEST_CASE("NoiseGenerator gaussian: stddev zero returns mean exactly") {
    NoiseGenerator rng(123);
    for (int i = 0; i < 100; ++i) {
        CHECK(rng.gaussian(2.5, 0.0) == doctest::Approx(2.5));
    }
}

TEST_CASE("NoiseGenerator uniform: values are within bounds") {
    NoiseGenerator rng(42);
    for (int i = 0; i < 5000; ++i) {
        double v = rng.uniform(-3.0, 7.0);
        CHECK(v >= -3.0);
        CHECK(v <= 7.0);
    }
}

TEST_CASE("NoiseGenerator bernoulli: p=0 always false, p=1 always true") {
    NoiseGenerator rng(7);
    for (int i = 0; i < 1000; ++i) {
        CHECK_FALSE(rng.bernoulli(0.0));
        CHECK(rng.bernoulli(1.0));
    }
}

TEST_CASE("NoiseGenerator deterministic: same seed gives identical sequence") {
    NoiseGenerator a(2024);
    NoiseGenerator b(2024);

    for (int i = 0; i < 200; ++i) {
        CHECK(a.gaussian(0.0, 1.0) == b.gaussian(0.0, 1.0));
        CHECK(a.uniform(-1.0, 1.0) == b.uniform(-1.0, 1.0));
        CHECK(a.bernoulli(0.35) == b.bernoulli(0.35));
    }
}

TEST_CASE("NoiseGenerator deterministic: reseed reproduces sequence") {
    NoiseGenerator rng(99);
    std::vector<double> seq_a;
    for (int i = 0; i < 20; ++i) {
        seq_a.push_back(rng.gaussian(0.5, 2.0));
    }

    rng.reseed(99);
    for (int i = 0; i < 20; ++i) {
        CHECK(rng.gaussian(0.5, 2.0) == seq_a[i]);
    }
}

TEST_CASE("NoiseGenerator statistical: gaussian mean and stddev are reasonable") {
    NoiseGenerator rng(314159);
    constexpr int N = 20000;
    constexpr double mu = 3.0;
    constexpr double sigma = 1.2;

    double sum = 0.0;
    double sum_sq = 0.0;
    for (int i = 0; i < N; ++i) {
        double x = rng.gaussian(mu, sigma);
        sum += x;
        sum_sq += x * x;
    }

    double mean = sum / N;
    double var = (sum_sq / N) - mean * mean;
    double stddev = std::sqrt(std::max(0.0, var));

    CHECK(mean == doctest::Approx(mu).epsilon(0.03));
    CHECK(stddev == doctest::Approx(sigma).epsilon(0.08));
}

TEST_CASE("NoiseGenerator statistical: uniform mean is reasonable") {
    NoiseGenerator rng(271828);
    constexpr int N = 20000;
    constexpr double lo = -4.0;
    constexpr double hi = 10.0;
    constexpr double expected = (lo + hi) / 2.0;

    double sum = 0.0;
    for (int i = 0; i < N; ++i) {
        sum += rng.uniform(lo, hi);
    }
    double mean = sum / N;

    CHECK(mean == doctest::Approx(expected).epsilon(0.03));
}

TEST_CASE("NoiseGenerator statistical: bernoulli frequency tracks probability") {
    NoiseGenerator rng(123456);
    constexpr int N = 20000;
    constexpr double p = 0.27;

    int trues = 0;
    for (int i = 0; i < N; ++i) {
        if (rng.bernoulli(p)) trues++;
    }
    double observed = static_cast<double>(trues) / N;
    CHECK(observed == doctest::Approx(p).epsilon(0.08));
}
