#include "sim/field.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace sim {

namespace {
bool point_in_aabb(double x, double y, const AABB& box) {
    return x >= box.min_x && x <= box.max_x && y >= box.min_y && y <= box.max_y;
}

// Liang-Barsky segment-vs-AABB clip test.
bool segment_intersects_aabb(double x0, double y0, double x1, double y1, const AABB& box) {
    double t0 = 0.0;
    double t1 = 1.0;
    const double dx = x1 - x0;
    const double dy = y1 - y0;

    auto clip = [&](double p, double q) -> bool {
        constexpr double eps = 1e-12;
        if (std::fabs(p) < eps) {
            return q >= 0.0;
        }
        const double r = q / p;
        if (p < 0.0) {
            if (r > t1) return false;
            if (r > t0) t0 = r;
        } else {
            if (r < t0) return false;
            if (r < t1) t1 = r;
        }
        return true;
    };

    if (!clip(-dx, x0 - box.min_x)) return false;
    if (!clip( dx, box.max_x - x0)) return false;
    if (!clip(-dy, y0 - box.min_y)) return false;
    if (!clip( dy, box.max_y - y0)) return false;
    return t0 <= t1;
}
} // namespace

bool Field::is_inside(double x, double y) const {
    const double limit = field_half - robot_radius;
    return (x >= -limit && x <= limit && y >= -limit && y <= limit);
}

void Field::clamp(double& x, double& y) const {
    const double limit = field_half - robot_radius;
    x = std::max(-limit, std::min(limit, x));
    y = std::max(-limit, std::min(limit, y));
}

bool Field::is_passable(double x, double y) const {
    if (!is_inside(x, y)) return false;
    for (const auto& box : obstacles) {
        if (point_in_aabb(x, y, box)) return false;
    }
    return true;
}

bool Field::segment_hits_obstacle(double x0, double y0, double x1, double y1) const {
    for (const auto& box : obstacles) {
        if (segment_intersects_aabb(x0, y0, x1, y1, box)) return true;
    }
    return false;
}

} // namespace sim
