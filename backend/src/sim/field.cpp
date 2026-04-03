#include "sim/field.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <type_traits>

namespace sim {

namespace {
constexpr double kEps = 1e-12;

bool point_in_aabb(double x, double y, const AABB& box) {
    return x >= box.min_x && x <= box.max_x && y >= box.min_y && y <= box.max_y;
}

bool point_in_circle(double x, double y, const Circle& c) {
    const double dx = x - c.cx;
    const double dy = y - c.cy;
    return (dx * dx + dy * dy) <= c.radius * c.radius;
}

double orient2d(double ax, double ay, double bx, double by, double cx, double cy) {
    return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
}

bool triangle_degenerate(const Triangle& t) {
    const double area2 = orient2d(t.x0, t.y0, t.x1, t.y1, t.x2, t.y2);
    return std::fabs(area2) < kEps;
}

bool point_in_triangle(double x, double y, const Triangle& t) {
    if (triangle_degenerate(t)) return false;
    const double d1 = orient2d(t.x0, t.y0, t.x1, t.y1, x, y);
    const double d2 = orient2d(t.x1, t.y1, t.x2, t.y2, x, y);
    const double d3 = orient2d(t.x2, t.y2, t.x0, t.y0, x, y);
    const bool has_neg = (d1 < -kEps) || (d2 < -kEps) || (d3 < -kEps);
    const bool has_pos = (d1 > kEps) || (d2 > kEps) || (d3 > kEps);
    return !(has_neg && has_pos);
}

// Liang-Barsky segment-vs-AABB clip test.
bool segment_intersects_aabb(double x0, double y0, double x1, double y1, const AABB& box) {
    double t0 = 0.0;
    double t1 = 1.0;
    const double dx = x1 - x0;
    const double dy = y1 - y0;

    auto clip = [&](double p, double q) -> bool {
        if (std::fabs(p) < kEps) {
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

bool segment_intersects_circle(double x0, double y0, double x1, double y1, const Circle& c) {
    const double dx = x1 - x0;
    const double dy = y1 - y0;
    const double fx = x0 - c.cx;
    const double fy = y0 - c.cy;
    const double a = dx * dx + dy * dy;
    if (a < kEps) {
        return point_in_circle(x0, y0, c);
    }
    const double b = 2.0 * (fx * dx + fy * dy);
    const double cc = fx * fx + fy * fy - c.radius * c.radius;
    double disc = b * b - 4.0 * a * cc;
    if (disc < 0.0) return false;
    disc = std::max(0.0, disc);
    const double root = std::sqrt(disc);
    const double t1 = (-b - root) / (2.0 * a);
    const double t2 = (-b + root) / (2.0 * a);
    return (t1 >= 0.0 && t1 <= 1.0) || (t2 >= 0.0 && t2 <= 1.0);
}

bool on_segment(double ax, double ay, double bx, double by, double px, double py) {
    return px >= std::min(ax, bx) - kEps && px <= std::max(ax, bx) + kEps &&
           py >= std::min(ay, by) - kEps && py <= std::max(ay, by) + kEps;
}

bool segments_intersect(
    double ax,
    double ay,
    double bx,
    double by,
    double cx,
    double cy,
    double dx,
    double dy) {
    const double o1 = orient2d(ax, ay, bx, by, cx, cy);
    const double o2 = orient2d(ax, ay, bx, by, dx, dy);
    const double o3 = orient2d(cx, cy, dx, dy, ax, ay);
    const double o4 = orient2d(cx, cy, dx, dy, bx, by);

    const bool straddle1 = (o1 > kEps && o2 < -kEps) || (o1 < -kEps && o2 > kEps);
    const bool straddle2 = (o3 > kEps && o4 < -kEps) || (o3 < -kEps && o4 > kEps);
    if (straddle1 && straddle2) return true;
    if (std::fabs(o1) < kEps && on_segment(ax, ay, bx, by, cx, cy)) return true;
    if (std::fabs(o2) < kEps && on_segment(ax, ay, bx, by, dx, dy)) return true;
    if (std::fabs(o3) < kEps && on_segment(cx, cy, dx, dy, ax, ay)) return true;
    if (std::fabs(o4) < kEps && on_segment(cx, cy, dx, dy, bx, by)) return true;
    return false;
}

bool segment_intersects_triangle(double x0, double y0, double x1, double y1, const Triangle& t) {
    if (triangle_degenerate(t)) return false;
    if (point_in_triangle(x0, y0, t) || point_in_triangle(x1, y1, t)) return true;
    return segments_intersect(x0, y0, x1, y1, t.x0, t.y0, t.x1, t.y1) ||
           segments_intersect(x0, y0, x1, y1, t.x1, t.y1, t.x2, t.y2) ||
           segments_intersect(x0, y0, x1, y1, t.x2, t.y2, t.x0, t.y0);
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
    for (const auto& obstacle : obstacles) {
        const bool blocked = std::visit(
            [&](const auto& shape) -> bool {
                using T = std::decay_t<decltype(shape)>;
                if constexpr (std::is_same_v<T, AABB>) {
                    return point_in_aabb(x, y, shape);
                } else if constexpr (std::is_same_v<T, Circle>) {
                    return point_in_circle(x, y, shape);
                } else {
                    return point_in_triangle(x, y, shape);
                }
            },
            obstacle.shape);
        if (blocked) return false;
    }
    return true;
}

bool Field::segment_hits_obstacle(double x0, double y0, double x1, double y1) const {
    for (const auto& obstacle : obstacles) {
        const bool hit = std::visit(
            [&](const auto& shape) -> bool {
                using T = std::decay_t<decltype(shape)>;
                if constexpr (std::is_same_v<T, AABB>) {
                    return segment_intersects_aabb(x0, y0, x1, y1, shape);
                } else if constexpr (std::is_same_v<T, Circle>) {
                    return segment_intersects_circle(x0, y0, x1, y1, shape);
                } else {
                    return segment_intersects_triangle(x0, y0, x1, y1, shape);
                }
            },
            obstacle.shape);
        if (hit) return true;
    }
    return false;
}

} // namespace sim
