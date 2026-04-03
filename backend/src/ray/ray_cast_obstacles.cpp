#include "ray/ray_cast_obstacles.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <type_traits>

namespace ray {

namespace {
constexpr double kEps = 1e-12;

bool point_in_aabb(const distance_loc::Vec2& p, const sim::AABB& b) {
    return p.x >= b.min_x && p.x <= b.max_x && p.y >= b.min_y && p.y <= b.max_y;
}

bool point_in_circle(const distance_loc::Vec2& p, const sim::Circle& c) {
    const double dx = p.x - c.cx;
    const double dy = p.y - c.cy;
    return (dx * dx + dy * dy) <= c.radius * c.radius;
}

double orient2d(double ax, double ay, double bx, double by, double cx, double cy) {
    return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
}

bool triangle_degenerate(const sim::Triangle& t) {
    const double area2 = orient2d(t.x0, t.y0, t.x1, t.y1, t.x2, t.y2);
    return std::fabs(area2) < kEps;
}

bool point_in_triangle(const distance_loc::Vec2& p, const sim::Triangle& t) {
    if (triangle_degenerate(t)) return false;
    const double d1 = orient2d(t.x0, t.y0, t.x1, t.y1, p.x, p.y);
    const double d2 = orient2d(t.x1, t.y1, t.x2, t.y2, p.x, p.y);
    const double d3 = orient2d(t.x2, t.y2, t.x0, t.y0, p.x, p.y);
    const bool has_neg = (d1 < -kEps) || (d2 < -kEps) || (d3 < -kEps);
    const bool has_pos = (d1 > kEps) || (d2 > kEps) || (d3 > kEps);
    return !(has_neg && has_pos);
}

double ray_aabb_intersection_t(const distance_loc::Vec2& origin,
                               const distance_loc::Vec2& dir,
                               const sim::AABB& b) {
    constexpr double inf = std::numeric_limits<double>::infinity();

    if (point_in_aabb(origin, b)) return 0.0;

    double tmin = 0.0;
    double tmax = inf;

    auto axis_clip = [&](double o, double d, double mn, double mx) -> bool {
        if (std::fabs(d) < kEps) {
            return o >= mn && o <= mx;
        }
        double t1 = (mn - o) / d;
        double t2 = (mx - o) / d;
        if (t1 > t2) std::swap(t1, t2);
        if (t1 > tmin) tmin = t1;
        if (t2 < tmax) tmax = t2;
        return tmin <= tmax;
    };

    if (!axis_clip(origin.x, dir.x, b.min_x, b.max_x)) return inf;
    if (!axis_clip(origin.y, dir.y, b.min_y, b.max_y)) return inf;

    if (tmax < 0.0) return inf;
    return (tmin >= 0.0) ? tmin : tmax;
}

double ray_circle_intersection_t(
    const distance_loc::Vec2& origin,
    const distance_loc::Vec2& dir,
    const sim::Circle& c) {
    constexpr double inf = std::numeric_limits<double>::infinity();
    if (point_in_circle(origin, c)) return 0.0;

    const double ox = origin.x - c.cx;
    const double oy = origin.y - c.cy;
    const double a = dir.x * dir.x + dir.y * dir.y;
    if (a < kEps) return inf;
    const double b = 2.0 * (ox * dir.x + oy * dir.y);
    const double cc = ox * ox + oy * oy - c.radius * c.radius;
    double disc = b * b - 4.0 * a * cc;
    if (disc < 0.0) return inf;
    disc = std::max(0.0, disc);
    const double root = std::sqrt(disc);
    const double t1 = (-b - root) / (2.0 * a);
    const double t2 = (-b + root) / (2.0 * a);
    if (t1 >= 0.0) return t1;
    if (t2 >= 0.0) return t2;
    return inf;
}

double cross2d(const distance_loc::Vec2& a, const distance_loc::Vec2& b) {
    return a.x * b.y - a.y * b.x;
}

double ray_segment_intersection_t(
    const distance_loc::Vec2& origin,
    const distance_loc::Vec2& dir,
    const distance_loc::Vec2& a,
    const distance_loc::Vec2& b) {
    constexpr double inf = std::numeric_limits<double>::infinity();
    const distance_loc::Vec2 e{ b.x - a.x, b.y - a.y };
    const distance_loc::Vec2 ao{ a.x - origin.x, a.y - origin.y };
    const double denom = cross2d(dir, e);
    if (std::fabs(denom) < kEps) return inf;
    const double t = cross2d(ao, e) / denom;
    const double u = cross2d(ao, dir) / denom;
    if (t < 0.0) return inf;
    if (u < -kEps || u > 1.0 + kEps) return inf;
    return t;
}

double ray_triangle_intersection_t(
    const distance_loc::Vec2& origin,
    const distance_loc::Vec2& dir,
    const sim::Triangle& t) {
    constexpr double inf = std::numeric_limits<double>::infinity();
    if (triangle_degenerate(t)) return inf;
    if (point_in_triangle(origin, t)) return 0.0;
    const distance_loc::Vec2 v0{ t.x0, t.y0 };
    const distance_loc::Vec2 v1{ t.x1, t.y1 };
    const distance_loc::Vec2 v2{ t.x2, t.y2 };
    const double t01 = ray_segment_intersection_t(origin, dir, v0, v1);
    const double t12 = ray_segment_intersection_t(origin, dir, v1, v2);
    const double t20 = ray_segment_intersection_t(origin, dir, v2, v0);
    return std::min(t01, std::min(t12, t20));
}
} // namespace

double ray_distance_with_obstacles(
    const distance_loc::Vec2& robot_pos,
    double theta_deg,
    const distance_loc::Vec2& sensor_offset,
    double sensor_rel_deg,
    const std::vector<sim::Obstacle>& obstacles) {
    const distance_loc::Vec2 offset_field =
        distance_loc::rotateOffsetByHeading(sensor_offset, theta_deg);
    const distance_loc::Vec2 sensor_pos{
        robot_pos.x + offset_field.x,
        robot_pos.y + offset_field.y
    };

    const double ray_angle_deg = theta_deg + sensor_rel_deg;
    const double a = distance_loc::deg2rad(ray_angle_deg);
    const distance_loc::Vec2 dir{ std::sin(a), std::cos(a) };

    double best = distance_loc::rayDistanceToSquareWalls(sensor_pos, dir);
    for (const auto& obstacle : obstacles) {
        const double t = std::visit(
            [&](const auto& shape) -> double {
                using T = std::decay_t<decltype(shape)>;
                if constexpr (std::is_same_v<T, sim::AABB>) {
                    return ray_aabb_intersection_t(sensor_pos, dir, shape);
                } else if constexpr (std::is_same_v<T, sim::Circle>) {
                    return ray_circle_intersection_t(sensor_pos, dir, shape);
                } else {
                    return ray_triangle_intersection_t(sensor_pos, dir, shape);
                }
            },
            obstacle.shape);
        if (std::isfinite(t) && t >= 0.0) {
            best = std::min(best, t);
        }
    }
    return best;
}

} // namespace ray
