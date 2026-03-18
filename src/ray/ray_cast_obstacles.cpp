#include "ray/ray_cast_obstacles.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace ray {

namespace {
bool point_in_aabb(const distance_loc::Vec2& p, const sim::AABB& b) {
    return p.x >= b.min_x && p.x <= b.max_x && p.y >= b.min_y && p.y <= b.max_y;
}

double ray_aabb_intersection_t(const distance_loc::Vec2& origin,
                               const distance_loc::Vec2& dir,
                               const sim::AABB& b) {
    constexpr double inf = std::numeric_limits<double>::infinity();
    constexpr double eps = 1e-12;

    if (point_in_aabb(origin, b)) return 0.0;

    double tmin = 0.0;
    double tmax = inf;

    auto axis_clip = [&](double o, double d, double mn, double mx) -> bool {
        if (std::fabs(d) < eps) {
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
} // namespace

double ray_distance_with_obstacles(
    const distance_loc::Vec2& robot_pos,
    double theta_deg,
    const distance_loc::Vec2& sensor_offset,
    double sensor_rel_deg,
    const std::vector<sim::AABB>& obstacles) {
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
    for (const auto& box : obstacles) {
        const double t = ray_aabb_intersection_t(sensor_pos, dir, box);
        if (std::isfinite(t) && t >= 0.0) {
            best = std::min(best, t);
        }
    }
    return best;
}

} // namespace ray
