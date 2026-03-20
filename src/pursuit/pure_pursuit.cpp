#include "pursuit/pure_pursuit.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace pursuit {

namespace {
constexpr double kPi = 3.14159265358979323846;
}

PurePursuit::PurePursuit(const PurePursuitConfig& config)
    : config_(config) {}

double PurePursuit::normalize_signed_deg(double deg) {
    while (deg > 180.0) deg -= 360.0;
    while (deg < -180.0) deg += 360.0;
    return deg;
}

double PurePursuit::distance(double x0, double y0, double x1, double y1) {
    const double dx = x1 - x0;
    const double dy = y1 - y0;
    return std::sqrt(dx * dx + dy * dy);
}

double PurePursuit::clamp(double v, double lo, double hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

PurePursuit::PathPoint PurePursuit::closest_point_on_segment(
    double px, double py,
    const distance_loc::Vec2& a,
    const distance_loc::Vec2& b,
    int seg_idx) {
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double len2 = dx * dx + dy * dy;
    double t = 0.0;
    if (len2 > 1e-12) {
        const double ux = px - a.x;
        const double uy = py - a.y;
        t = clamp((ux * dx + uy * dy) / len2, 0.0, 1.0);
    }
    PathPoint out;
    out.x = a.x + t * dx;
    out.y = a.y + t * dy;
    out.seg_idx = seg_idx;
    out.seg_t = t;
    return out;
}

bool PurePursuit::segment_circle_first_intersection(
    double px, double py, double radius,
    const distance_loc::Vec2& a,
    const distance_loc::Vec2& b,
    double min_t,
    PathPoint& out,
    int seg_idx) {
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double fx = a.x - px;
    const double fy = a.y - py;

    const double qa = dx * dx + dy * dy;
    const double qb = 2.0 * (fx * dx + fy * dy);
    const double qc = fx * fx + fy * fy - radius * radius;

    if (qa < 1e-12) return false;
    const double disc = qb * qb - 4.0 * qa * qc;
    if (disc < 0.0) return false;
    const double sqrt_disc = std::sqrt(disc);
    const double inv = 1.0 / (2.0 * qa);
    double t1 = (-qb - sqrt_disc) * inv;
    double t2 = (-qb + sqrt_disc) * inv;
    if (t1 > t2) std::swap(t1, t2);

    double picked_t = std::numeric_limits<double>::infinity();
    const double lo = clamp(min_t, 0.0, 1.0);
    if (t1 >= lo && t1 <= 1.0) picked_t = t1;
    else if (t2 >= lo && t2 <= 1.0) picked_t = t2;
    if (!std::isfinite(picked_t)) return false;

    out.x = a.x + picked_t * dx;
    out.y = a.y + picked_t * dy;
    out.seg_idx = seg_idx;
    out.seg_t = picked_t;
    return true;
}

Command PurePursuit::compute(const sim::RobotState& state, const std::vector<distance_loc::Vec2>& waypoints) {
    Command out{};
    const int n = static_cast<int>(waypoints.size());
    if (n == 0) {
        complete_ = true;
        return out;
    }
    if (complete_) return out;
    if (n == 1) {
        const auto& target = waypoints[0];
        if (distance(state.x, state.y, target.x, target.y) <= config_.waypoint_tolerance) {
            complete_ = true;
            return out;
        }
        const double dx = target.x - state.x;
        const double dy = target.y - state.y;
        const double ld = std::max(1e-6, std::sqrt(dx * dx + dy * dy));
        const double target_heading_deg = std::atan2(dx, dy) * 180.0 / kPi;
        const double alpha_deg = normalize_signed_deg(target_heading_deg - state.heading_deg);
        const double alpha_rad = alpha_deg * kPi / 180.0;
        const double kappa = (2.0 * std::sin(alpha_rad)) / ld;
        out.linear_vel = config_.linear_velocity;
        out.angular_vel_deg = clamp(
            out.linear_vel * kappa * 180.0 / kPi,
            -config_.max_angular_velocity_deg,
            config_.max_angular_velocity_deg);
        return out;
    }

    if (current_idx_ < 1) current_idx_ = 1;
    if (current_idx_ >= n) {
        complete_ = true;
        return out;
    }

    while (current_idx_ < n) {
        const auto& w = waypoints[static_cast<size_t>(current_idx_)];
        if (distance(state.x, state.y, w.x, w.y) > config_.waypoint_tolerance) break;
        current_idx_++;
    }
    if (current_idx_ >= n) {
        complete_ = true;
        return out;
    }

    const int start_seg = std::max(0, current_idx_ - 1);
    PathPoint closest{};
    double best_d2 = std::numeric_limits<double>::infinity();
    for (int i = start_seg; i < n - 1; ++i) {
        const auto c = closest_point_on_segment(
            state.x, state.y,
            waypoints[static_cast<size_t>(i)],
            waypoints[static_cast<size_t>(i + 1)],
            i);
        const double ddx = c.x - state.x;
        const double ddy = c.y - state.y;
        const double d2 = ddx * ddx + ddy * ddy;
        if (d2 < best_d2 || (std::fabs(d2 - best_d2) < 1e-9 && c.seg_idx > closest.seg_idx)) {
            best_d2 = d2;
            closest = c;
        }
    }
    current_idx_ = std::max(current_idx_, closest.seg_idx + 1);

    PathPoint target{};
    bool found_lookahead = false;
    const double ld_cfg = std::max(1e-6, config_.lookahead_distance);
    for (int i = closest.seg_idx; i < n - 1; ++i) {
        const double min_t = (i == closest.seg_idx) ? closest.seg_t : 0.0;
        if (segment_circle_first_intersection(
                state.x, state.y, ld_cfg,
                waypoints[static_cast<size_t>(i)],
                waypoints[static_cast<size_t>(i + 1)],
                min_t, target, i)) {
            found_lookahead = true;
            break;
        }
    }

    if (!found_lookahead) {
        const auto& last = waypoints.back();
        target.x = last.x;
        target.y = last.y;
    }

    const double dx = target.x - state.x;
    const double dy = target.y - state.y;
    const double ld = std::max(1e-6, std::sqrt(dx * dx + dy * dy));

    // Heading convention matches physics: 0 deg = +Y and CW positive.
    const double target_heading_deg = std::atan2(dx, dy) * 180.0 / kPi;
    const double alpha_deg = normalize_signed_deg(target_heading_deg - state.heading_deg);
    const double alpha_rad = alpha_deg * kPi / 180.0;
    const double kappa = (2.0 * std::sin(alpha_rad)) / ld;

    out.linear_vel = config_.linear_velocity;
    out.angular_vel_deg = clamp(
        out.linear_vel * kappa * 180.0 / kPi,
        -config_.max_angular_velocity_deg,
        config_.max_angular_velocity_deg);
    return out;
}

bool PurePursuit::is_complete() const {
    return complete_;
}

int PurePursuit::current_waypoint_index() const {
    return current_idx_;
}

void PurePursuit::reset() {
    current_idx_ = 0;
    complete_ = false;
}

} // namespace pursuit

