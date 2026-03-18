#pragma once

#include <vector>

namespace sim {

struct AABB {
    double min_x = 0.0;
    double min_y = 0.0;
    double max_x = 0.0;
    double max_y = 0.0;
};

struct Field {
    double field_half = 72.0;
    double robot_radius = 8.0;
    std::vector<AABB> obstacles;

    bool is_inside(double x, double y) const;
    void clamp(double& x, double& y) const;
    bool is_passable(double x, double y) const;
    bool segment_hits_obstacle(double x0, double y0, double x1, double y1) const;
};

} // namespace sim
