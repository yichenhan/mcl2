#pragma once

#include "distance_localization.hpp"
#include "sim/field.hpp"

#include <vector>

namespace ray {

double ray_distance_with_obstacles(
    const distance_loc::Vec2& robot_pos,
    double theta_deg,
    const distance_loc::Vec2& sensor_offset,
    double sensor_rel_deg,
    const std::vector<sim::AABB>& obstacles
);

} // namespace ray
