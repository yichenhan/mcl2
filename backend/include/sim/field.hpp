#pragma once

#include "nlohmann/json.hpp"

#include <string>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

namespace sim {

struct AABB {
    double min_x = 0.0;
    double min_y = 0.0;
    double max_x = 0.0;
    double max_y = 0.0;
};

struct Circle {
    double cx = 0.0;
    double cy = 0.0;
    double radius = 0.0;
};

struct Triangle {
    double x0 = 0.0;
    double y0 = 0.0;
    double x1 = 0.0;
    double y1 = 0.0;
    double x2 = 0.0;
    double y2 = 0.0;
};

struct Obstacle {
    std::variant<AABB, Circle, Triangle> shape;
    std::string color;

    Obstacle() = default;
    Obstacle(double min_x, double min_y, double max_x, double max_y)
        : shape(AABB{min_x, min_y, max_x, max_y}), color("") {}
    Obstacle(const AABB& aabb, std::string c = "") : shape(aabb), color(std::move(c)) {}
    Obstacle(const Circle& circle, std::string c = "") : shape(circle), color(std::move(c)) {}
    Obstacle(const Triangle& triangle, std::string c = "") : shape(triangle), color(std::move(c)) {}
};

struct Field {
    double field_half = 72.0;
    double robot_radius = 8.0;
    std::vector<Obstacle> obstacles;

    bool is_inside(double x, double y) const;
    void clamp(double& x, double& y) const;
    bool is_passable(double x, double y) const;
    bool segment_hits_obstacle(double x0, double y0, double x1, double y1) const;
};

inline void to_json(nlohmann::json& j, const AABB& b) {
    j = nlohmann::json{
        { "min_x", b.min_x },
        { "min_y", b.min_y },
        { "max_x", b.max_x },
        { "max_y", b.max_y },
    };
}

inline void from_json(const nlohmann::json& j, AABB& b) {
    b.min_x = j.value("min_x", 0.0);
    b.min_y = j.value("min_y", 0.0);
    b.max_x = j.value("max_x", 0.0);
    b.max_y = j.value("max_y", 0.0);
}

inline void to_json(nlohmann::json& j, const Circle& c) {
    j = nlohmann::json{
        { "cx", c.cx },
        { "cy", c.cy },
        { "radius", c.radius },
    };
}

inline void from_json(const nlohmann::json& j, Circle& c) {
    c.cx = j.value("cx", 0.0);
    c.cy = j.value("cy", 0.0);
    c.radius = j.value("radius", 0.0);
}

inline void to_json(nlohmann::json& j, const Triangle& t) {
    j = nlohmann::json{
        { "x0", t.x0 },
        { "y0", t.y0 },
        { "x1", t.x1 },
        { "y1", t.y1 },
        { "x2", t.x2 },
        { "y2", t.y2 },
    };
}

inline void from_json(const nlohmann::json& j, Triangle& t) {
    t.x0 = j.value("x0", 0.0);
    t.y0 = j.value("y0", 0.0);
    t.x1 = j.value("x1", 0.0);
    t.y1 = j.value("y1", 0.0);
    t.x2 = j.value("x2", 0.0);
    t.y2 = j.value("y2", 0.0);
}

inline void to_json(nlohmann::json& j, const Obstacle& o) {
    j = nlohmann::json::object();
    std::visit(
        [&](const auto& shape) {
            using T = std::decay_t<decltype(shape)>;
            if constexpr (std::is_same_v<T, AABB>) {
                j = nlohmann::json(shape);
                j["type"] = "rect";
            } else if constexpr (std::is_same_v<T, Circle>) {
                j = nlohmann::json(shape);
                j["type"] = "circle";
            } else if constexpr (std::is_same_v<T, Triangle>) {
                j = nlohmann::json(shape);
                j["type"] = "triangle";
            }
        },
        o.shape);
    if (!o.color.empty()) {
        j["color"] = o.color;
    }
}

inline void from_json(const nlohmann::json& j, Obstacle& o) {
    const std::string type = j.value("type", std::string{});
    if (type == "circle") {
        o.shape = j.get<Circle>();
    } else if (type == "triangle") {
        o.shape = j.get<Triangle>();
    } else {
        // Backward-compatible default: treat untyped objects as AABB.
        o.shape = j.get<AABB>();
    }
    o.color = j.value("color", std::string{});
}

} // namespace sim
