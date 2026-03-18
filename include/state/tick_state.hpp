#pragma once

#include "mcl/mcl_engine.hpp"
#include "sim/physics.hpp"

#include "nlohmann/json.hpp"

#include <array>
#include <string>
#include <vector>

namespace state {

struct MCLSnapshot {
    std::vector<mcl::Particle> particles;
    mcl::Estimate estimate{ 0.0f, 0.0f };
    double n_eff = 0.0;
};

struct TickState {
    int tick = 0;
    sim::RobotState ground_truth{};
    std::array<double, 4> observed_readings{ -1.0, -1.0, -1.0, -1.0 };
    double observed_heading = 0.0;
    std::vector<std::string> active_failures;

    MCLSnapshot post_predict;
    MCLSnapshot post_update;
    MCLSnapshot post_resample;

    double mcl_error = 0.0;
    double odom_error = 0.0;
    int valid_sensor_count = 0;
};

inline void to_json(nlohmann::json& j, const MCLSnapshot& s) {
    j = nlohmann::json::object();
    j["particles"] = nlohmann::json::array();
    for (const auto& p : s.particles) {
        j["particles"].push_back({
            { "x", p.x },
            { "y", p.y },
            { "weight", p.weight },
        });
    }
    j["estimate"] = {
        { "x", s.estimate.x },
        { "y", s.estimate.y },
    };
    j["n_eff"] = s.n_eff;
}

inline void from_json(const nlohmann::json& j, MCLSnapshot& s) {
    s.particles.clear();
    for (const auto& p : j.at("particles")) {
        mcl::Particle part;
        part.x = p.at("x").get<float>();
        part.y = p.at("y").get<float>();
        part.weight = p.at("weight").get<float>();
        s.particles.push_back(part);
    }
    const auto& est = j.at("estimate");
    s.estimate.x = est.at("x").get<float>();
    s.estimate.y = est.at("y").get<float>();
    s.n_eff = j.at("n_eff").get<double>();
}

inline void to_json(nlohmann::json& j, const TickState& t) {
    j = nlohmann::json::object();
    j["tick"] = t.tick;
    j["ground_truth"] = {
        { "x", t.ground_truth.x },
        { "y", t.ground_truth.y },
        { "heading_deg", t.ground_truth.heading_deg },
    };
    j["observed_readings"] = t.observed_readings;
    j["observed_heading"] = t.observed_heading;
    j["active_failures"] = t.active_failures;
    j["post_predict"] = t.post_predict;
    j["post_update"] = t.post_update;
    j["post_resample"] = t.post_resample;
    j["mcl_error"] = t.mcl_error;
    j["odom_error"] = t.odom_error;
    j["valid_sensor_count"] = t.valid_sensor_count;
}

inline void from_json(const nlohmann::json& j, TickState& t) {
    t.tick = j.at("tick").get<int>();
    const auto& gt = j.at("ground_truth");
    t.ground_truth.x = gt.at("x").get<double>();
    t.ground_truth.y = gt.at("y").get<double>();
    t.ground_truth.heading_deg = gt.at("heading_deg").get<double>();
    t.observed_readings = j.at("observed_readings").get<std::array<double, 4>>();
    t.observed_heading = j.at("observed_heading").get<double>();
    t.active_failures = j.at("active_failures").get<std::vector<std::string>>();
    t.post_predict = j.at("post_predict").get<MCLSnapshot>();
    t.post_update = j.at("post_update").get<MCLSnapshot>();
    t.post_resample = j.at("post_resample").get<MCLSnapshot>();
    t.mcl_error = j.at("mcl_error").get<double>();
    t.odom_error = j.at("odom_error").get<double>();
    t.valid_sensor_count = j.at("valid_sensor_count").get<int>();
}

} // namespace state
