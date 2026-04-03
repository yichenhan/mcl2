#pragma once

#include "mcl/mcl_controller.hpp"
#include "mcl/mcl_engine.hpp"
#include "sim/physics.hpp"

#include "nlohmann/json.hpp"

#include <array>
#include <string>
#include <vector>

namespace state {

using MCLSnapshot = mcl::PhaseSnapshot;

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
    double accepted_error = 0.0;
    double odom_error = 0.0;
    int valid_sensor_count = 0;
    bool update_skipped = false;
    bool pose_gated = false;
    std::string timestamp_iso;
    mcl::Pose odom_pose{};
    mcl::Estimate raw_estimate{};
    mcl::Estimate accepted_estimate{};
    mcl::GateDecision gate_decision{};
    mcl::ClusterStats cluster_stats{};
    std::array<double, 4> mcl_predicted_readings{ -1.0, -1.0, -1.0, -1.0 };
    std::array<double, 4> sensor_residuals{ 0.0, 0.0, 0.0, 0.0 };
    // MCL-corrected pose used for all movement decisions.
    // This is what PoseCorrectionController writes to the chassis after each tick.
    mcl::Pose chassis_pose{};
};

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
    j["accepted_error"] = t.accepted_error;
    j["odom_error"] = t.odom_error;
    j["valid_sensor_count"] = t.valid_sensor_count;
    j["update_skipped"] = t.update_skipped;
    j["pose_gated"] = t.pose_gated;
    j["timestamp_iso"] = t.timestamp_iso;
    j["odom_pose"] = t.odom_pose;
    j["raw_estimate"] = {
        { "x", t.raw_estimate.x },
        { "y", t.raw_estimate.y },
    };
    j["accepted_estimate"] = {
        { "x", t.accepted_estimate.x },
        { "y", t.accepted_estimate.y },
    };
    j["gate_decision"] = t.gate_decision;
    j["cluster_stats"] = {
        { "spread", t.cluster_stats.spread },
        { "radius_90", t.cluster_stats.radius_90 },
        { "centroid", {
            { "x", t.cluster_stats.centroid.x },
            { "y", t.cluster_stats.centroid.y },
        } },
    };
    j["mcl_predicted_readings"] = t.mcl_predicted_readings;
    j["sensor_residuals"] = t.sensor_residuals;
    j["chassis_pose"] = t.chassis_pose;
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
    t.accepted_error = j.value("accepted_error", 0.0);
    t.odom_error = j.at("odom_error").get<double>();
    t.valid_sensor_count = j.at("valid_sensor_count").get<int>();
    if (j.contains("update_skipped"))
        t.update_skipped = j.at("update_skipped").get<bool>();
    if (j.contains("pose_gated"))
        t.pose_gated = j.at("pose_gated").get<bool>();
    t.timestamp_iso = j.value("timestamp_iso", std::string{});
    t.odom_pose = j.value("odom_pose", mcl::Pose{});
    if (j.contains("raw_estimate")) {
        const auto& raw = j["raw_estimate"];
        t.raw_estimate.x = raw.value("x", 0.0f);
        t.raw_estimate.y = raw.value("y", 0.0f);
    } else {
        t.raw_estimate = mcl::Estimate{};
    }
    if (j.contains("accepted_estimate")) {
        const auto& accepted = j["accepted_estimate"];
        t.accepted_estimate.x = accepted.value("x", 0.0f);
        t.accepted_estimate.y = accepted.value("y", 0.0f);
    } else {
        t.accepted_estimate = mcl::Estimate{};
    }
    t.gate_decision = j.value("gate_decision", mcl::GateDecision{});
    if (j.contains("cluster_stats")) {
        const auto& cs = j["cluster_stats"];
        t.cluster_stats.spread = cs.value("spread", 0.0);
        t.cluster_stats.radius_90 = cs.value("radius_90", 0.0);
        const auto& centroid = cs.contains("centroid") ? cs["centroid"] : nlohmann::json::object();
        t.cluster_stats.centroid.x = centroid.value("x", 0.0f);
        t.cluster_stats.centroid.y = centroid.value("y", 0.0f);
    } else {
        t.cluster_stats = mcl::ClusterStats{};
    }
    t.mcl_predicted_readings = j.value(
        "mcl_predicted_readings",
        std::array<double, 4>{ -1.0, -1.0, -1.0, -1.0 });
    t.sensor_residuals = j.value(
        "sensor_residuals",
        std::array<double, 4>{ 0.0, 0.0, 0.0, 0.0 });
    // chassis_pose: new replays have it directly; old replays fall back to
    // accepted_estimate + observed_heading for backward compatibility.
    if (j.contains("chassis_pose")) {
        t.chassis_pose = j["chassis_pose"].get<mcl::Pose>();
    } else if (j.contains("accepted_estimate")) {
        const auto& ae = j["accepted_estimate"];
        t.chassis_pose.x = ae.value("x", 0.0);
        t.chassis_pose.y = ae.value("y", 0.0);
        t.chassis_pose.theta = j.value("observed_heading", 0.0);
    } else {
        t.chassis_pose = {};
    }
}

} // namespace state
