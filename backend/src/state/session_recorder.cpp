#include "state/session_recorder.hpp"

#include <cstdio>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <random>
#include <sstream>
#include <utility>

namespace state {

SessionRecorder::SessionRecorder(std::string directory, std::string session_id)
    : directory_(std::move(directory)), session_id_(std::move(session_id)) {}

namespace {

std::string sanitize_token(const std::string& in, const std::string& fallback) {
    std::string out;
    out.reserve(in.size());
    for (char c : in) {
        if ((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9')) {
            out.push_back(c);
            continue;
        }
        if (c >= 'A' && c <= 'Z') {
            out.push_back(static_cast<char>(c - 'A' + 'a'));
            continue;
        }
        if (c == '_' || c == '-') {
            out.push_back(c);
        }
    }
    return out.empty() ? fallback : out;
}

std::string join_failures(const std::vector<std::string>& failures) {
    std::ostringstream ss;
    for (size_t i = 0; i < failures.size(); ++i) {
        if (i > 0) ss << ",";
        ss << failures[i];
    }
    return ss.str();
}

TickState strip_particles_for_replay(const TickState& tick) {
    TickState out = tick;
    out.post_predict.particles.clear();
    out.post_update.particles.clear();
    out.post_resample.particles.clear();
    return out;
}

} // namespace

std::string SessionRecorder::generate_session_name(const std::string& mode, const std::string& optional_label) {
    const std::string mode_token = sanitize_token(mode, "live");
    const std::string label_token = sanitize_token(optional_label, "");

    const auto now = std::chrono::system_clock::now();
    const std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm tm_buf{};
#if defined(_WIN32)
    localtime_s(&tm_buf, &t);
#else
    localtime_r(&t, &tm_buf);
#endif

    std::random_device rd;
    std::uniform_int_distribution<int> dist(0, 0xFFF);
    const int suffix = dist(rd);

    std::ostringstream ss;
    ss << mode_token << "_";
    if (!label_token.empty()) {
        ss << label_token << "_";
    }
    ss << std::put_time(&tm_buf, "%Y-%m-%d_%H-%M-%S")
       << "_" << std::hex << std::nouppercase << std::setw(3) << std::setfill('0') << suffix;
    return ss.str();
}

void SessionRecorder::set_config(const nlohmann::json& config_json) {
    config_json_ = config_json;
}

void SessionRecorder::set_obstacles(const nlohmann::json& obstacles_json) {
    obstacles_json_ = obstacles_json;
}

void SessionRecorder::set_analytics(const SessionAnalytics& analytics) {
    nlohmann::json j;
    to_json(j, analytics);
    analytics_json_ = std::move(j);
    has_analytics_ = true;
}

void SessionRecorder::record(const TickState& tick) {
    ticks_.push_back(tick);
}

std::string SessionRecorder::output_path() const {
    return directory_ + "/" + session_id_ + ".json";
}

std::string SessionRecorder::log_path() const {
    return directory_ + "/" + session_id_ + ".txt";
}

int SessionRecorder::tick_count() const {
    return static_cast<int>(ticks_.size());
}

bool SessionRecorder::write_atomic() const {
    std::error_code ec;
    std::filesystem::create_directories(directory_, ec);
    if (ec) return false;

    nlohmann::json j;
    j["session_id"] = session_id_;
    j["config"] = config_json_;
    j["obstacles"] = obstacles_json_;
    j["total_ticks"] = ticks_.size();
    // Replay files can be very large. Persist snapshots without particle clouds.
    nlohmann::json serialized_ticks = nlohmann::json::array();
    for (const auto& tick : ticks_) {
        serialized_ticks.push_back(strip_particles_for_replay(tick));
    }
    j["ticks"] = std::move(serialized_ticks);
    if (has_analytics_) {
        j["analytics"] = analytics_json_;
    }

    const std::string final_path = output_path();
    const std::string tmp_path = final_path + ".tmp";

    std::ofstream out(tmp_path, std::ios::trunc);
    if (!out.is_open()) return false;
    out << j.dump(2);
    out.close();
    if (!out) return false;

    if (std::rename(tmp_path.c_str(), final_path.c_str()) != 0) {
        std::remove(tmp_path.c_str());
        return false;
    }
    return true;
}

bool SessionRecorder::write_log() const {
    std::error_code ec;
    std::filesystem::create_directories(directory_, ec);
    if (ec) return false;

    std::ofstream out(log_path(), std::ios::trunc);
    if (!out.is_open()) return false;

    out << "=== MCL Session: " << session_id_ << " ===\n";
    out << "Mode: " << config_json_.value("mode", std::string("live")) << "\n";
    out << "Seed: " << config_json_.value("seed", 0) << "\n";
    out << "Particles: " << config_json_.value("num_particles", 0) << "\n";
    out << "Map: " << config_json_.value("map_name", std::string("custom")) << "\n";
    out << "Algorithm: " << config_json_.value("algorithm", std::string("manual")) << "\n";
    out << "Total ticks: " << ticks_.size() << "\n\n";

    out << "Tick |  MCL Err | Odom Err | Gate     | R90   | Failures\n";
    for (const auto& tick : ticks_) {
        out << std::setw(4) << tick.tick << " | "
            << std::setw(8) << std::fixed << std::setprecision(3) << tick.mcl_error << " | "
            << std::setw(8) << std::fixed << std::setprecision(3) << tick.odom_error << " | "
            << std::setw(8) << (tick.gate_decision.accepted ? "accepted" : "REJECTED") << " | "
            << std::setw(5) << std::fixed << std::setprecision(2) << tick.cluster_stats.radius_90 << " | "
            << join_failures(tick.active_failures) << "\n";
    }

    if (!ticks_.empty()) {
        double total = 0.0;
        for (const auto& tick : ticks_) total += tick.mcl_error;
        out << "\nFinal MCL error: " << std::fixed << std::setprecision(3) << ticks_.back().mcl_error << "\n";
        out << "Mean MCL error: " << std::fixed << std::setprecision(3) << (total / static_cast<double>(ticks_.size())) << "\n";
    }

    out.close();
    return static_cast<bool>(out);
}

} // namespace state
