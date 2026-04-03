#include "state/session_analytics.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace state {

namespace {

double safe_div(double a, double b) {
    if (b <= 1e-12) return (a >= 0.0 ? 1.0 : -1.0);
    return a / b;
}

} // namespace

void to_json(nlohmann::json& j, const SessionAnalytics& a) {
    j = nlohmann::json{
        { "peak_accepted_error", a.peak_accepted_error },
        { "peak_accepted_error_tick", a.peak_accepted_error_tick },
        { "mean_accepted_error", a.mean_accepted_error },
        { "p95_accepted_error", a.p95_accepted_error },
        { "oracle_threshold", a.oracle_threshold },
        { "gate_precision", a.gate_precision },
        { "gate_recall", a.gate_recall },
        { "false_accept_count", a.false_accept_count },
        { "false_accept_threshold", a.false_accept_threshold },
        { "longest_false_accept_streak", a.longest_false_accept_streak },
        { "mean_false_accept_duration", a.mean_false_accept_duration },
        { "peak_error_during_false_accept", a.peak_error_during_false_accept },
        { "false_accept_nearest_gate_r90", a.false_accept_nearest_gate_r90 },
        { "false_accept_nearest_gate_residual", a.false_accept_nearest_gate_residual },
        { "false_accept_nearest_gate_centroid_jump", a.false_accept_nearest_gate_centroid_jump },
        { "false_reject_count", a.false_reject_count },
        { "false_reject_threshold", a.false_reject_threshold },
        { "r90_reject_count", a.r90_reject_count },
        { "residual_reject_count", a.residual_reject_count },
        { "centroid_jump_reject_count", a.centroid_jump_reject_count },
        { "passability_reject_count", a.passability_reject_count },
        { "r90_false_reject_count", a.r90_false_reject_count },
        { "residual_false_reject_count", a.residual_false_reject_count },
        { "centroid_jump_false_reject_count", a.centroid_jump_false_reject_count },
        { "passability_false_reject_count", a.passability_false_reject_count },
        { "gate_availability", a.gate_availability },
        { "longest_rejection_streak", a.longest_rejection_streak },
        { "cold_start_convergence_ticks", a.cold_start_convergence_ticks },
        { "cold_start_convergence_error", a.cold_start_convergence_error },
        { "worst_reacquisition_ticks", a.worst_reacquisition_ticks },
        { "mean_reacquisition_ticks", a.mean_reacquisition_ticks },
        { "reacquisition_count", a.reacquisition_count },
        { "post_convergence_error_stddev", a.post_convergence_error_stddev },
        { "peak_r90_at_acceptance", a.peak_r90_at_acceptance },
        { "neff_collapse_count", a.neff_collapse_count },
        { "sensor_dropout_rate", a.sensor_dropout_rate },
        { "total_ticks", a.total_ticks },
    };
}

void from_json(const nlohmann::json& j, SessionAnalytics& a) {
    a.peak_accepted_error = j.value("peak_accepted_error", 0.0);
    a.peak_accepted_error_tick = j.value("peak_accepted_error_tick", -1);
    a.mean_accepted_error = j.value("mean_accepted_error", 0.0);
    a.p95_accepted_error = j.value("p95_accepted_error", 0.0);
    a.oracle_threshold = j.value("oracle_threshold", 3.0);
    a.gate_precision = j.value("gate_precision", 0.0);
    a.gate_recall = j.value("gate_recall", 0.0);
    a.false_accept_count = j.value("false_accept_count", 0);
    a.false_accept_threshold = j.value("false_accept_threshold", 3.0);
    a.longest_false_accept_streak = j.value("longest_false_accept_streak", 0);
    a.mean_false_accept_duration = j.value("mean_false_accept_duration", 0.0);
    a.peak_error_during_false_accept = j.value("peak_error_during_false_accept", 0.0);
    a.false_accept_nearest_gate_r90 = j.value("false_accept_nearest_gate_r90", 0);
    a.false_accept_nearest_gate_residual = j.value("false_accept_nearest_gate_residual", 0);
    a.false_accept_nearest_gate_centroid_jump = j.value("false_accept_nearest_gate_centroid_jump", 0);
    a.false_reject_count = j.value("false_reject_count", 0);
    a.false_reject_threshold = j.value("false_reject_threshold", 3.0);
    a.r90_reject_count = j.value("r90_reject_count", 0);
    a.residual_reject_count = j.value("residual_reject_count", 0);
    a.centroid_jump_reject_count = j.value("centroid_jump_reject_count", 0);
    a.passability_reject_count = j.value("passability_reject_count", 0);
    a.r90_false_reject_count = j.value("r90_false_reject_count", 0);
    a.residual_false_reject_count = j.value("residual_false_reject_count", 0);
    a.centroid_jump_false_reject_count = j.value("centroid_jump_false_reject_count", 0);
    a.passability_false_reject_count = j.value("passability_false_reject_count", 0);
    a.gate_availability = j.value("gate_availability", 0.0);
    a.longest_rejection_streak = j.value("longest_rejection_streak", 0);
    a.cold_start_convergence_ticks = j.value("cold_start_convergence_ticks", -1);
    a.cold_start_convergence_error = j.value("cold_start_convergence_error", 0.0);
    a.worst_reacquisition_ticks = j.value("worst_reacquisition_ticks", -1);
    a.mean_reacquisition_ticks = j.value("mean_reacquisition_ticks", 0.0);
    a.reacquisition_count = j.value("reacquisition_count", 0);
    a.post_convergence_error_stddev = j.value("post_convergence_error_stddev", 0.0);
    a.peak_r90_at_acceptance = j.value("peak_r90_at_acceptance", 0.0);
    a.neff_collapse_count = j.value("neff_collapse_count", 0);
    a.sensor_dropout_rate = j.value("sensor_dropout_rate", 0.0);
    a.total_ticks = j.value("total_ticks", 0);
}

SessionAnalytics compute_analytics(
    const std::vector<TickState>& ticks,
    const mcl::GateConfig& gate_config,
    double false_accept_threshold,
    double false_reject_threshold,
    double oracle_threshold,
    int neff_collapse_particle_count) {

    SessionAnalytics out;
    out.false_accept_threshold = false_accept_threshold;
    out.false_reject_threshold = false_reject_threshold;
    out.oracle_threshold = oracle_threshold;
    out.total_ticks = static_cast<int>(ticks.size());
    if (ticks.empty()) return out;

    const double max_r90 = std::max(gate_config.max_radius_90_in, 1e-9);
    const double max_speed = std::max(gate_config.max_centroid_jump_ft_per_s, 1e-9);

    int accept_count = 0;
    int current_reject_streak = 0;
    int reject_streak_start_tick = 0;
    int current_fa_streak = 0;
    std::vector<int> fa_streak_lengths;
    int oracle_tp = 0;
    int oracle_fp = 0;
    int oracle_fn = 0;
    int total_sensor_slots = 0;
    int dropout_slots = 0;
    std::vector<double> accepted_errors;
    std::vector<std::pair<int, int>> loss_events;

    for (const auto& t : ticks) {
        const bool accepted = t.gate_decision.accepted;
        const bool oracle_accept = (t.mcl_error < oracle_threshold);

        if (accepted && oracle_accept) oracle_tp++;
        else if (accepted && !oracle_accept) oracle_fp++;
        else if (!accepted && oracle_accept) oracle_fn++;

        if (accepted) {
            accept_count++;
            accepted_errors.push_back(t.accepted_error);
            if (t.accepted_error > out.peak_accepted_error) {
                out.peak_accepted_error = t.accepted_error;
                out.peak_accepted_error_tick = t.tick;
            }
            if (t.cluster_stats.radius_90 > out.peak_r90_at_acceptance)
                out.peak_r90_at_acceptance = t.cluster_stats.radius_90;
        }

        if (!accepted) {
            if (current_reject_streak == 0) reject_streak_start_tick = t.tick;
            current_reject_streak++;
            if (current_reject_streak > out.longest_rejection_streak)
                out.longest_rejection_streak = current_reject_streak;
        } else {
            if (current_reject_streak >= 3) {
                loss_events.emplace_back(reject_streak_start_tick, current_reject_streak);
            }
            current_reject_streak = 0;
        }

        if (!accepted) {
            if (t.gate_decision.failed_r90) out.r90_reject_count++;
            else if (t.gate_decision.failed_residual) out.residual_reject_count++;
            else if (t.gate_decision.failed_centroid_jump) out.centroid_jump_reject_count++;
            else if (t.gate_decision.failed_passability) out.passability_reject_count++;
        }

        if (accepted && t.accepted_error > false_accept_threshold) {
            out.false_accept_count++;
            current_fa_streak++;
            if (t.accepted_error > out.peak_error_during_false_accept)
                out.peak_error_during_false_accept = t.accepted_error;

            const double r90_margin = safe_div(max_r90 - t.gate_decision.radius_90_in, max_r90);
            const double resid_margin =
                (t.gate_decision.residual_threshold_in > 1e-12)
                ? safe_div(
                      t.gate_decision.residual_threshold_in - t.gate_decision.max_sensor_residual_in,
                      t.gate_decision.residual_threshold_in)
                : 1.0;
            const double vel_margin =
                safe_div(max_speed - t.gate_decision.centroid_jump_ft_per_s, max_speed);

            if (r90_margin <= resid_margin && r90_margin <= vel_margin)
                out.false_accept_nearest_gate_r90++;
            else if (resid_margin <= vel_margin)
                out.false_accept_nearest_gate_residual++;
            else
                out.false_accept_nearest_gate_centroid_jump++;
        } else {
            if (current_fa_streak > 0) fa_streak_lengths.push_back(current_fa_streak);
            current_fa_streak = 0;
        }

        if (!accepted && t.mcl_error < false_reject_threshold
            && t.gate_decision.radius_90_in < gate_config.max_radius_90_in * 1.5) {
            out.false_reject_count++;
            if (t.gate_decision.failed_r90) out.r90_false_reject_count++;
            if (t.gate_decision.failed_residual) out.residual_false_reject_count++;
            if (t.gate_decision.failed_centroid_jump) out.centroid_jump_false_reject_count++;
            if (t.gate_decision.failed_passability) out.passability_false_reject_count++;
        }

        for (int s = 0; s < 4; ++s) {
            total_sensor_slots++;
            if (t.observed_readings[static_cast<size_t>(s)] < 0.0) dropout_slots++;
        }

        const double neff_floor = static_cast<double>(neff_collapse_particle_count) * 0.1;
        if (t.post_resample.n_eff < neff_floor) out.neff_collapse_count++;
    }

    if (current_fa_streak > 0) fa_streak_lengths.push_back(current_fa_streak);

    out.gate_availability = static_cast<double>(accept_count) / static_cast<double>(ticks.size());
    out.sensor_dropout_rate =
        (total_sensor_slots > 0) ? static_cast<double>(dropout_slots) / static_cast<double>(total_sensor_slots) : 0.0;

    const int tp_fp = oracle_tp + oracle_fp;
    const int tp_fn = oracle_tp + oracle_fn;
    out.gate_precision = (tp_fp > 0) ? static_cast<double>(oracle_tp) / static_cast<double>(tp_fp) : 1.0;
    out.gate_recall = (tp_fn > 0) ? static_cast<double>(oracle_tp) / static_cast<double>(tp_fn) : 1.0;

    if (!fa_streak_lengths.empty()) {
        out.longest_false_accept_streak =
            *std::max_element(fa_streak_lengths.begin(), fa_streak_lengths.end());
        double sum = 0.0;
        for (int l : fa_streak_lengths) sum += static_cast<double>(l);
        out.mean_false_accept_duration = sum / static_cast<double>(fa_streak_lengths.size());
    }

    if (!accepted_errors.empty()) {
        double sum = 0.0;
        for (double e : accepted_errors) sum += e;
        out.mean_accepted_error = sum / static_cast<double>(accepted_errors.size());

        std::sort(accepted_errors.begin(), accepted_errors.end());
        const size_t n = accepted_errors.size();
        size_t idx = static_cast<size_t>(std::floor(0.95 * static_cast<double>(n)));
        if (idx >= n) idx = n - 1;
        out.p95_accepted_error = accepted_errors[idx];
    }

    for (size_t i = 0; i < ticks.size(); ++i) {
        if (ticks[i].gate_decision.accepted && ticks[i].mcl_error < oracle_threshold) {
            out.cold_start_convergence_ticks = ticks[i].tick;
            out.cold_start_convergence_error = ticks[i].mcl_error;
            std::vector<double> window;
            for (size_t k = i; k < std::min(i + 50, ticks.size()); ++k) {
                if (ticks[k].gate_decision.accepted) window.push_back(ticks[k].accepted_error);
            }
            if (window.size() >= 2) {
                double mean = 0.0;
                for (double v : window) mean += v;
                mean /= static_cast<double>(window.size());
                double var = 0.0;
                for (double v : window) var += (v - mean) * (v - mean);
                out.post_convergence_error_stddev = std::sqrt(var / static_cast<double>(window.size()));
            }
            break;
        }
    }

    int total_reacq = 0;
    for (const auto& ev : loss_events) {
        const int loss_start = ev.first;
        const int loss_len = ev.second;
        const int resume_tick = loss_start + loss_len;
        bool found = false;
        for (size_t k = 0; k < ticks.size(); ++k) {
            if (ticks[k].tick >= resume_tick && ticks[k].gate_decision.accepted
                && ticks[k].mcl_error < false_reject_threshold) {
                const int reacq_ticks = ticks[k].tick - resume_tick;
                total_reacq += reacq_ticks;
                out.reacquisition_count++;
                if (out.worst_reacquisition_ticks < 0 || reacq_ticks > out.worst_reacquisition_ticks)
                    out.worst_reacquisition_ticks = reacq_ticks;
                found = true;
                break;
            }
        }
        (void)found;
    }
    if (out.reacquisition_count > 0)
        out.mean_reacquisition_ticks =
            static_cast<double>(total_reacq) / static_cast<double>(out.reacquisition_count);

    return out;
}

} // namespace state
