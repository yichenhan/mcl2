#pragma once

#include "mcl/mcl_controller.hpp"
#include "state/tick_state.hpp"

#include "nlohmann/json.hpp"

#include <array>
#include <utility>
#include <vector>

namespace state {

struct SessionAnalytics {
    double peak_accepted_error = 0.0;
    int peak_accepted_error_tick = -1;
    double mean_accepted_error = 0.0;
    double p95_accepted_error = 0.0;

    double oracle_threshold = 3.0;
    double gate_precision = 0.0;
    double gate_recall = 0.0;

    int false_accept_count = 0;
    double false_accept_threshold = 3.0;
    int longest_false_accept_streak = 0;
    double mean_false_accept_duration = 0.0;
    double peak_error_during_false_accept = 0.0;
    int false_accept_nearest_gate_r90 = 0;
    int false_accept_nearest_gate_residual = 0;
    int false_accept_nearest_gate_centroid_jump = 0;

    int false_reject_count = 0;
    double false_reject_threshold = 3.0;
    int r90_reject_count = 0;
    int residual_reject_count = 0;
    int centroid_jump_reject_count = 0;
    int passability_reject_count = 0;
    int r90_false_reject_count = 0;
    int residual_false_reject_count = 0;
    int centroid_jump_false_reject_count = 0;
    int passability_false_reject_count = 0;

    double gate_availability = 0.0;
    int longest_rejection_streak = 0;

    int cold_start_convergence_ticks = -1;
    double cold_start_convergence_error = 0.0;
    int worst_reacquisition_ticks = -1;
    double mean_reacquisition_ticks = 0.0;
    int reacquisition_count = 0;
    double post_convergence_error_stddev = 0.0;

    double peak_r90_at_acceptance = 0.0;
    int neff_collapse_count = 0;
    double sensor_dropout_rate = 0.0;
    int total_ticks = 0;
};

void to_json(nlohmann::json& j, const SessionAnalytics& a);
void from_json(const nlohmann::json& j, SessionAnalytics& a);

SessionAnalytics compute_analytics(
    const std::vector<TickState>& ticks,
    const mcl::GateConfig& gate_config = {},
    double false_accept_threshold = 3.0,
    double false_reject_threshold = 3.0,
    double oracle_threshold = 3.0,
    int neff_collapse_particle_count = 300);

// ---------------------------------------------------------------------------
// KPI rating infrastructure
// ---------------------------------------------------------------------------

enum class KpiId {
    ConvergenceLatency,  // cold_start_convergence_ticks; -1 = never converged
    PeakAcceptedError,   // peak_accepted_error (inches)
    GatePrecision,       // gate_precision [0,1]
    PostConvStdDev,      // post_convergence_error_stddev (inches)
    GateRecall,          // gate_recall [0,1]
    ReacqLatency,        // worst_reacquisition_ticks; -1 = no event (trivially great)
};

enum class KpiRating { Great, Good, Failing };

struct KpiThreshold {
    double good;            // boundary for "good" (inclusive)
    double great;           // boundary for "great" (inclusive, stricter than good)
    bool   lower_is_better;
};

// Single authoritative threshold table — update here to change everywhere.
inline const std::array<std::pair<KpiId, KpiThreshold>, 6>& kpi_thresholds() {
    static const std::array<std::pair<KpiId, KpiThreshold>, 6> t{{
        { KpiId::ConvergenceLatency, { 10.0, 4.0,  true  } },
        { KpiId::PeakAcceptedError,  {  6.0, 3.0,  true  } },
        { KpiId::GatePrecision,      {  0.90, 0.97, false } },
        { KpiId::PostConvStdDev,     {  2.0,  1.0,  true  } },
        { KpiId::GateRecall,         {  0.80, 0.92, false } },
        { KpiId::ReacqLatency,       { 15.0,  5.0,  true  } },
    }};
    return t;
}

// Extract the scalar value for a KpiId from a SessionAnalytics.
// Returns -1.0 for ReacqLatency when worst_reacquisition_ticks == -1
// (no loss event → handled as trivially great by rate_kpi).
inline double kpi_value(KpiId id, const SessionAnalytics& a) {
    switch (id) {
        case KpiId::ConvergenceLatency: return static_cast<double>(a.cold_start_convergence_ticks);
        case KpiId::PeakAcceptedError:  return a.peak_accepted_error;
        case KpiId::GatePrecision:      return a.gate_precision;
        case KpiId::PostConvStdDev:     return a.post_convergence_error_stddev;
        case KpiId::GateRecall:         return a.gate_recall;
        case KpiId::ReacqLatency:       return static_cast<double>(a.worst_reacquisition_ticks);
    }
    return 0.0;
}

// Rate a single KPI against its thresholds.
inline KpiRating rate_kpi(KpiId id, const SessionAnalytics& a) {
    // Special cases: "never happened" is trivially great.
    if (id == KpiId::ReacqLatency && a.worst_reacquisition_ticks < 0)
        return KpiRating::Great;
    // Never converged is always Failing for ConvergenceLatency.
    if (id == KpiId::ConvergenceLatency && a.cold_start_convergence_ticks < 0)
        return KpiRating::Failing;

    const double v = kpi_value(id, a);
    for (const auto& [kid, thresh] : kpi_thresholds()) {
        if (kid != id) continue;
        const bool is_great = thresh.lower_is_better ? (v <= thresh.great) : (v >= thresh.great);
        const bool is_good  = thresh.lower_is_better ? (v <= thresh.good)  : (v >= thresh.good);
        if (is_great) return KpiRating::Great;
        if (is_good)  return KpiRating::Good;
        return KpiRating::Failing;
    }
    return KpiRating::Failing;
}

// True only when every KpiId rates Great.
inline bool is_all_great(const SessionAnalytics& a) {
    for (const auto& [id, thresh] : kpi_thresholds()) {
        (void)thresh;
        if (rate_kpi(id, a) != KpiRating::Great) return false;
    }
    return true;
}

} // namespace state
